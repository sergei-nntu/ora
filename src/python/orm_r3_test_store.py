"""Filesystem storage for repeatability test sessions (single server writer)."""
import base64
import binascii
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


class TestSessionExists(Exception):
    pass


class TestSessionNotFound(Exception):
    pass


class TestSessionStore:
    def __init__(self, root):
        self.root = Path(root).resolve()

    def _session_path(self, test_id):
        if not isinstance(test_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9 _.-]{0,99}", test_id):
            raise ValueError("test_id must be 1–100 characters: letters, digits, spaces, _, . or -, starting with a letter or digit")
        if test_id.endswith((" ", ".")):
            raise ValueError("test_id must not end with a space or dot")
        path = self.root / test_id
        if path.is_symlink():
            raise ValueError("Session directory must not be a symbolic link")
        return path

    def list_sessions(self):
        if not self.root.exists():
            return []
        sessions = []
        for folder in self.root.iterdir():
            try:
                path = self._session_path(folder.name)
            except ValueError:
                continue
            records = path / "test_records.json"
            if path.is_dir() and records.is_file() and not records.is_symlink():
                sessions.append(folder.name)
        return sorted(sessions)

    def records(self, test_id):
        path = self._session_path(test_id) / "test_records.json"
        if path.is_symlink():
            raise ValueError("Records file must not be a symbolic link")
        if not path.is_file():
            raise TestSessionNotFound("Test session does not exist")
        records = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(records, list):
            raise ValueError("Invalid session records file")
        return records

    def image(self, test_id, filename):
        if not isinstance(filename, str) or not re.fullmatch(r"[a-f0-9]{32}\.(png|jpg)", filename):
            raise ValueError("Invalid image filename")
        if not any(record.get("image") == filename for record in self.records(test_id)):
            raise TestSessionNotFound("Image is not part of this session")
        path = self._session_path(test_id) / filename
        if path.is_symlink():
            raise ValueError("Image must not be a symbolic link")
        if not path.is_file():
            raise TestSessionNotFound("Image does not exist")
        return path.read_bytes(), "image/png" if path.suffix == ".png" else "image/jpeg"

    @staticmethod
    def _write_json(path, data):
        temporary = path.with_name(f".{uuid4().hex}.json.tmp")
        try:
            temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)

    def calibration(self, test_id):
        self.records(test_id)
        path = self._session_path(test_id) / "calibration.json"
        if path.is_symlink():
            raise ValueError("Calibration must not be a symbolic link")
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None

    def annotate(self, data, scale=False):
        records = self.records(data["test_id"])
        record = next((item for item in records if item["record_id"] == data["record_id"]), None)
        if record is None:
            raise TestSessionNotFound("State does not exist")
        width, height = (self._number(data[key]) for key in ("image_width", "image_height"))
        x, y = (self._number(data[key]) for key in ("x", "y"))
        if width <= 0 or height <= 0 or not 0 <= x < width or not 0 <= y < height:
            raise ValueError("Annotation must be inside the image")
        annotation = {"x": x, "y": y, "image_width": width, "image_height": height}
        folder = self._session_path(data["test_id"])
        if scale:
            size = self._number(data["size_pixels"])
            if size <= 0 or x + size > width or y + size > height:
                raise ValueError("Calibration square must fit inside the image")
            annotation.update(size_pixels=size, size_mm=100, mm_per_pixel=100 / size,
                              record_id=data["record_id"])
            self._write_json(folder / "calibration.json", annotation)
        else:
            record["end_effector_pixel"] = annotation
            self._write_json(folder / "test_records.json", records)
        return annotation

    def start(self, test_id):
        folder = self._session_path(test_id)
        self.root.mkdir(parents=True, exist_ok=True)
        try:
            folder.mkdir()
        except FileExistsError:
            raise TestSessionExists("Test session already exists")
        try:
            (folder / "test_records.json").write_text("[]\n", encoding="utf-8")
        except Exception:
            # Do not leave an apparently valid session after failed creation.
            (folder / "test_records.json").unlink(missing_ok=True)
            folder.rmdir()
            raise
        return {"test_id": test_id, "records_file": "test_records.json"}

    @staticmethod
    def _number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("Coordinates and angles must be finite numbers")
        return value

    def add_state(self, data):
        test_id = data["test_id"]
        folder = self._session_path(test_id)
        records_path = folder / "test_records.json"
        if not folder.is_dir() or not records_path.is_file():
            raise TestSessionNotFound("Test session does not exist; call /test/start first")
        if records_path.is_symlink():
            raise ValueError("Records file must not be a symbolic link")
        position_id = data["position_id"]
        if isinstance(position_id, bool) or not isinstance(position_id, (str, int)) or position_id == "":
            raise ValueError("position_id must be a nonempty string or integer")
        coordinates = data["desired_cartesian_coordinates"]
        if not isinstance(coordinates, dict):
            raise ValueError("desired_cartesian_coordinates must contain x, y, z in metres")
        coordinates = {axis: self._number(coordinates[axis]) for axis in ("x", "y", "z")}
        angles = {}
        for key in ("desired_joint_angles", "actual_joint_angles"):
            values = data[key]
            if not isinstance(values, list) or len(values) != 6:
                raise ValueError(f"{key} must contain six angles in degrees, in joint address order")
            angles[key] = [None if key == "actual_joint_angles" and value is None
                           else self._number(value) for value in values]
        image = data["image"]
        if not isinstance(image, str):
            raise ValueError("image must be a PNG or JPEG base64 data URL")
        header, separator, encoded = image.partition(",")
        formats = {"data:image/png;base64": ("png", b"\x89PNG\r\n\x1a\n"),
                   "data:image/jpeg;base64": ("jpg", b"\xff\xd8\xff")}
        if not separator or header not in formats:
            raise ValueError("image must be a PNG or JPEG base64 data URL")
        try:
            content = base64.b64decode(encoded, validate=True)
        except (ValueError, binascii.Error):
            raise ValueError("Invalid image base64")
        extension, signature = formats[header]
        if not content.startswith(signature):
            raise ValueError("Image content does not match its PNG/JPEG type")
        records = json.loads(records_path.read_text(encoding="utf-8"))
        if not isinstance(records, list):
            raise ValueError("Invalid session records file")
        record_id = uuid4().hex
        filename = f"{record_id}.{extension}"
        record = {"record_id": record_id, "position_id": position_id,
                  "timestamp": datetime.now(timezone.utc).isoformat(), "image": filename,
                  "desired_cartesian_coordinates": coordinates, **angles,
                  "units": {"cartesian": "metres", "joint_angles": "degrees"}}
        image_path = folder / filename
        temporary = folder / f".{record_id}.json.tmp"
        try:
            with image_path.open("xb") as output:
                output.write(content)
            temporary.write_text(json.dumps(records + [record], indent=2, allow_nan=False) + "\n", encoding="utf-8")
            temporary.replace(records_path)
        except Exception:
            image_path.unlink(missing_ok=True)
            raise
        finally:
            temporary.unlink(missing_ok=True)
        return {"test_id": test_id, "record": record, "record_count": len(records) + 1}
