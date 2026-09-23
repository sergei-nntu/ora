import argparse
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlsplit, parse_qs

from orm_r3_control import ORMR3Control
from orm_r3_test_store import TestSessionStore, TestSessionExists, TestSessionNotFound


class ORMR3ControlRequestHandler(BaseHTTPRequestHandler):
    control = None
    test_store = TestSessionStore(Path(__file__).with_name("test_sessions"))

    def do_GET(self):
        if urlsplit(self.path).path.startswith("/test/"):
            self._get_test()
            return
        if urlsplit(self.path).path == "/joint_angles":
            self._send_json(200, self.control.get_joint_angles())
        elif urlsplit(self.path).path == "/pose":
            response = self._pose_response(
                self.control.end_effector_position,
                self.control.end_effector_euler_angles,
            )
            self._send_json(200, response)
        else:
            self._send_json(404, {"error": "Route not found"})

    def _get_test(self):
        url = urlsplit(self.path)
        query = parse_qs(url.query)
        try:
            if url.path == "/test/sessions":
                self._send_json(200, {"sessions": self.test_store.list_sessions()})
            elif url.path == "/test/states":
                test_id = query["test_id"][0]
                self._send_json(200, {"test_id": test_id, "records": self.test_store.records(test_id), "calibration": self.test_store.calibration(test_id)})
            elif url.path == "/test/image":
                content, content_type = self.test_store.image(query["test_id"][0], query["filename"][0])
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self._send_cors_headers()
                self.end_headers()
                self.wfile.write(content)
            else:
                self._send_json(404, {"error": "Route not found"})
        except TestSessionNotFound as error:
            self._send_json(404, {"error": str(error)})
        except (KeyError, ValueError, TypeError) as error:
            self._send_json(400, {"error": str(error)})
        except Exception as error:
            self.log_error("Test read failed: %s", error)
            self._send_json(500, {"error": "Internal server error"})

    def do_OPTIONS(self):
        self.send_response(204)
        self._send_cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_POST(self):
        try:
            data = self._read_json_body()
            path = urlsplit(self.path).path

            if path in ("/test/pose_estimate", "/test/scale"):
                response = self.test_store.annotate(data, scale=path == "/test/scale")
            elif path == "/test/start":
                response = self.test_store.start(data["test_id"])
                self._send_json(201, response)
                return
            elif path == "/test/state":
                response = self.test_store.add_state(data)
                self._send_json(201, response)
                return
            elif path == "/set_joints_angles":
                response = self._set_joints_angles(data)
            elif path == "/set_joint_angle":
                position, euler_angles = self.control.set_joint_angle(data["address"], data["angle"])
                response = self._pose_response(position, euler_angles)
            elif path == "/set_pose":
                response = self._set_pose(data)
            else:
                self._send_json(404, {"error": "Route not found"})
                return

            self._send_json(200, response)
        except TestSessionExists as error:
            self._send_json(409, {"error": str(error)})
        except TestSessionNotFound as error:
            self._send_json(404, {"error": str(error)})
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {"error": str(error)})
        except Exception as error:
            self.log_error("Request failed: %s", error)
            self._send_json(500, {"error": "Internal server error"})

    def _read_json_body(self):
        content_length = self.headers.get("Content-Length")
        if content_length is None:
            raise ValueError("Content-Length header is required")

        length = int(content_length)
        if not 0 < length <= 20 * 1024 * 1024:
            raise ValueError("Request body must be between 1 byte and 20 MiB")
        body = self.rfile.read(length)
        data = json.loads(body)
        if not isinstance(data, dict):
            raise TypeError("Request body must be a JSON object")
        return data

    def _set_joints_angles(self, data):
        position, euler_angles = self.control.set_joints_angles(data["angles"])
        return self._pose_response(position, euler_angles)

    def _set_pose(self, data):
        target_angles, position, euler_angles = self.control.set_pose(
            x=data["x"],
            y=data["y"],
            z=data["z"],
            pitch=data.get("pitch"),
            roll=data.get("roll"),
            yaw=data.get("yaw"),
            command=data.get("command", "set_angle"),
        )
        response = self._pose_response(position, euler_angles)
        response["target_angles"] = target_angles.tolist()
        return response

    @staticmethod
    def _pose_response(position, euler_angles):
        x, y, z = (float(value) for value in position)
        roll, pitch, yaw = (float(value) for value in euler_angles)
        return {
            "position": {"x": x, "y": y, "z": z},
            "euler_angles": {
                "pitch": pitch,
                "roll": roll,
                "yaw": yaw,
            },
        }

    def _send_json(self, status, data):
        response = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(response)))
        self._send_cors_headers()
        self.end_headers()
        self.wfile.write(response)

    def _send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")


def parse_arguments():
    parser = argparse.ArgumentParser(description="ORM R3 control HTTP server")
    parser.add_argument(
        "serial_port",
        nargs="?",
        default=None,
        help="Optional serial device, e.g. /dev/ttyUSB0",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Listening address")
    parser.add_argument("--port", default=8000, type=int, help="Listening port")
    parser.add_argument("--test-dir", type=Path, default=Path(__file__).with_name("test_sessions"),
                        help="Directory for repeatability test sessions")
    return parser.parse_args()


def main():
    args = parse_arguments()
    control = ORMR3Control(args.serial_port)
    ORMR3ControlRequestHandler.control = control
    ORMR3ControlRequestHandler.test_store = TestSessionStore(args.test_dir)
    server = HTTPServer((args.host, args.port), ORMR3ControlRequestHandler)

    print(f"ORM R3 control server listening on {args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping ORM R3 control server")
    finally:
        server.server_close()
        if control.osp is not None:
            control.osp.stop()


if __name__ == "__main__":
    main()
