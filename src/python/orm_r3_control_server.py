import argparse
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlsplit

from orm_r3_control import ORMR3Control


class ORMR3ControlRequestHandler(BaseHTTPRequestHandler):
    control = None

    def do_GET(self):
        if urlsplit(self.path).path == "/pose":
            response = self._pose_response(
                self.control.end_effector_position,
                self.control.end_effector_euler_angles,
            )
            self._send_json(200, response)
        else:
            self._send_json(404, {"error": "Route not found"})

    def do_OPTIONS(self):
        self.send_response(204)
        self._send_cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_POST(self):
        try:
            data = self._read_json_body()
            path = urlsplit(self.path).path

            if path == "/set_joints_angles":
                response = self._set_joints_angles(data)
            elif path == "/set_pose":
                response = self._set_pose(data)
            else:
                self._send_json(404, {"error": "Route not found"})
                return

            self._send_json(200, response)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {"error": str(error)})
        except Exception as error:
            self.log_error("Request failed: %s", error)
            self._send_json(500, {"error": "Internal server error"})

    def _read_json_body(self):
        content_length = self.headers.get("Content-Length")
        if content_length is None:
            raise ValueError("Content-Length header is required")

        body = self.rfile.read(int(content_length))
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
    return parser.parse_args()


def main():
    args = parse_arguments()
    control = ORMR3Control(args.serial_port)
    ORMR3ControlRequestHandler.control = control
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
