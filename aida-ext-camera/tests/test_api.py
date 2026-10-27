import importlib
import json
import tempfile
import time
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
from fastapi.testclient import TestClient

from calibration import Board


class ApiTests(unittest.TestCase):
    def test_usb3_formats_include_yuyv_and_4k_mjpeg(self):
        from app import parse_camera_modes
        modes = parse_camera_modes("""
        [0]: 'YUYV' (YUYV 4:2:2)
          Size: Discrete 640x360
          Size: Discrete 1920x1080
        [1]: 'MJPG' (Motion-JPEG, compressed)
          Size: Discrete 3840x2160
        """)
        self.assertEqual(modes, [{"width": 3840, "height": 2160, "fourcc": "MJPG"},
                                 {"width": 1920, "height": 1080, "fourcc": "YUYV"},
                                 {"width": 640, "height": 360, "fourcc": "YUYV"}])
        self.assertEqual(parse_camera_modes(""), [])

    def test_legacy_corners_reprocessed_without_changing_raw_image(self):
        photo = Path("image copy.png")
        if not photo.exists():
            self.skipTest("Reference image is not mounted")
        module = importlib.import_module("app")
        with tempfile.TemporaryDirectory() as directory, patch.object(module, "DATA", Path(directory)):
            data = Path(directory)
            session = data / "sessions" / "legacy"
            session.mkdir(parents=True)
            raw = photo.read_bytes()
            (session / "frame.png").write_bytes(raw)
            image = cv2.imread(str(photo))
            old_board = asdict(Board())
            old_board.pop("border_bits")
            old = {"session": "legacy", "board": old_board, "confirmed": True,
                   "samples": [{"id": "frame", "tags": 27}],
                   "image_size": [image.shape[1], image.shape[0]], "camera": {}, "result": None}
            (data / "current.json").write_text(json.dumps(old))
            with TestClient(module.app) as client:
                state = client.get("/api/state").json()
                self.assertEqual(state["board"]["border_bits"], 2)
                self.assertEqual(len(state["samples"]), 1)
                self.assertGreaterEqual(state["samples"][0]["tags"], 80)
                self.assertEqual(state["migration"]["frames"][0]["old_tags"], 27)
                self.assertEqual((session / "frame.png").read_bytes(), raw)
                self.assertTrue((session / "session_before_detector_v2.json").exists())

    def test_capture_calibration_export_restore_and_reset(self):
        module = importlib.import_module("app")
        with tempfile.TemporaryDirectory() as directory, patch.object(module, "DATA", Path(directory)), patch.object(module, "optics", return_value={}):
            with TestClient(module.app) as client:
                state = client.get("/api/state").json()
                self.assertFalse(state["confirmed"])
                self.assertEqual(client.post("/api/samples").status_code, 400)
                self.assertEqual(client.post("/api/calibrate").status_code, 400)
                self.assertEqual(client.post("/api/board", json={"tag_size_mm": -1}).status_code, 422)
                board = Board()
                response = client.post("/api/board", json={**asdict(board), "confirmed": True})
                self.assertEqual(response.status_code, 200, response.text)
                studio = module.studio
                # Synthetic observations exercise the HTTP/storage/calculation
                # flow independently of USB hardware and camera motion.
                studio.camera.thread = type("AliveThread", (), {"is_alive": lambda self: True, "join": lambda self, timeout: None})()
                studio.camera.meta = {"name": "synthetic", "id": module.CAMERA_ID, "width": 1920, "height": 1080}
                objects = np.concatenate([board.points(i) for i in range(88)])
                bw, bh = board.size
                board_quad = np.float32([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]])
                k = np.array([[1150., 0, 960], [0, 1140., 540], [0, 0, 1]])
                rng = np.random.default_rng(12)
                saved = 0
                for i in range(40):
                    r = rng.uniform(-.5, .5, 3)
                    t = np.array([rng.uniform(-400, 0), rng.uniform(-250, 0), rng.uniform(650, 1000)])
                    pixels = cv2.projectPoints(objects, r, t, k, np.zeros(5))[0].reshape(-1, 2)
                    polygon = cv2.projectPoints(board_quad, r, t, k, np.zeros(5))[0].reshape(-1, 2)
                    studio.latest = {"timestamp": time.monotonic(), "image_size": (1920, 1080),
                                     "quality": {"ready": True, "tags": 88, "sharpness": 500},
                                     "object_points": objects, "image_points": pixels, "polygon": polygon,
                                     "frame": np.full((1080, 1920, 3), 220, np.uint8)}
                    response = client.post("/api/samples")
                    if response.status_code == 200:
                        saved += 1
                    if saved == 15:
                        break
                self.assertEqual(saved, 15)
                self.assertEqual(client.post("/api/samples").status_code, 400)  # duplicate
                changed = {**asdict(board), "tag_size_mm": 26, "confirmed": True}
                self.assertEqual(client.post("/api/board", json=changed).status_code, 400)
                self.assertEqual(client.post("/api/calibrate").status_code, 200)
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    state = client.get("/api/state").json()
                    if not state["job"]["running"]:
                        break
                    time.sleep(.05)
                self.assertIsNone(state["job"]["error"], state["job"])
                self.assertIsNotNone(state["result"])
                self.assertLess(state["result"]["rms_px"], .01)
                exported = client.get("/api/export/json")
                self.assertEqual(exported.status_code, 200)
                self.assertEqual(exported.json()["image_width"], 1920)
                yaml = client.get("/api/export/yaml")
                self.assertEqual(yaml.status_code, 200)
                self.assertIn("camera_matrix", yaml.text)
                sample_id = state["samples"][0]["id"]
                self.assertEqual(client.get(f"/api/samples/{sample_id}/thumbnail").status_code, 200)
                old_directory = studio.directory
                # Replace test stub before lifecycle shutdown.
                studio.camera.thread = None
            with TestClient(module.app) as client:
                state = client.get("/api/state").json()
                self.assertEqual(len(state["samples"]), 15)
                self.assertIsNotNone(state["result"])
                self.assertEqual(client.delete(f"/api/samples/{sample_id}").status_code, 200)
                self.assertEqual(client.get("/api/export/json").status_code, 404)
                response = client.post("/api/session/new")
                self.assertEqual(response.json()["samples"], [])
                self.assertTrue((old_directory / "intrinsics.json").exists())


if __name__ == "__main__":
    unittest.main()
