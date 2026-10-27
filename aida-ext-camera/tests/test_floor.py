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

from calibration import Board, detect
from floor_geometry import aggregate, fit_floor, pixel_to_floor, reference_transform


def synthetic_floor(noise=.12):
    board = Board()
    k = np.array([[1900., 0, 1920], [0, 1910., 1080], [0, 0, 1]])
    d = np.array([-.05, .01, .0001, -.0002, .002])
    centre = np.array([0., -2., 2.5])
    forward = np.array([0., 2.3, -2.5]); forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.array([right, down, forward])
    translation = -rotation @ centre
    intrinsics = {"camera_matrix": k.tolist(), "distortion_coefficients": d.tolist(),
                  "image_width": 3840, "image_height": 2160, "rms_px": .4, "board": asdict(board),
                  "per_view_errors": [],
                  "camera": {"name": "synthetic", "optical_controls": {}}}
    objects = np.concatenate([board.points(i) for i in range(88)])/1000
    reference = reference_transform(board, .002)
    samples = []
    rng = np.random.default_rng(7)
    placements = [(0, 0, 0), (-.8, .4, .3), (.8, .3, -.2), (-.6, 1.1, -.5),
                  (.7, 1.3, .6), (0, -.5, .2), (.2, .8, -.7), (-.4, -.3, .9)]
    for i, (x, y, yaw) in enumerate(placements):
        points = objects @ reference[:3, :3].T + reference[:3, 3]
        c, s = np.cos(yaw), np.sin(yaw)
        points[:, :2] = points[:, :2] @ np.array([[c, s], [-s, c]]) + [x, y]
        pixels = cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], translation, k, d)[0][:, 0]
        pixels += rng.normal(0, noise, pixels.shape)
        samples.append({"id": str(i), "object_points": (objects*1000).tolist(), "image_points": pixels.tolist(),
                        "tags": 88, "ids": list(range(88)), "jitter_px": .1, "burst_frames": 5, "pose_rms_px": .2})
    return samples, intrinsics, board, centre, rotation, translation


class FloorGeometryTests(unittest.TestCase):
    def test_freely_moving_board_recovers_camera_and_floor_points(self):
        samples, intrinsics, board, centre, rotation, translation = synthetic_floor()
        result = fit_floor(samples, intrinsics, board, thickness_mm=2)
        self.assertTrue(result["valid"], result["warnings"])
        self.assertLess(result["rms_px"], .3)
        self.assertLess(np.linalg.norm(np.array(result["camera_position_floor_m"])-centre), .02)
        self.assertLess(abs(result["camera_height_m"]-2.5), .015)
        matrix = np.array(result["T_camera_from_floor"])
        np.testing.assert_allclose(matrix @ np.array(result["T_floor_from_camera"]), np.eye(4), atol=1e-8)
        for height in (0., .25):
            expected = np.array([.4, .7, height])
            pixel = cv2.projectPoints(expected[None], cv2.Rodrigues(rotation)[0], translation,
                                      np.array(intrinsics["camera_matrix"]), np.array(intrinsics["distortion_coefficients"]))[0][0, 0]
            measured = pixel_to_floor(pixel, result, height)
            self.assertLess(np.linalg.norm(np.array([measured["x_m"], measured["y_m"]])-expected[:2]), .02)
        json.dumps(result, allow_nan=False)

    def test_stationary_burst_averaging_rejects_motion(self):
        samples, _, _, _, _, _ = synthetic_floor()
        original = np.array(samples[0]["image_points"])
        objects = np.array(samples[0]["object_points"])
        frames = [{"timestamp": i*.3, "quality": {"ids": list(range(88)), "tags": 88},
                   "image_points": original+.1*i, "object_points": objects} for i in range(3)]
        combined = aggregate(frames)
        self.assertEqual(combined["burst_frames"], 3)
        np.testing.assert_allclose(combined["image_points"], original+.1)
        frames[-1]["image_points"] += 8
        with self.assertRaises(ValueError):
            aggregate(frames)

    def test_small_tags_detected_in_floor_profile(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        # Twelve 14-pixel tags, rendered at higher resolution then downsampled.
        high = np.full((220*4, 350*4, 3), 240, np.uint8)
        for tag_id in range(12):
            row, col = divmod(tag_id, 4)
            x, y = (110+col*19)*4, (70+row*19)*4
            tag = cv2.aruco.generateImageMarker(dictionary, tag_id, 14*4, borderBits=2)
            high[y:y+56, x:x+56] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
        image = cv2.resize(high, (350, 220), interpolation=cv2.INTER_AREA)
        board = Board(columns=4, rows=3, rotation=0, ids_bottom_up=False, gap_mm=25*5/14)
        normal = detect(image, board)
        distant = detect(image, board, distant=True)
        self.assertEqual(normal["quality"]["tags"], 0)
        self.assertGreaterEqual(distant["quality"]["tags"], 6, distant["quality"])
        cropped = detect(image, board, distant=True, search_roi=(60, 30, 180, 150))
        self.assertEqual(cropped["image_size"], (350, 220))
        shared = set(cropped["quality"]["ids"]) & set(distant["quality"]["ids"])
        self.assertGreaterEqual(len(shared), 6)
        for tag in shared:
            original_index = distant["quality"]["ids"].index(tag)
            cropped_index = cropped["quality"]["ids"].index(tag)
            original_points = distant["image_points"].reshape(-1, 4, 2)[original_index]
            cropped_points = cropped["image_points"].reshape(-1, 4, 2)[cropped_index]
            self.assertLess(np.linalg.norm(original_points-cropped_points, axis=1).max(), 1)

    def test_floor_accepts_a_single_corrupted_code_bit(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image = np.full((300, 440, 3), 240, np.uint8)
        for tag_id in range(6):
            row, col = divmod(tag_id, 3)
            x, y = 35+col*130, 35+row*130
            tag = cv2.aruco.generateImageMarker(dictionary, tag_id, 100, borderBits=2)
            tag[40:50, 40:50] = 255-tag[40:50, 40:50]
            image[y:y+100, x:x+100] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
        board = Board(columns=3, rows=2, rotation=0, ids_bottom_up=False)
        result = detect(image, board, distant=True)
        self.assertEqual(result["quality"]["ids"], list(range(6)))

    def test_connected_grid_outline_recovers_measured_tag_edges(self):
        from tag_edges import recover_grid
        board = Board(rotation=0, ids_bottom_up=False)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image = np.full((700, 1000), 230, np.uint8)
        for tag_id in range(88):
            row, col = divmod(tag_id, 11)
            x, y = 100+col*39, 100+row*39
            image[y:y+30, x:x+30] = cv2.aruco.generateImageMarker(dictionary, tag_id, 30, borderBits=2)
        for row in range(9):
            for col in range(12):
                x, y = 100+col*39-9, 100+row*39-9
                image[y:y+9, x:x+9] = 0
        found = recover_grid(image, board, [])
        self.assertGreaterEqual(len(found), 80)
        for tag_id, corners in found.items():
            row, col = divmod(tag_id, 11)
            x, y = 100+col*39, 100+row*39
            expected = np.array([[x-.5, y-.5], [x+29.5, y-.5], [x+29.5, y+29.5], [x-.5, y+29.5]])
            self.assertLess(np.linalg.norm(corners-expected, axis=1).max(), .6)


class FloorApiTests(unittest.TestCase):
    def test_new_floor_preserves_intrinsics_and_exports_transforms(self):
        import app as module
        samples, intrinsics, board, _, _, _ = synthetic_floor()
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, "DATA", Path(temporary)), patch.object(module, "optics", return_value={}):
            with TestClient(module.app) as client:
                module.studio.result = intrinsics
                module.studio.board = board
                module.studio.samples = [{"id": "intrinsics-frame", "tags": 88, "sharpness": 100}]
                response = client.post("/api/mode", json={"mode": "floor"})
                self.assertEqual(response.status_code, 200, response.text)
                self.assertEqual(len(module.studio.samples), 1)
                self.assertEqual(client.post("/api/floor/calibrate").status_code, 400)
                class FakeThread:
                    stopped = False
                    def is_alive(self):
                        return not self.stopped
                    def join(self, timeout=None):
                        self.stopped = True
                module.studio.camera.thread = FakeThread()
                module.studio.camera.meta = {"name": "synthetic", "width": 3840, "height": 2160}
                now = time.monotonic()
                for index in range(3):
                    observation = {"timestamp": now-.6+index*.3, "image_size": (3840, 2160),
                                   "quality": {"ready": True, "ids": list(range(88)), "tags": 88},
                                   "object_points": np.array(samples[0]["object_points"]),
                                   "image_points": np.array(samples[0]["image_points"]),
                                   "frame": np.full((2160, 3840, 3), 220, np.uint8)}
                    module.studio.floor.update(observation)
                response = client.post("/api/floor/samples")
                self.assertEqual(response.status_code, 200, response.text)
                self.assertEqual(len(response.json()["samples"]), 1)
                self.assertEqual(client.post("/api/floor/samples").status_code, 400)
                module.studio.floor.samples = samples
                module.studio.floor.thickness_mm = 2
                response = client.post("/api/floor/calibrate")
                self.assertEqual(response.status_code, 200, response.text)
                deadline = time.monotonic()+30
                while time.monotonic() < deadline:
                    state = client.get("/api/floor/state").json()
                    if not state["running"]:
                        break
                    time.sleep(.1)
                self.assertEqual(state["error"], "")
                self.assertTrue(state["result"]["valid"])
                self.assertEqual(client.get("/api/floor/export/json").status_code, 200)
                self.assertIn("T_floor_from_camera", client.get("/api/floor/export/yaml").text)
                self.assertEqual(client.delete("/api/floor/samples/0").status_code, 400)
                self.assertEqual(client.post("/api/floor/new").status_code, 200)
                self.assertEqual(len(module.studio.samples), 1)
                self.assertIsNotNone(module.studio.result)


if __name__ == "__main__":
    unittest.main()
