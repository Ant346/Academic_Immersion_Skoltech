import unittest
from pathlib import Path

import cv2
import numpy as np

from calibration import Board, calibrate, detect, too_similar


class CalibrationTests(unittest.TestCase):
    def test_known_intrinsics_recovered_from_different_tilts(self):
        rng = np.random.default_rng(42)
        board = Board()
        objects = np.concatenate([board.points(i) for i in range(88)])
        matrix = np.array([[1150., 0, 960], [0, 1140., 540], [0, 0, 1]])
        distortion = np.array([-.12, .04, .001, -.0005, .005])
        samples = []
        for i in range(28):
            rotation = rng.uniform([-.6, -.6, -.25], [.6, .6, .25])
            translation = np.array([-175+rng.uniform(-180, 180), -125+rng.uniform(-130, 130), rng.uniform(650, 1050)])
            pixels, _ = cv2.projectPoints(objects, rotation, translation, matrix, distortion)
            pixels = pixels.reshape(-1, 2) + rng.normal(0, .06, (len(objects), 2))
            samples.append({"id": str(i), "object_points": objects.tolist(), "image_points": pixels.tolist()})
        result = calibrate(samples, [1920, 1080], board, {"name": "synthetic"})
        recovered = np.array(result["camera_matrix"])
        self.assertLess(abs(recovered[0, 0]-1150)/1150, .01)
        self.assertLess(abs(recovered[1, 1]-1140)/1140, .01)
        self.assertLess(abs(recovered[0, 2]-960), 3)
        self.assertLess(result["rms_px"], .15)
        self.assertEqual(len(result["per_view_errors"]), 28)
        self.assertEqual(result["distortion_order"], ["k1", "k2", "p1", "p2", "k3"])

    def test_supplied_photo_has_correct_corner_order(self):
        photo = Path("image copy.png")
        if not photo.exists():
            self.skipTest("Reference photo is not mounted")
        image = cv2.imread(str(photo))
        observation = detect(image, Board())
        self.assertGreaterEqual(observation["quality"]["tags"], 80)
        self.assertTrue(observation["quality"]["ready"], observation["quality"])
        self.assertLess(observation["quality"]["layout_error_px"], 1.5)
        wrong = detect(image, Board(rotation=0))
        self.assertFalse(wrong["quality"]["ready"], wrong["quality"])

    def test_duplicate_pose_and_shifted_pose(self):
        observation = {"polygon": np.float32([[100, 100], [600, 100], [600, 500], [100, 500]]),
                       "image_size": (1920, 1080)}
        samples = [{"polygon": observation["polygon"].tolist()}]
        self.assertTrue(too_similar(observation, samples))
        observation["polygon"] += [120, 0]
        self.assertFalse(too_similar(observation, samples))

    def test_blank_image_and_insufficient_observations(self):
        self.assertFalse(detect(np.full((480, 640, 3), 255, np.uint8), Board())["quality"]["ready"])
        with self.assertRaises(ValueError):
            calibrate([], [1920, 1080], Board(), {})

    def test_geometry_validation(self):
        with self.assertRaises(ValueError):
            Board.parse({"columns": 30, "rows": 30})
        with self.assertRaises(ValueError):
            Board.parse({"gap_mm": 0})
        points = Board().points(0)
        self.assertTrue(np.array_equal(points[0], [25, 252.5, 0]))

    def test_two_module_border_and_measured_outer_corners(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image = np.full((320, 450, 3), 240, np.uint8)
        expected = []
        for tag_id in range(6):
            row, col = divmod(tag_id, 3)
            x, y = 40+col*130, 40+row*130
            tag = cv2.aruco.generateImageMarker(dictionary, tag_id, 100, borderBits=2)
            image[y:y+100, x:x+100] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
            expected.append([[x-.5, y-.5], [x+99.5, y-.5], [x+99.5, y+99.5], [x-.5, y+99.5]])
        observation = detect(image, Board(columns=3, rows=2, rotation=0, ids_bottom_up=False))
        self.assertEqual(observation["quality"]["ids"], list(range(6)))
        error = np.linalg.norm(observation["image_points"].reshape(-1, 4, 2)-np.array(expected), axis=2)
        self.assertLess(float(error.max()), .4)

    def test_uniform_image_cannot_generate_a_refined_quad(self):
        from tag_edges import refine_quad, matches_code
        gray = np.full((200, 200), 180, np.uint8)
        corners = np.float32([[20, 20], [150, 20], [150, 150], [20, 150]])
        self.assertIsNone(refine_quad(gray.astype(np.float32), corners))
        self.assertFalse(matches_code(gray, corners, 0, 2))


if __name__ == "__main__":
    unittest.main()
