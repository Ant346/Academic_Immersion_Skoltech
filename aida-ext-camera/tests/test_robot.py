import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from calibration import Board
from robot_tracking import (RobotSession, free_pose, board_plane_points, board_pose, fit_drive, fit_spin,
                            floor_footprint, project, rotation)


def synthetic_floor():
    """A camera 1.8 m high looking down at ~45 degrees, like the real setup."""
    k = [[2400., 0, 1920], [0, 2400., 1080], [0, 0, 1]]
    pitch = math.radians(135)
    camera_from_floor = np.eye(4)
    camera_from_floor[:3, :3] = cv2.Rodrigues(np.array([pitch, 0, 0]))[0]
    camera_position = np.array([0., -1., 1.8])
    camera_from_floor[:3, 3] = -camera_from_floor[:3, :3] @ camera_position
    return {"intrinsics": {"camera_matrix": k, "distortion_coefficients": [.05, -.02, 0, 0, 0],
                           "image_width": 3840, "image_height": 2160},
            "T_floor_from_camera": np.linalg.inv(camera_from_floor).tolist(),
            "camera_position_floor_m": camera_position.tolist(), "reference_id": "x", "created_at": "now"}


def observation(board, floor, x, y, yaw, height, noise=0., missing=(6,), seed=0):
    ids = [i for i in range(board.columns*board.rows) if i not in missing]
    objects = np.concatenate([board.points(i) for i in ids])
    local = board_plane_points(board, objects)
    pixels = project([yaw, x, y], local, floor, height)
    pixels += np.random.default_rng(seed).normal(0, noise, pixels.shape)
    return {"image_points": pixels.astype(np.float32), "object_points": objects, "timestamp": 0.,
            "quality": {"ready": True, "tags": len(ids), "reason": ""}}


class RobotTrackingTests(unittest.TestCase):
    board = Board(columns=3, rows=3, tag_size_mm=50, gap_mm=15)

    def test_pose_and_height_recovered(self):
        floor = synthetic_floor()
        for x, y, yaw in [(0, 1, 0), (.8, 2.5, 1.2), (-1, 1.7, -2.8)]:
            obs = observation(self.board, floor, x, y, yaw, .55, noise=.2)
            pose = board_pose(obs, floor, self.board, .55)
            self.assertAlmostEqual(pose["x_m"], x, delta=.003)
            self.assertAlmostEqual(pose["y_m"], y, delta=.003)
            self.assertLess(abs(math.remainder(pose["yaw_rad"]-yaw, 2*math.pi)), math.radians(.5))
            self.assertEqual(pose["tags"], 8)
            self.assertAlmostEqual(free_pose(obs, floor, self.board)["height_m"], .55, delta=.03)

    def test_misread_tag_is_dropped(self):
        floor = synthetic_floor()
        obs = observation(self.board, floor, .3, 1.5, .4, .5)
        obs["image_points"][4:8] += 25
        pose = board_pose(obs, floor, self.board, .5)
        self.assertEqual(pose["tags"], 7)
        self.assertAlmostEqual(pose["x_m"], .3, delta=.002)

    def test_board_off_its_height_falls_back_to_free_pose(self):
        floor = synthetic_floor()
        with tempfile.TemporaryDirectory() as directory:
            session = RobotSession(Path(directory))
            session.activate(self.board, floor)
            session.height_m = .55
            session.recording = "spin"
            session.update(observation(self.board, floor, .2, 1.4, .3, .25), floor)
            robot = session.live["robot"]
            self.assertEqual(robot["source"], "free_6dof")
            self.assertAlmostEqual(robot["z_m"], .25, delta=.03)
            self.assertAlmostEqual(robot["x_m"], .2, delta=.03)
            self.assertAlmostEqual(robot["y_m"], 1.4, delta=.03)
            self.assertEqual(session.recorded, [])
            session.update(observation(self.board, floor, .2, 1.4, .3, .55), floor)
            self.assertEqual(session.live["robot"]["source"], "board_plane")
            self.assertEqual(len(session.recorded), 1)

    def test_spin_and_drive_calibration(self):
        offset, heading = np.array([.12, -.05]), math.radians(30)
        spin_centre = np.array([.4, 1.6])
        spin = []
        for yaw in np.linspace(0, 2*math.pi, 40):
            board = spin_centre - rotation(yaw) @ offset
            spin.append({"x_m": board[0], "y_m": board[1], "yaw_rad": math.remainder(yaw, 2*math.pi)})
        result = fit_spin(spin)
        np.testing.assert_allclose(result["offset_m"], offset, atol=1e-9)
        drive = []
        board_yaw = .7
        for s in np.linspace(0, 1, 20):
            centre = np.array([-.5, 1.]) + s*np.array([math.cos(board_yaw+heading), math.sin(board_yaw+heading)])
            board = centre - rotation(board_yaw) @ offset
            drive.append({"x_m": board[0], "y_m": board[1], "yaw_rad": board_yaw})
        self.assertAlmostEqual(fit_drive(drive, offset)["heading_offset_rad"], heading, places=6)
        with self.assertRaises(ValueError):
            fit_spin(spin[:10])

    def test_session_persists_and_reports_robot_pose(self):
        floor = synthetic_floor()
        with tempfile.TemporaryDirectory() as directory:
            session = RobotSession(Path(directory))
            session.activate(self.board, floor)
            session.height_m, session.offset_m, session.heading_offset_rad = .5, [.1, 0.], math.pi/2
            session.save()
            session = RobotSession(Path(directory))
            obs = observation(self.board, floor, 0., 1.5, 0., .5)
            session.update(obs, floor)
            robot = session.live["robot"]
            self.assertAlmostEqual(robot["x_m"], .1, delta=.003)
            self.assertAlmostEqual(robot["yaw_rad"], math.pi/2, delta=.01)
            self.assertTrue(json.loads((Path(directory)/"robot"/"current.json").read_text())["height_m"])
        self.assertGreater(len(floor_footprint(floor)), 10)


if __name__ == "__main__":
    unittest.main()
