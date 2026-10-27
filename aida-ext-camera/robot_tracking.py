"""Robot pose on the calibrated floor from a flat AprilGrid mounted on the robot.

The board lies parallel to the floor at a known height, so its pose has only
three unknowns: x, y and yaw. Every corner ray is intersected with the plane
z = height, a 2-D rigid fit gives the starting pose and a reprojection fit
refines it. Lengths are metres, angles radians, floor frame of step 2.

Board frame: origin at the centre of the printed grid, X along printed rows,
Y toward the printed top, Z up (board face up).
"""
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import time

import cv2
import numpy as np
from scipy.optimize import least_squares

from calibration import Board


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def board_plane_points(board, object_points_mm):
    width, height = board.size
    points = np.asarray(object_points_mm, float)[:, :2]/1000
    return np.c_[points[:, 0]-width/2000, -(points[:, 1]-height/2000)]


def camera_model(floor_result):
    intrinsics = floor_result["intrinsics"]
    floor_from_camera = np.array(floor_result["T_floor_from_camera"])
    return (np.array(intrinsics["camera_matrix"]), np.array(intrinsics["distortion_coefficients"]),
            floor_from_camera, np.linalg.inv(floor_from_camera))


def rays_to_plane(pixels, floor_result, height_m):
    k, d, floor_from_camera, _ = camera_model(floor_result)
    rays = cv2.undistortPoints(np.asarray(pixels, np.float64).reshape(-1, 1, 2), k, d)[:, 0]
    directions = np.c_[rays, np.ones(len(rays))] @ floor_from_camera[:3, :3].T
    origin = floor_from_camera[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (height_m-origin[2])/directions[:, 2]
    valid = (np.abs(directions[:, 2]) > .015) & (distance > 0) & (distance < 30)
    return origin[:2]+distance[:, None]*directions[:, :2], valid


def rigid_fit(local, world):
    local_centre, world_centre = local.mean(axis=0), world.mean(axis=0)
    u, _, vt = np.linalg.svd((local-local_centre).T @ (world-world_centre))
    matrix = vt.T @ u.T
    if np.linalg.det(matrix) < 0:
        vt[-1] *= -1
        matrix = vt.T @ u.T
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return yaw, world_centre-rotation(yaw) @ local_centre


def project(parameters, local, floor_result, height_m):
    k, d, _, camera_from_floor = camera_model(floor_result)
    yaw, x, y = parameters
    world = np.c_[local @ rotation(yaw).T + [x, y], np.full(len(local), height_m)]
    camera = world @ camera_from_floor[:3, :3].T + camera_from_floor[:3, 3]
    return cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)[0][:, 0]


def board_pose(observation, floor_result, board, height_m):
    """Board centre (x, y) and yaw on the plane z = height_m, or None."""
    pixels = np.asarray(observation["image_points"], np.float64)
    if len(pixels) < 8:
        return None
    local = board_plane_points(board, observation["object_points"])
    world, valid = rays_to_plane(pixels, floor_result, height_m)
    keep = valid.reshape(-1, 4).all(axis=1)
    if np.count_nonzero(keep) < 2:
        return None
    for _ in range(2):
        mask = np.repeat(keep, 4)
        yaw, offset = rigid_fit(local[mask], world[mask])
        error = np.linalg.norm(local @ rotation(yaw).T + offset - world, axis=1).reshape(-1, 4).max(axis=1)
        # Drop whole tags that disagree with the rest (a misread or occluded tag).
        limit = max(.01, 4*float(np.median(error[keep])))
        refined = keep & (error <= limit)
        if np.count_nonzero(refined) < 2 or np.array_equal(refined, keep):
            break
        keep = refined
    mask = np.repeat(keep, 4)
    local, pixels = local[mask], pixels[mask]
    fit = least_squares(lambda p: (project(p, local, floor_result, height_m)-pixels).ravel(),
                        [yaw, *offset], method="lm")
    residual = fit.fun.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    sigma2 = max(float(np.sum(fit.fun**2))/max(1, fit.fun.size-3), .05)
    try:
        covariance = np.linalg.inv(fit.jac.T @ fit.jac)*sigma2
        std = np.sqrt(np.maximum(np.diag(covariance), 0))
    except np.linalg.LinAlgError:
        std = np.full(3, np.nan)
    yaw, x, y = fit.x
    return {"x_m": float(x), "y_m": float(y), "yaw_rad": float(wrap(yaw)), "rms_px": rms,
            "tags": int(np.count_nonzero(keep)), "std_xy_m": float(np.hypot(std[1], std[2])),
            "std_yaw_deg": float(math.degrees(std[0]))}


def free_pose(observation, floor_result, board):
    """Unconstrained 6-DoF board pose: centre, yaw, height and tilt."""
    objects = np.asarray(observation["object_points"], np.float64)/1000
    pixels = np.asarray(observation["image_points"], np.float64)
    if len(objects) < 16:
        return None
    k, d, floor_from_camera, _ = camera_model(floor_result)
    ok, rotations, translations, errors = cv2.solvePnPGeneric(objects, pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    best = None
    for rvec, tvec in zip(rotations, translations):
        matrix = np.eye(4)
        matrix[:3, :3] = cv2.Rodrigues(rvec)[0]
        matrix[:3, 3] = tvec.ravel()
        floor_from_board = floor_from_camera @ matrix
        projected = cv2.projectPoints(objects, rvec, tvec, k, d)[0][:, 0]
        rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))
        if best is None or rms < best[0]:
            best = (rms, floor_from_board)
    rms, floor_from_board = best
    width, height = board.size
    centre = floor_from_board @ [width/2000, height/2000, 0, 1]
    return {"height_m": float(centre[2]), "tilt_deg": math.degrees(math.acos(min(1., abs(floor_from_board[2, 2])))),
            "x_m": float(centre[0]), "y_m": float(centre[1]), "rms_px": rms,
            "yaw_rad": math.atan2(floor_from_board[1, 0], floor_from_board[0, 0])}


def fit_spin(poses):
    """Robot rotation centre in the board frame from an in-place rotation."""
    if len(poses) < 12:
        raise ValueError("Мало кадров вращения: крутите робота медленнее и дольше.")
    yaws = np.unwrap([p["yaw_rad"] for p in poses])
    span = math.degrees(yaws.max()-yaws.min())
    if span < 90:
        raise ValueError(f"Робот повернулся только на {span:.0f}°; нужно хотя бы 90°, лучше полный оборот.")
    rows, values = [], []
    for pose in poses:
        rows.append(np.c_[rotation(pose["yaw_rad"]), -np.eye(2)])
        values.append(-np.array([pose["x_m"], pose["y_m"]]))
    solution, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(values), rcond=None)
    centre_board, centre_world = solution[:2], solution[2:]
    spread = [np.linalg.norm(rotation(p["yaw_rad"]) @ centre_board + [p["x_m"], p["y_m"]] - centre_world) for p in poses]
    return {"offset_m": centre_board.tolist(), "centre_floor_m": centre_world.tolist(),
            "residual_m": float(np.sqrt(np.mean(np.square(spread)))), "span_deg": span, "frames": len(poses)}


def fit_drive(poses, offset_m):
    """Heading of the robot's forward direction relative to the board X axis."""
    if len(poses) < 6:
        raise ValueError("Мало кадров движения.")
    points = np.array([rotation(p["yaw_rad"]) @ offset_m + [p["x_m"], p["y_m"]] for p in poses])
    travel = points[-1]-points[0]
    if np.linalg.norm(travel) < .3:
        raise ValueError(f"Робот проехал {np.linalg.norm(travel):.2f} м; нужно хотя бы 0,3 м по прямой.")
    centred = points-points.mean(axis=0)
    direction = np.linalg.svd(centred)[2][0]
    if direction @ travel < 0:
        direction = -direction
    lateral = centred @ np.array([-direction[1], direction[0]])
    yaws = np.unwrap([p["yaw_rad"] for p in poses])
    heading = math.atan2(direction[1], direction[0])
    offset = wrap(heading-float(np.mean(yaws)))
    return {"heading_offset_rad": offset, "distance_m": float(np.linalg.norm(travel)),
            "lateral_rms_m": float(np.sqrt(np.mean(lateral**2))),
            "yaw_change_deg": float(math.degrees(yaws.max()-yaws.min())), "frames": len(poses)}


def floor_footprint(floor_result, samples=24):
    """Camera field of view on the floor, as a polygon (clipped at 12 m)."""
    width, height = floor_result["intrinsics"]["image_width"], floor_result["intrinsics"]["image_height"]
    edge = np.linspace(0, 1, samples)
    border = np.r_[np.c_[edge*width, np.zeros(samples)], np.c_[np.full(samples, width), edge*height],
                   np.c_[(1-edge)*width, np.full(samples, height)], np.c_[np.zeros(samples), (1-edge)*height]]
    points, valid = rays_to_plane(border, floor_result, 0.)
    camera = np.array(floor_result["camera_position_floor_m"][:2])
    polygon = []
    for point, ok in zip(points, valid):
        if ok and np.linalg.norm(point-camera) < 12:
            polygon.append([float(point[0]), float(point[1])])
    return polygon


class RobotSession:
    def __init__(self, data):
        self.root = data / "robot"
        self.board = None
        self.height_m = None
        self.offset_m = [0., 0.]
        self.heading_offset_rad = 0.
        self.robot_size_m = [.9, .6]
        self.spin = None
        self.drive = None
        self.floor_reference = None
        self.recording = None
        self.recorded = []
        self.live = None
        self.heights = deque(maxlen=40)
        self.trail = deque(maxlen=600)
        self.sequence = 0
        path = self.root / "current.json"
        if path.exists():
            saved = json.loads(path.read_text())
            self.board = Board.parse(saved["board"]) if saved.get("board") else None
            for key in ("height_m", "offset_m", "heading_offset_rad", "robot_size_m", "spin", "drive", "floor_reference"):
                if key in saved:
                    setattr(self, key, saved[key])

    def save(self):
        self.root.mkdir(parents=True, exist_ok=True)
        data = self.calibration()
        data["saved_at"] = datetime.now(timezone.utc).isoformat()
        temporary = self.root / "current.tmp"
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
        temporary.replace(self.root / "current.json")

    def calibration(self):
        return {"board": asdict(self.board) if self.board else None, "height_m": self.height_m,
                "offset_m": self.offset_m, "heading_offset_rad": self.heading_offset_rad,
                "robot_size_m": self.robot_size_m, "spin": self.spin, "drive": self.drive,
                "floor_reference": self.floor_reference,
                "frames": "board: centre of printed grid, X along printed rows, Y to printed top; "
                          "robot: offset_m in board frame, yaw = board yaw + heading_offset_rad"}

    def activate(self, floor_board, floor_result):
        # Height and offsets are properties of the robot, so a new floor
        # calibration keeps them; only remember which floor they were used with.
        self.floor_reference = [floor_result.get("reference_id"), floor_result.get("created_at")]
        if self.board is None:
            self.board = floor_board

    def robot_pose(self, pose):
        offset = rotation(pose["yaw_rad"]) @ np.array(self.offset_m)
        return {"x_m": pose["x_m"]+float(offset[0]), "y_m": pose["y_m"]+float(offset[1]),
                "yaw_rad": float(wrap(pose["yaw_rad"]+self.heading_offset_rad))}

    def update(self, observation, floor_result):
        now = time.time()
        live = {"timestamp": observation["timestamp"], "time": now, "tags": observation["quality"]["tags"],
                "board": None, "robot": None, "reason": observation["quality"]["reason"]}
        if observation["quality"]["ready"]:
            free = free_pose(observation, floor_result, self.board)
            if free is not None:
                self.heights.append(free)
            pose = None
            if self.height_m is not None:
                pose = board_pose(observation, floor_result, self.board, self.height_m)
            # The flat-at-known-height model is the accurate one for a mounted
            # board; when the board is clearly elsewhere (held, tilted) fall back
            # to the unconstrained pose instead of drawing a wrong position.
            off_plane = free is not None and (pose is None or (
                pose["rms_px"] > 3 and free["rms_px"] < pose["rms_px"]/2
                and (abs(free["height_m"]-self.height_m) > .03 or free["tilt_deg"] > 4)))
            if off_plane:
                pose = {"x_m": free["x_m"], "y_m": free["y_m"], "yaw_rad": free["yaw_rad"], "rms_px": free["rms_px"],
                        "tags": observation["quality"]["tags"], "std_xy_m": 0., "std_yaw_deg": 0.}
            if pose is not None:
                live["board"] = pose
                live["robot"] = {**self.robot_pose(pose), "std_xy_m": pose["std_xy_m"],
                                 "std_yaw_deg": pose["std_yaw_deg"], "rms_px": pose["rms_px"],
                                 "z_m": free["height_m"] if free else self.height_m,
                                 "tilt_deg": free["tilt_deg"] if free else None,
                                 "source": "free_6dof" if off_plane else "board_plane"}
                self.sequence += 1
                self.trail.append({"t": now, **live["robot"]})
                # Only accurate, on-plane poses calibrate the robot.
                if self.recording and not off_plane:
                    self.recorded.append(pose)
                if self.height_m is None:
                    live["reason"] = "Высота доски не задана: показана приблизительная 3D-поза."
                elif off_plane:
                    live["reason"] = (f"Доска не на заданной высоте ({free['height_m']*1000:.0f} мм вместо "
                                      f"{self.height_m*1000:.0f}, наклон {free['tilt_deg']:.1f}°): "
                                      "показана приблизительная 3D-поза.")
                else:
                    live["reason"] = "Робот найден."
        self.live = live

    def height_estimate(self):
        if len(self.heights) < 5:
            return None
        heights = np.array([h["height_m"] for h in self.heights])
        tilts = np.array([h["tilt_deg"] for h in self.heights])
        return {"height_m": float(np.median(heights)), "spread_m": float(np.std(heights)),
                "tilt_deg": float(np.median(tilts)), "frames": len(heights)}

    def state(self, live_window=1.5):
        live = self.live if self.live and time.monotonic()-self.live["timestamp"] < live_window else None
        return {"calibration": self.calibration(), "recording": self.recording, "recorded": len(self.recorded),
                "height_estimate": self.height_estimate(), "live": live, "sequence": self.sequence}
