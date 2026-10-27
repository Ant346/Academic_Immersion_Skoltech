"""Persistent floor workflow; mutations are protected by the Studio lock."""
from collections import deque
from dataclasses import asdict
import hashlib
import json
import time
import uuid
import threading

import cv2
import numpy as np

from calibration import Board
from floor_geometry import aggregate, fit_floor, pose_candidates


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


class FloorSession:
    def __init__(self, data, lock=None):
        self.lock = lock or threading.RLock()
        self.root = data / "floor"
        self.session = uuid.uuid4().hex
        self.intrinsics = None
        self.board = None
        self.samples = []
        self.result = None
        self.thickness_mm = 0.
        self.buffer = deque(maxlen=5)
        self.live = None
        self.error = ""
        self.running = False
        path = self.root / "current.json"
        if path.exists():
            saved = json.loads(path.read_text())
            for key in ("session", "intrinsics", "samples", "result", "thickness_mm"):
                setattr(self, key, saved[key])
            self.board = Board.parse(saved["board"]) if saved["board"] else None

    @property
    def directory(self):
        return self.root / "sessions" / self.session

    def save(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        saved = {"session": self.session, "intrinsics": self.intrinsics,
                 "board": asdict(self.board) if self.board else None,
                 "samples": self.samples, "result": self.result, "thickness_mm": self.thickness_mm}
        text = json.dumps(saved, ensure_ascii=False, indent=2, allow_nan=False)
        temporary = self.root / "current.tmp"
        temporary.write_text(text)
        temporary.replace(self.root / "current.json")
        (self.directory / "session.json").write_text(text)

    def activate(self, intrinsics):
        if intrinsics is None:
            raise ValueError("Сначала завершите шаг 1: рассчитайте intrinsics.")
        if self.intrinsics and fingerprint(self.intrinsics) != fingerprint(intrinsics):
            raise ValueError("Intrinsics изменились. Начните новую привязку пола; старая сохранена на диске.")
        self.intrinsics = intrinsics
        # The floor may use its own (larger) board; default to the intrinsics one.
        if self.board is None:
            self.board = Board.parse(intrinsics["board"])
        self.buffer.clear()
        self.save()

    def update(self, observation):
        self.live = observation
        if observation["quality"]["ready"]:
            self.buffer.append(observation)

    def readiness(self):
        fresh = [o for o in self.buffer if time.monotonic()-o["timestamp"] < 8]
        try:
            if self.live is None or not self.live["quality"]["ready"]:
                raise ValueError("Покажите лежащую на полу доску: нужны минимум 4 маркера.")
            combined = aggregate(fresh)
            if self.live and time.monotonic()-self.live["timestamp"] > 3:
                raise ValueError("Нет свежего наблюдения доски.")
            candidates, _, _, _ = pose_candidates(combined, self.intrinsics)
            return {"ready": True, "reason": "Доска стабильна. Можно сохранить положение.",
                    "jitter_px": combined["jitter_px"], "burst_frames": combined["burst_frames"],
                    "pose_rms_px": candidates[0]["rms_px"]}
        except (ValueError, cv2.error) as error:
            return {"ready": False, "reason": str(error)}

    def capture(self):
        if self.running:
            raise ValueError("Подождите окончания расчёта пола.")
        if len(self.samples) >= 40:
            raise ValueError("Достаточно 40 положений; исключите лишние.")
        readiness = self.readiness()
        if not readiness["ready"]:
            raise ValueError(readiness["reason"])
        combined = aggregate([o for o in self.buffer if time.monotonic()-o["timestamp"] < 8])
        candidates, objects, pixels, keep = pose_candidates(combined, self.intrinsics)
        centre = np.mean(pixels, axis=0)
        span = np.ptp(pixels, axis=0)
        for sample in self.samples:
            previous = np.array(sample["image_points"])
            if np.linalg.norm(centre-previous.mean(axis=0)) < max(4., np.linalg.norm(span)*.12):
                raise ValueError("Положение уже есть: переместите доску в другую точку пола.")
        sample_id = uuid.uuid4().hex[:12]
        ids = np.array(combined["quality"]["ids"])[keep.reshape(-1, 4).all(axis=1)].tolist()
        sample = {"id": sample_id, "object_points": (objects*1000).tolist(),
                  "image_points": pixels.tolist(), "tags": len(ids), "ids": ids,
                  "jitter_px": combined["jitter_px"], "burst_frames": combined["burst_frames"],
                  "pose_rms_px": candidates[0]["rms_px"]}
        self.directory.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.directory / f"{sample_id}.png"), combined["frame"]):
            raise ValueError("Не удалось сохранить исходный кадр пола.")
        self.samples.append(sample)
        self.result = None
        self.save()

    def calculate(self):
        result = fit_floor(self.samples, self.intrinsics, self.board, self.thickness_mm)
        with self.lock:
            self.result = result
            self.save()
        (self.directory / "extrinsics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
        storage = cv2.FileStorage(str(self.directory / "extrinsics.yaml"), cv2.FILE_STORAGE_WRITE)
        for key in ("T_camera_from_floor", "T_floor_from_camera", "H_floor_to_undistorted_pixels", "H_undistorted_pixels_to_floor"):
            storage.write(key, np.array(result[key]))
        storage.write("units", "metres")
        storage.write("valid", int(result["valid"]))
        storage.write("board_thickness_mm", result["board_thickness_mm"])
        storage.write("plane_equation_camera", np.array(result["plane_equation_camera"]))
        storage.write("camera_pose_covariance_conditional", np.array(result["camera_pose_covariance_conditional"]))
        storage.write("camera_height_m", result["camera_height_m"])
        storage.write("rms_px", result["rms_px"])
        storage.write("camera_matrix", np.array(self.intrinsics["camera_matrix"]))
        storage.write("distortion_coefficients", np.array(self.intrinsics["distortion_coefficients"]))
        storage.release()

    def state(self):
        return {"session": self.session, "intrinsics_ready": self.intrinsics is not None,
                "intrinsics": {key: self.intrinsics[key] for key in ("image_width", "image_height", "rms_px")} if self.intrinsics else None,
                "board": asdict(self.board) if self.board else None, "thickness_mm": self.thickness_mm,
                "samples": [{key: sample[key] for key in ("id", "tags", "pose_rms_px", "jitter_px", "burst_frames")} for sample in self.samples],
                "result": self.result, "running": self.running, "error": self.error,
                "quality": self.live["quality"] if self.live and time.monotonic()-self.live["timestamp"] < 4 else {"tags": 0, "reason": "Покажите доску на полу."},
                "stability": self.readiness() if self.intrinsics else {"ready": False, "reason": "Нужны intrinsics."}}
