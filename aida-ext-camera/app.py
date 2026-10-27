"""Local USB camera calibration server. Camera access stays on the host."""
import json
import multiprocessing
import os
import re
import subprocess
import threading
import time
import uuid
from collections import deque
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
OPENCV_THREADS = max(1, min(8, int(os.environ.get("OPENCV_THREADS", "4"))))
cv2.setNumThreads(OPENCV_THREADS)
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from calibration import Board, DETECTOR_VERSION, calibrate, detect, init_detector_process, too_similar
from floor_session import FloorSession, fingerprint
from floor_geometry import pixel_to_floor
from robot_tracking import RobotSession, fit_drive, fit_spin, floor_footprint

DATA = Path(os.environ.get("DATA_DIR", "/data"))
DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/camera")
CAMERA_NAME = os.environ.get("CAMERA_NAME", "USB camera")
CAMERA_ID = os.environ.get("CAMERA_ID", DEVICE)


def v4l(*args):
    try:
        return subprocess.run(["v4l2-ctl", "-d", DEVICE, *args], capture_output=True,
                              text=True, timeout=5, check=True).stdout
    except (subprocess.SubprocessError, OSError):
        return ""


def parse_camera_modes(text):
    modes, codec = {}, None
    for line in text.splitlines():
        match = re.search(r"\[\d+\]: '(\w+)'", line)
        if match:
            codec = match[1]
        match = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if match and codec in ("MJPG", "YUYV"):
            key = (int(match[1]), int(match[2]))
            if key not in modes or codec == "MJPG":
                modes[key] = {"width": key[0], "height": key[1], "fourcc": codec}
    return sorted(modes.values(), key=lambda mode: mode["width"]*mode["height"], reverse=True)


def camera_info():
    text = v4l("--list-formats-ext")
    return {"name": CAMERA_NAME, "device": DEVICE, "available": Path(DEVICE).exists() and bool(text),
            "modes": parse_camera_modes(text),
            "controls": v4l("--list-ctrls")}


def optics():
    text = v4l("--list-ctrls")
    values = {}
    for line in text.splitlines():
        match = re.search(r"^\s*(focus_\w+|zoom_absolute|pan_absolute|tilt_absolute)\s+.*?value=(-?\d+)", line)
        if match:
            values[match[1]] = int(match[2])
    return values


class Camera:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.decode_lock = threading.Lock()
        self.frame = None
        # MJPEG frames are kept compressed: a full 4K decode costs ~40-55 ms, so
        # it runs only when analysis asks for the frame, never on the video path.
        self.raw = None
        self.timestamp = 0
        self.sequence = 0
        self.meta = {}
        self.error = ""
        self.started_at = 0

    def start(self, width, height):
        self.stop()
        mode = next((mode for mode in camera_info()["modes"]
                     if mode["width"] == width and mode["height"] == height), None)
        if mode is None:
            raise ValueError("Этот режим недоступен. Обновите страницу и проверьте подключение USB-камеры.")
        capture = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise ValueError("Не удалось открыть USB-камеру. Закройте другие приложения, использующие её.")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode["fourcc"]))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        compressed = mode["fourcc"] == "MJPG" and capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        ok, frame = capture.read()
        raw = None
        if ok and compressed:
            raw, frame = frame, decode_jpeg(frame)
            ok = frame is not None
        if not ok:
            capture.release()
            raise ValueError("Камера открылась, но не передала изображение.")
        actual_h, actual_w = frame.shape[:2]
        if (actual_w, actual_h) != (width, height):
            capture.release()
            raise ValueError(f"Камера выдала {actual_w} × {actual_h} вместо {width} × {height}. Переподключите её и обновите список режимов.")
        self.meta = {"name": CAMERA_NAME, "id": CAMERA_ID, "device": DEVICE, "width": actual_w, "height": actual_h,
                     "requested_width": width, "requested_height": height,
                     "fps": capture.get(cv2.CAP_PROP_FPS), "fourcc": mode["fourcc"],
                     "controls_at_start": v4l("--list-ctrls")}
        self.stop_event.clear()
        self.started_at = time.monotonic()
        with self.lock:
            self.frame, self.raw, self.timestamp = frame, raw, time.monotonic()
            self.sequence += 1
            self.error = ""
        self.thread = threading.Thread(target=self._read, args=(capture, compressed), daemon=True)
        self.thread.start()

    def _read(self, capture, compressed):
        try:
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    with self.lock:
                        self.error = "Нет изображения от USB-камеры. Проверьте кабель и подключите её снова."
                    break
                with self.lock:
                    if compressed:
                        self.frame, self.raw = None, frame
                    else:
                        self.frame, self.raw = frame, None
                    self.timestamp = time.monotonic()
                    self.sequence += 1
        finally:
            capture.release()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise ValueError("Камера не отвечает. Переподключите USB-камеру и перезапустите контейнер.")
        self.thread = None
        with self.lock:
            self.frame = None
            self.raw = None
            self.timestamp = 0

    def latest(self, copy=True):
        """Full-resolution BGR frame; decoded on demand and cached per sequence."""
        with self.lock:
            frame, raw, timestamp, sequence = self.frame, self.raw, self.timestamp, self.sequence
        if frame is None and raw is not None:
            with self.decode_lock:
                with self.lock:
                    if self.sequence == sequence and self.frame is not None:
                        frame = self.frame
                if frame is None:
                    frame = decode_jpeg(raw)
                    with self.lock:
                        if self.sequence == sequence:
                            self.frame = frame
        return (None if frame is None else frame.copy() if copy else frame, timestamp, sequence)

    def preview(self):
        """Half-resolution frame for display (JPEG DCT scaling), plus full (width, height)."""
        with self.lock:
            frame, raw, timestamp, sequence = self.frame, self.raw, self.timestamp, self.sequence
        if frame is not None:
            return frame, (frame.shape[1], frame.shape[0]), timestamp, sequence
        if raw is None:
            return None, None, timestamp, sequence
        small = decode_jpeg(raw, cv2.IMREAD_REDUCED_COLOR_2)
        if small is None:
            return None, None, timestamp, sequence
        return small, (self.meta.get("width", small.shape[1]*2), self.meta.get("height", small.shape[0]*2)), timestamp, sequence


def decode_jpeg(raw, flags=cv2.IMREAD_COLOR):
    try:
        return cv2.imdecode(raw, flags)
    except cv2.error:
        return None


class Studio:
    def __init__(self):
        DATA.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.camera = Camera()
        self.board = Board()
        self.confirmed = False
        self.session = uuid.uuid4().hex
        self.samples = []
        self.image_size = None
        self.session_camera = None
        self.result = None
        self.job = {"running": False, "error": None}
        self.latest = None
        self.jpeg = None
        self.preview_undistort = False
        self.closing = threading.Event()
        self.maps = None
        self.migration = None
        self.mode = "intrinsics"
        self.performance = {"preview_fps": 0., "analysis_fps": 0., "analysis_ms": 0.}
        self._restore()
        self.floor = FloorSession(DATA, self.lock)
        self.robot = RobotSession(DATA)
        self.detector_pool = None
        self.worker = threading.Thread(target=self._process, daemon=True)
        self.worker.start()
        self.preview_worker = threading.Thread(target=self._preview, daemon=True)
        self.preview_worker.start()

    @property
    def directory(self):
        return DATA / "sessions" / self.session

    def _restore(self):
        path = DATA / "current.json"
        if path.exists():
            saved = json.loads(path.read_text())
            self.board = Board.parse(saved["board"])
            self.confirmed = saved["confirmed"]
            self.session = saved["session"]
            self.samples = saved["samples"]
            self.image_size = saved["image_size"]
            self.session_camera = saved["camera"]
            self.result = saved["result"]
            self.migration = saved.get("migration")
            if saved.get("detector_version") != DETECTOR_VERSION:
                backup = self.directory / "session_before_detector_v2.json"
                backup.parent.mkdir(parents=True, exist_ok=True)
                if not backup.exists():
                    backup.write_text(path.read_text())
                report, corrected = [], []
                for sample in self.samples:
                    image = cv2.imread(str(self.directory / f"{sample['id']}.png"))
                    observation = detect(image, self.board) if image is not None else None
                    valid = (observation is not None and observation["quality"]["ready"]
                             and list(observation["image_size"]) == self.image_size)
                    report.append({"id": sample["id"], "old_tags": sample["tags"],
                                   "new_tags": observation["quality"]["tags"] if observation else 0,
                                   "accepted": bool(valid)})
                    if valid:
                        corrected.append({**sample, "tags": observation["quality"]["tags"],
                                          "sharpness": observation["quality"]["sharpness"],
                                          "image_points": observation["image_points"].tolist(),
                                          "object_points": observation["object_points"].tolist(),
                                          "polygon": observation["polygon"].tolist(),
                                          "detector_version": DETECTOR_VERSION})
                self.samples = corrected
                self.result = None
                self.migration = {"frames": report, "backup": backup.name} if report else None
                self.save()

    def save(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        data = {"session": self.session, "board": asdict(self.board), "confirmed": self.confirmed,
                "samples": self.samples, "image_size": self.image_size,
                "camera": self.session_camera, "result": self.result,
                "detector_version": DETECTOR_VERSION, "migration": self.migration}
        content = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
        path = DATA / "current.tmp"
        path.write_text(content)
        path.replace(DATA / "current.json")
        (self.directory / "session.json").write_text(content)

    def idle(self):
        if self.job["running"]:
            raise ValueError("Подождите окончания расчёта.")
        if hasattr(self, "floor") and self.floor.running:
            raise ValueError("Подождите окончания расчёта пола.")

    def mode_board(self):
        if self.mode == "floor" and self.floor.board:
            return self.floor.board
        if self.mode == "robot" and self.robot.board:
            return self.robot.board
        return self.board

    def _detect(self, *args, **kwargs):
        """Detection runs in its own process: its Python-heavy edge fitting would
        otherwise hold the GIL and stall the video thread."""
        if self.detector_pool is None:
            self.detector_pool = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context("spawn"),
                                                     initializer=init_detector_process, initargs=(OPENCV_THREADS,))
        try:
            return self.detector_pool.submit(detect, *args, **kwargs).result()
        except BrokenProcessPool:
            self.detector_pool = None
            raise

    def _process(self):
        try:
            self._analyse()
        finally:
            if self.detector_pool is not None:
                self.detector_pool.shutdown(wait=False, cancel_futures=True)

    def _analyse(self):
        previous = -1
        roi = None
        roi_mode = None
        last_analysis = time.monotonic()
        while not self.closing.wait(.04):
            frame, timestamp, sequence = self.camera.latest(copy=False)
            if frame is None or sequence == previous:
                continue
            previous = sequence
            with self.lock:
                board = self.mode_board()
                generation = self.session
                mode = self.mode
                floor_generation = self.floor.session
            try:
                started = time.monotonic()
                if roi_mode != (mode, board, floor_generation):
                    roi = None
                    roi_mode = (mode, board, floor_generation)
                distant = mode in ("floor", "robot")
                observation = self._detect(frame, board, distant=distant, search_roi=roi if distant else None)
                if distant and roi is not None and not observation["quality"]["ready"]:
                    observation = self._detect(frame, board, distant=True)
                roi = None
                if distant and observation["quality"]["ready"]:
                    points = observation["polygon"] if observation["polygon"] is not None else observation["image_points"]
                    if np.isfinite(points).all():
                        x, y, w, h = cv2.boundingRect(points.astype(np.float32))
                        padding = int(max(48, .6*max(w, h)))
                        x0, y0 = max(0, x-padding), max(0, y-padding)
                        x1, y1 = min(frame.shape[1], x+w+padding), min(frame.shape[0], y+h+padding)
                        if x1>x0 and y1>y0:
                            roi = (x0, y0, x1-x0, y1-y0)
                observation.update(timestamp=timestamp, sequence=sequence, frame=frame)
                with self.lock:
                    current_board = self.mode_board()
                    if (generation != self.session or board != current_board
                            or mode != self.mode or floor_generation != self.floor.session
                            or timestamp < self.camera.started_at or self.camera.stop_event.is_set()):
                        continue
                    self.latest = observation
                    if mode == "floor":
                        self.floor.update(observation)
                    elif mode == "robot" and self.floor.result:
                        self.robot.update(observation, self.floor.result)
                    self.camera.error = ""
                    now = time.monotonic()
                    self.performance["analysis_ms"] = round((now-started)*1000, 1)
                    self.performance["analysis_fps"] = round(1/max(.001, now-last_analysis), 1)
                    last_analysis = now
                self.closing.wait(max(0, .1-(time.monotonic()-started)))
            except Exception as error:
                with self.lock:
                    self.camera.error = f"Ошибка обработки кадра: {error}"

    def _preview(self):
        """Fresh video independent of analysis; optical flow is display-only."""
        previous_sequence = -1
        detection_sequence = -1
        previous_gray = None
        tracked = None
        ids = []
        counter, clock = 0, time.monotonic()
        map_key, maps = None, None
        grid_key, grid = None, []
        camera_clock, camera_sequence = time.monotonic(), None
        # A 4K MJPEG decode, even at half size, takes ~30 ms on one core; keep two
        # in flight (imdecode releases the GIL) so the video can follow 30 fps.
        decoder = ThreadPoolExecutor(2)
        pending = deque()
        submitted_sequence = -1
        while not self.closing.is_set():
            with self.camera.lock:
                current = self.camera.sequence
            if current != submitted_sequence and len(pending) < 2:
                pending.append(decoder.submit(self.camera.preview))
                submitted_sequence = current
            if not pending or not pending[0].done():
                self.closing.wait(.003)
                continue
            started = time.monotonic()
            try:
                frame, full_size, timestamp, sequence = pending.popleft().result()
            except Exception as error:
                with self.lock:
                    self.performance["preview_error"] = str(error)
                continue
            if frame is None or sequence <= previous_sequence:
                continue
            previous_sequence = sequence
            try:
                scale = min(1., 1280/full_size[0])
                size = (round(full_size[0]*scale), round(full_size[1]*scale))
                image = cv2.resize(frame, size) if frame.shape[1::-1] != size else frame.copy()
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                with self.lock:
                    observation = self.latest
                    floor_result = self.floor.result if self.mode == "floor" else None
                    intrinsics = self.result if self.preview_undistort else None
                if observation and observation["sequence"] != detection_sequence:
                    detection_sequence = observation["sequence"]
                    source = cv2.resize(observation["frame"], size)
                    previous_gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
                    tracked = (observation["image_points"]*scale).astype(np.float32).reshape(-1, 1, 2)
                    ids = list(observation["quality"]["ids"])
                if tracked is not None and len(tracked) and previous_gray is not None and previous_gray.shape == gray.shape:
                    moved, status, _ = cv2.calcOpticalFlowPyrLK(previous_gray, gray, tracked, None, winSize=(21, 21), maxLevel=3)
                    if moved is not None:
                        back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, previous_gray, moved, None, winSize=(21, 21), maxLevel=3)
                        valid = ((status[:, 0] > 0) & (reverse[:, 0] > 0)
                                 & (np.linalg.norm(back[:, 0]-tracked[:, 0], axis=1) < .8))
                        good = valid.reshape(-1, 4).all(axis=1)
                        ids = [tag for tag, ok in zip(ids, good) if ok]
                        tracked = moved.reshape(-1, 4, 2)[good].reshape(-1, 1, 2)
                    else:
                        tracked = None
                        ids = []
                previous_gray = gray
                if observation and time.monotonic()-observation["timestamp"] < 2.5 and tracked is not None:
                    for tag, points in zip(ids, tracked.reshape(-1, 4, 2)):
                        polygon = np.rint(points).astype(np.int32)
                        cv2.polylines(image, [polygon], True, (100, 240, 80), 1)
                        cv2.putText(image, str(tag), tuple(polygon[0]), cv2.FONT_HERSHEY_SIMPLEX, .35, (40, 240, 240), 1)
                if floor_result:
                    if grid_key is None or grid_key[0] is not floor_result or grid_key[1] != size:
                        grid_key = (floor_result, size)
                        grid = floor_grid_segments(floor_result, size, image_scale=scale)
                    for a, b, color in grid:
                        cv2.line(image, a, b, color, 1, cv2.LINE_AA)
                if intrinsics:
                    key = (id(intrinsics), size)
                    if key != map_key:
                        k = np.array(intrinsics["camera_matrix"]); k[:2] *= scale
                        maps = cv2.initUndistortRectifyMap(k, np.array(intrinsics["distortion_coefficients"]), None, k, size, cv2.CV_32FC1)
                        map_key = key
                    image = cv2.remap(image, *maps, cv2.INTER_LINEAR)
                ok, jpg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self.lock:
                        self.jpeg = jpg.tobytes()
                        counter += 1
                        self.performance["preview_ms"] = round((time.monotonic()-started)*1000, 1)
                        if time.monotonic()-clock >= 1:
                            self.performance["preview_fps"] = round(counter/(time.monotonic()-clock), 1)
                            counter, clock = 0, time.monotonic()
                        if camera_sequence is None or time.monotonic()-camera_clock >= 1:
                            if camera_sequence is not None:
                                self.performance["camera_fps"] = round((sequence-camera_sequence)/(time.monotonic()-camera_clock), 1)
                            camera_clock, camera_sequence = time.monotonic(), sequence
            except Exception as error:
                tracked = None
                previous_gray = None
                with self.lock:
                    self.performance["preview_error"] = str(error)
                # Keep raw video flowing even when a visualization fails.
                fallback = cv2.resize(frame, (1280, round(frame.shape[0]*1280/frame.shape[1])))
                ok, jpg = cv2.imencode(".jpg", fallback, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self.lock:
                        self.jpeg = jpg.tobytes()
        decoder.shutdown(wait=False, cancel_futures=True)

    def state(self):
        with self.lock:
            latest = self.latest
            fresh = latest is not None and time.monotonic()-latest["timestamp"] < 2
            quality = dict(latest["quality"]) if latest and fresh else {
                "tags": 0, "ready": False, "reason": "Подключите камеру и покажите доску.",
                "sharpness": 0, "area_percent": 0}
            if fresh and quality["ready"] and too_similar(latest, self.samples):
                quality.update(ready=False, reason="Такое положение уже сохранено. Сдвиньте или наклоните доску.")
            errors = {e["id"]: e["rms_px"] for e in self.result["per_view_errors"]} if self.result else {}
            coverage = np.zeros((4, 6), int)
            if self.image_size:
                w, h = self.image_size
                for sample in self.samples:
                    points = np.array(sample["image_points"])
                    cells = np.unique(np.column_stack((np.clip((points[:, 1]*4/h).astype(int), 0, 3),
                                                       np.clip((points[:, 0]*6/w).astype(int), 0, 5))), axis=0)
                    for row, col in cells:
                        coverage[row, col] += 1
            return {"board": asdict(self.board), "confirmed": self.confirmed, "session": self.session,
                    "performance": dict(self.performance), "mode": self.mode, "connected": self.camera.thread is not None and self.camera.thread.is_alive(),
                    "camera": self.camera.meta or self.session_camera or {}, "camera_error": self.camera.error,
                    "quality": quality, "result": self.result, "job": self.job,
                    "preview_undistort": self.preview_undistort, "coverage": coverage.tolist(),
                    "detector_version": DETECTOR_VERSION, "migration": self.migration,
                    "samples": [{"id": s["id"], "tags": s["tags"], "sharpness": s["sharpness"],
                                 "rms_px": errors.get(s["id"]), "thumbnail": f"/api/samples/{s['id']}/thumbnail"}
                                for s in self.samples]}

    def capture(self):
        with self.lock:
            self.idle()
            if self.mode != "intrinsics":
                raise ValueError("Для intrinsics откройте страницу первого шага.")
            if not self.confirmed:
                raise ValueError("Измерьте размеры доски и подтвердите их в настройках.")
            if self.camera.thread is None or not self.camera.thread.is_alive():
                raise ValueError("USB-камера отключена. Подключите её снова.")
            controls = optics()
            if self.session_camera and controls != self.session_camera.get("optical_controls", controls):
                raise ValueError("Изменились фокус, зум или кадрирование. Верните настройки или начните новую сессию.")
            observation = self.latest
            if observation is None or time.monotonic()-observation["timestamp"] > 2:
                raise ValueError("Нет свежего кадра. Подключите камеру.")
            if not observation["quality"]["ready"]:
                raise ValueError(observation["quality"]["reason"])
            if too_similar(observation, self.samples):
                raise ValueError("Положение уже сохранено: измените положение или наклон доски.")
            size = list(observation["image_size"])
            if self.image_size is not None and size != self.image_size:
                raise ValueError("Разрешение изменилось. Начните новую сессию.")
            if len(self.samples) >= 100:
                raise ValueError("В сессии уже 100 кадров. Удалите лишние наблюдения.")
            self.directory.mkdir(parents=True, exist_ok=True)
            sample_id = uuid.uuid4().hex[:12]
            if not cv2.imwrite(str(self.directory / f"{sample_id}.png"), observation["frame"]):
                raise OSError("Не удалось сохранить исходный кадр.")
            frame = observation["frame"]
            thumbnail = cv2.resize(frame, (240, round(frame.shape[0]*240/frame.shape[1])))
            if not cv2.imwrite(str(self.directory / f"{sample_id}.jpg"), thumbnail):
                raise OSError("Не удалось сохранить миниатюру.")
            self.samples.append({"id": sample_id, "tags": observation["quality"]["tags"],
                                 "detector_version": DETECTOR_VERSION,
                                 "sharpness": observation["quality"]["sharpness"],
                                 "object_points": observation["object_points"].tolist(),
                                 "image_points": observation["image_points"].tolist(),
                                 "polygon": observation["polygon"].tolist()})
            self.image_size = size
            self.session_camera = dict(self.camera.meta)
            self.session_camera["optical_controls"] = controls
            self.result = None
            self.maps = None
            self.preview_undistort = False
            self.save()

    def calculate(self):
        with self.lock:
            self.idle()
            if len(self.samples) < 12:
                raise ValueError("Нужно минимум 12 разных кадров. Рекомендуется 25–35.")
            samples, size, board, camera = list(self.samples), self.image_size, self.board, self.session_camera
            self.job = {"running": True, "error": None}

        def work():
            try:
                result = calibrate(samples, size, board, camera)
                with self.lock:
                    self.result = result
                    self.maps = None
                    (self.directory / "intrinsics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
                    storage = cv2.FileStorage(str(self.directory / "intrinsics.yaml"), cv2.FILE_STORAGE_WRITE)
                    storage.write("image_width", result["image_width"])
                    storage.write("image_height", result["image_height"])
                    storage.write("camera_matrix", np.array(result["camera_matrix"]))
                    storage.write("distortion_coefficients", np.array(result["distortion_coefficients"]).reshape(1, -1))
                    storage.write("distortion_model", result["distortion_model"])
                    storage.write("rms_px", result["rms_px"])
                    storage.release()
                    self.save()
                    self.job = {"running": False, "error": None}
            except Exception as error:
                with self.lock:
                    self.job = {"running": False, "error": str(error)}
        threading.Thread(target=work, daemon=True).start()


studio = None


@asynccontextmanager
async def lifespan(app):
    global studio
    studio = Studio()
    yield
    studio.closing.set()
    studio.camera.stop()


app = FastAPI(lifespan=lifespan)


@app.exception_handler(ValueError)
async def value_error(request, error):
    from fastapi.responses import JSONResponse
    return JSONResponse({"detail": str(error)}, status_code=400)


class CameraRequest(BaseModel):
    width: int = Field(default=1920, ge=320, le=4096)
    height: int = Field(default=1080, ge=240, le=2160)


class BoardRequest(BaseModel):
    columns: int = Field(default=11, ge=2, le=30)
    rows: int = Field(default=8, ge=2, le=30)
    tag_size_mm: float = Field(default=25, ge=1, le=500, allow_inf_nan=False)
    gap_mm: float = Field(default=7.5, gt=0, le=500, allow_inf_nan=False)
    first_id: int = Field(default=0, ge=0, le=586)
    rotation: int = 180
    ids_bottom_up: bool = True
    border_bits: int = Field(default=2, ge=1, le=3)
    confirmed: bool = False


@app.get("/api/camera")
def get_camera():
    return camera_info()


@app.get("/api/state")
def get_state():
    return studio.state()


@app.post("/api/camera/connect")
def connect(body: CameraRequest):
    with studio.lock:
        studio.idle()
        if studio.image_size and [body.width, body.height] != studio.image_size:
            raise ValueError("Для другого разрешения начните новую сессию.")
        if studio.session_camera and studio.session_camera.get("id", CAMERA_ID) != CAMERA_ID:
            raise ValueError("Эта сессия относится к другой USB-камере. Начните новую сессию.")
        studio.latest = None
        studio.jpeg = None
        studio.camera.start(body.width, body.height)
        if studio.image_size and [studio.camera.meta["width"], studio.camera.meta["height"]] != studio.image_size:
            studio.camera.stop()
            raise ValueError("Камера выдала другое разрешение. Начните новую сессию.")
    return studio.state()


@app.post("/api/camera/disconnect")
def disconnect():
    with studio.lock:
        studio.idle()
        studio.camera.stop()
        studio.latest = None
        studio.jpeg = None
    return studio.state()


@app.post("/api/board")
def set_board(body: BoardRequest):
    values = body.model_dump()
    confirmed = values.pop("confirmed")
    board = Board.parse(values)
    with studio.lock:
        studio.idle()
        if studio.samples and board != studio.board:
            raise ValueError("Для изменения геометрии доски начните новую сессию.")
        studio.board, studio.confirmed = board, confirmed
        studio.latest = None
        studio.save()
    return studio.state()


@app.post("/api/samples")
def capture():
    studio.capture()
    return studio.state()


@app.delete("/api/samples/{sample_id}")
def delete_sample(sample_id: str):
    with studio.lock:
        studio.idle()
        if not any(s["id"] == sample_id for s in studio.samples):
            raise HTTPException(404, "Кадр не найден.")
        studio.samples = [s for s in studio.samples if s["id"] != sample_id]
        studio.result = None
        studio.maps = None
        studio.preview_undistort = False
        studio.save()
    return studio.state()


@app.get("/api/samples/{sample_id}/thumbnail")
def thumbnail(sample_id: str):
    with studio.lock:
        if not any(s["id"] == sample_id for s in studio.samples):
            raise HTTPException(404)
        path = studio.directory / f"{sample_id}.jpg"
    return FileResponse(path, media_type="image/jpeg")


@app.post("/api/session/new")
def new_session():
    with studio.lock:
        studio.idle()
        # Old sessions and raw frames remain on disk.
        studio.session = uuid.uuid4().hex
        studio.samples = []
        studio.result = None
        studio.image_size = None
        studio.session_camera = None
        studio.maps = None
        studio.preview_undistort = False
        studio.latest = None
        studio.job = {"running": False, "error": None}
        studio.migration = None
        studio.save()
    return studio.state()


@app.post("/api/calibrate")
def run_calibration():
    studio.calculate()
    return studio.state()


class PreviewRequest(BaseModel):
    undistort: bool


@app.post("/api/preview")
def preview(body: PreviewRequest):
    with studio.lock:
        if body.undistort and studio.result is None:
            raise ValueError("Сначала выполните калибровку.")
        studio.preview_undistort = body.undistort
    return studio.state()


@app.get("/api/export/{kind}")
def export(kind: str):
    if kind not in ("json", "yaml"):
        raise HTTPException(404)
    with studio.lock:
        if studio.result is None:
            raise HTTPException(404, "Сначала выполните калибровку.")
        path = studio.directory / f"intrinsics.{kind}"
    return FileResponse(path, filename=f"intrinsics.{kind}")


@app.get("/api/stream")
def stream():
    def frames():
        previous = None
        while not studio.closing.is_set():
            with studio.lock:
                jpeg = studio.jpeg
            if jpeg is not None and jpeg is not previous:
                previous = jpeg
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            time.sleep(.02)
    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame",
                             headers={"Cache-Control": "no-store"})


def floor_grid_segments(result, size, image_scale=1.):
    """0.5 m floor grid as clipped image segments; cache it, the projection is slow."""
    segments = []
    if not result["valid"]:
        return segments
    k = np.array(result["intrinsics"]["camera_matrix"])
    k[:2] *= image_scale
    d = np.array(result["intrinsics"]["distortion_coefficients"])
    matrix = np.array(result["T_camera_from_floor"])
    rotation = cv2.Rodrigues(matrix[:3, :3])[0]
    positions = np.array([[p["x_m"], p["y_m"]] for p in result["placements"]])
    low = np.maximum(np.floor(positions.min(axis=0)-1), -10)
    high = np.minimum(np.ceil(positions.max(axis=0)+1), 10)
    w, h = size
    for axis in (0, 1):
        for value in np.arange(low[axis], high[axis]+.1, .5):
            points = np.zeros((40, 3))
            points[:, axis] = value
            points[:, 1-axis] = np.linspace(low[1-axis], high[1-axis], 40)
            camera = points @ matrix[:3, :3].T + matrix[:3, 3]
            pixels = cv2.projectPoints(points, rotation, matrix[:3, 3], k, d)[0][:, 0]
            color = (70, 200, 250) if abs(value) > .01 else ((70, 220, 70) if axis == 0 else (60, 80, 255))
            for i in range(39):
                if np.any(camera[i:i+2, 2] < .1) or np.any(np.abs(pixels[i:i+2]) > 100000):
                    continue
                ok, a, b = cv2.clipLine((0, 0, w, h), tuple(np.rint(pixels[i]).astype(int)), tuple(np.rint(pixels[i+1]).astype(int)))
                if ok:
                    segments.append((a, b, color))
    return segments


class ModeRequest(BaseModel):
    mode: str


@app.post("/api/mode")
def change_mode(body: ModeRequest):
    if body.mode not in ("intrinsics", "floor", "robot"):
        raise ValueError("Неизвестный режим.")
    with studio.lock:
        studio.idle()
        if body.mode == "floor":
            studio.floor.activate(studio.result)
        if body.mode == "robot":
            if not studio.floor.result or not studio.floor.result["valid"]:
                raise ValueError("Сначала получите привязку камеры к полу (шаг 2).")
            studio.robot.activate(studio.floor.board, studio.floor.result)
        studio.mode = body.mode
        studio.latest = None
        studio.floor.buffer.clear()
        studio.preview_undistort = False
    return studio.state()


def floor_state():
    with studio.lock:
        return {**studio.floor.state(), "mode": studio.mode,
                "performance": dict(studio.performance),
                "connected": studio.camera.thread is not None and studio.camera.thread.is_alive(),
                "camera": studio.camera.meta or studio.session_camera or {}, "camera_error": studio.camera.error}


@app.get("/api/floor/state")
def get_floor_state():
    return floor_state()


@app.get("/api/floor/detail.png")
def floor_detail():
    with studio.lock:
        observation = studio.floor.live
        if observation is None or time.monotonic()-observation["timestamp"] > 4 or not len(observation["image_points"]):
            raise HTTPException(404)
        frame = observation["frame"]
        x, y, w, h = cv2.boundingRect(observation["image_points"])
        crop = frame[max(0, y-12):min(frame.shape[0], y+h+12), max(0, x-12):min(frame.shape[1], x+w+12)].copy()
    factor = min(4., 700/max(crop.shape[:2]))
    crop = cv2.resize(crop, None, fx=factor, fy=factor, interpolation=cv2.INTER_NEAREST)
    ok, data = cv2.imencode(".png", crop)
    if not ok:
        raise HTTPException(500)
    return Response(data.tobytes(), media_type="image/png", headers={"Cache-Control": "no-store"})


@app.get("/api/floor/frame.png")
def floor_raw_frame():
    frame, timestamp, _ = studio.camera.latest()
    if frame is None or time.monotonic()-timestamp > 3:
        raise HTTPException(404, "Нет свежего изображения камеры.")
    ok, data = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise HTTPException(500)
    return Response(data.tobytes(), media_type="image/png", headers={"Cache-Control": "no-store"})


def floor_guard():
    studio.idle()
    if studio.mode != "floor" or studio.floor.intrinsics is None:
        raise ValueError("Сначала включите второй шаг с сохранёнными intrinsics.")
    if studio.result is None or fingerprint(studio.result) != fingerprint(studio.floor.intrinsics):
        raise ValueError("Intrinsics изменились. Начните новую привязку пола.")


class FloorSettings(BaseModel):
    thickness_mm: float = Field(default=0, ge=0, le=100, allow_inf_nan=False)
    tag_size_mm: float | None = Field(default=None, ge=1, le=500, allow_inf_nan=False)
    gap_mm: float | None = Field(default=None, gt=0, le=500, allow_inf_nan=False)
    columns: int | None = Field(default=None, ge=2, le=30)
    rows: int | None = Field(default=None, ge=2, le=30)


@app.post("/api/floor/settings")
def floor_settings(body: FloorSettings):
    with studio.lock:
        floor_guard()
        if studio.floor.samples and studio.floor.thickness_mm != body.thickness_mm:
            raise ValueError("Для другой толщины доски начните новую привязку пола.")
        board = studio.floor.board
        changes = {key: value for key, value in body.model_dump().items()
                   if key in ("tag_size_mm", "gap_mm", "columns", "rows") and value is not None}
        if board is not None and changes:
            board = Board.parse({**asdict(board), **changes})
        if studio.floor.samples and board != studio.floor.board:
            raise ValueError("Для другой доски начните новую привязку пола.")
        if board != studio.floor.board:
            studio.floor.board = board
            studio.floor.buffer.clear()
            studio.floor.live = None
            studio.latest = None
        studio.floor.thickness_mm = body.thickness_mm
        studio.floor.save()
    return floor_state()


@app.post("/api/floor/samples")
def floor_capture():
    with studio.lock:
        floor_guard()
        if studio.camera.thread is None or not studio.camera.thread.is_alive():
            raise ValueError("Подключите USB-камеру.")
        source = studio.floor.intrinsics
        if (studio.camera.meta["width"], studio.camera.meta["height"]) != (source["image_width"], source["image_height"]):
            raise ValueError("Разрешение не совпадает с intrinsics.")
        expected = source.get("camera", {}).get("optical_controls")
        if expected is not None and optics() != expected:
            raise ValueError("Настройки зума/фокуса отличаются от intrinsics.")
        studio.floor.capture()
    return floor_state()


@app.delete("/api/floor/samples/{sample_id}")
def floor_delete(sample_id: str):
    with studio.lock:
        floor_guard()
        if studio.floor.samples and studio.floor.samples[0]["id"] == sample_id:
            raise ValueError("Опорное положение задаёт начало координат. Для его замены начните новую привязку.")
        if not any(s["id"] == sample_id for s in studio.floor.samples):
            raise HTTPException(404)
        studio.floor.samples = [s for s in studio.floor.samples if s["id"] != sample_id]
        studio.floor.result = None
        studio.floor.save()
    return floor_state()


@app.post("/api/floor/new")
def floor_new():
    with studio.lock:
        studio.idle()
        if studio.result is None:
            raise ValueError("Сначала рассчитайте intrinsics.")
        studio.floor = FloorSession(DATA, studio.lock)
        studio.floor.session = uuid.uuid4().hex
        studio.floor.samples = []
        studio.floor.result = None
        studio.floor.intrinsics = None
        studio.floor.live = None
        studio.floor.activate(studio.result)
        studio.mode = "floor"
        studio.latest = None
        studio.preview_undistort = False
    return floor_state()


@app.post("/api/floor/calibrate")
def floor_calibrate():
    with studio.lock:
        floor_guard()
        if len(studio.floor.samples) < 6:
            raise ValueError("Нужно минимум 6 разных положений доски на полу.")
        target = studio.floor
        target.running = True
        target.error = ""
    def work():
        try:
            target.calculate()
        except Exception as error:
            with studio.lock:
                target.error = str(error)
        finally:
            with studio.lock:
                target.running = False
    threading.Thread(target=work, daemon=True).start()
    return floor_state()


@app.get("/api/floor/export/{kind}")
def floor_export(kind: str):
    if kind not in ("json", "yaml") or studio.floor.result is None:
        raise HTTPException(404)
    return FileResponse(studio.floor.directory / f"extrinsics.{kind}", filename=f"extrinsics.{kind}")


class FloorPixel(BaseModel):
    u: float = Field(ge=0, allow_inf_nan=False)
    v: float = Field(ge=0, allow_inf_nan=False)
    height_m: float = Field(default=0, ge=0, le=10, allow_inf_nan=False)


@app.post("/api/floor/point")
def floor_point(body: FloorPixel):
    with studio.lock:
        result = studio.floor.result
        if result is None or not result["valid"]:
            raise ValueError("Сначала получите однозначную привязку пола.")
        if body.u >= result["intrinsics"]["image_width"] or body.v >= result["intrinsics"]["image_height"]:
            raise ValueError("Пиксель вне изображения.")
        return pixel_to_floor([body.u, body.v], result, body.height_m)


def robot_state():
    with studio.lock:
        floor = studio.floor.result
        state = studio.robot.state()
        state.update(mode=studio.mode, performance=dict(studio.performance),
                     connected=studio.camera.thread is not None and studio.camera.thread.is_alive(),
                     camera_error=studio.camera.error, floor_ready=bool(floor and floor["valid"]))
        if floor:
            state["map"] = {"camera_floor_m": floor["camera_position_floor_m"],
                            "placements": [[p["x_m"], p["y_m"]] for p in floor["placements"]],
                            "floor_board_m": [b/1000 for b in studio.floor.board.size] if studio.floor.board else None}
        return state


@app.get("/api/robot/state")
def robot_state_endpoint():
    return robot_state()


_footprint_cache = {}


@app.get("/api/robot/map")
def robot_map():
    with studio.lock:
        floor = studio.floor.result
    if not floor or not floor["valid"]:
        raise ValueError("Сначала получите привязку камеры к полу (шаг 2).")
    key = floor.get("created_at")
    if key not in _footprint_cache:
        _footprint_cache.clear()
        _footprint_cache[key] = floor_footprint(floor)
    return {"footprint": _footprint_cache[key], "camera_floor_m": floor["camera_position_floor_m"],
            "placements": [[p["x_m"], p["y_m"]] for p in floor["placements"]],
            "floor_board_m": [b/1000 for b in studio.floor.board.size] if studio.floor.board else None}


@app.get("/api/robot/pose")
def robot_pose():
    """Latest robot pose for other programs; robot is null when not visible."""
    with studio.lock:
        live = studio.robot.state()["live"]
        return {"robot": live and live["robot"], "time": live and live["time"], "tags": live and live["tags"]}


@app.get("/api/robot/events")
def robot_events():
    def events():
        previous = None
        while not studio.closing.is_set():
            with studio.lock:
                live = studio.robot.state()["live"]
                sequence = studio.robot.sequence
            key = (sequence, live is None)
            if key != previous:
                previous = key
                payload = {"robot": live and live["robot"], "time": live and live["time"],
                           "tags": live and live["tags"], "reason": live["reason"] if live else "Робот не виден."}
                yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(.03)
    return StreamingResponse(events(), media_type="text/event-stream", headers={"Cache-Control": "no-store"})


def robot_guard():
    studio.idle()
    if studio.mode != "robot":
        raise ValueError("Сначала включите шаг 3.")


class RobotSettings(BaseModel):
    height_m: float | None = Field(default=None, ge=0, le=3, allow_inf_nan=False)
    robot_length_m: float | None = Field(default=None, gt=0, le=5, allow_inf_nan=False)
    robot_width_m: float | None = Field(default=None, gt=0, le=5, allow_inf_nan=False)


@app.post("/api/robot/settings")
def robot_settings(body: RobotSettings):
    with studio.lock:
        robot_guard()
        if body.height_m is not None:
            studio.robot.height_m = body.height_m
            studio.robot.trail.clear()
        size = list(studio.robot.robot_size_m)
        if body.robot_length_m is not None:
            size[0] = body.robot_length_m
        if body.robot_width_m is not None:
            size[1] = body.robot_width_m
        studio.robot.robot_size_m = size
        studio.robot.save()
    return robot_state()


class RobotRecording(BaseModel):
    kind: str


@app.post("/api/robot/record")
def robot_record(body: RobotRecording):
    if body.kind not in ("spin", "drive"):
        raise ValueError("Неизвестная запись.")
    with studio.lock:
        robot_guard()
        if studio.robot.height_m is None:
            raise ValueError("Сначала задайте высоту доски над полом.")
        if body.kind == "drive" and studio.robot.spin is None:
            raise ValueError("Сначала определите центр вращения.")
        studio.robot.recording = body.kind
        studio.robot.recorded = []
    return robot_state()


@app.post("/api/robot/stop")
def robot_stop():
    with studio.lock:
        robot = studio.robot
        kind, poses = robot.recording, list(robot.recorded)
        robot.recording, robot.recorded = None, []
        if kind == "spin":
            result = fit_spin(poses)
            robot.spin = result
            robot.offset_m = result["offset_m"]
        elif kind == "drive":
            result = fit_drive(poses, robot.offset_m)
            robot.drive = result
            robot.heading_offset_rad = result["heading_offset_rad"]
        robot.trail.clear()
        robot.save()
    return robot_state()


@app.post("/api/robot/reset")
def robot_reset():
    with studio.lock:
        robot_guard()
        robot = studio.robot
        robot.offset_m, robot.heading_offset_rad, robot.spin, robot.drive = [0., 0.], 0., None, None
        robot.trail.clear()
        robot.save()
    return robot_state()


@app.get("/api/robot/export")
def robot_export():
    with studio.lock:
        data = {"robot": studio.robot.calibration(), "floor": studio.floor.result}
    return Response(json.dumps(data, ensure_ascii=False, indent=2), media_type="application/json",
                    headers={"Content-Disposition": "attachment; filename=robot_tracking.json"})


app.mount("/", StaticFiles(directory=Path(__file__).parent / "web", html=True), name="web")
