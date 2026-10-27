"""AprilGrid observations and OpenCV pinhole intrinsics calibration."""
from dataclasses import asdict, dataclass
from datetime import datetime, timezone

import cv2
import numpy as np
from tag_edges import extend_from_tags, find_quads, identify_quads, matches_code, refine_quad, recover_grid

DETECTOR_VERSION = "aprilgrid-border2-edges-v2"


@dataclass(frozen=True)
class Board:
    columns: int = 11
    rows: int = 8
    tag_size_mm: float = 25.0
    gap_mm: float = 7.5
    first_id: int = 0
    # The supplied calib.io board has ID 0 at bottom left and each decoded
    # OpenCV tag rotated 180 degrees relative to the board's printed upright.
    rotation: int = 180
    ids_bottom_up: bool = True
    border_bits: int = 2

    @classmethod
    def parse(cls, values):
        board = cls(**values)
        if not (2 <= board.columns <= 30 and 2 <= board.rows <= 30):
            raise ValueError("Число строк и столбцов должно быть от 2 до 30.")
        if not (1 <= board.tag_size_mm <= 500 and 0 < board.gap_mm <= 500):
            raise ValueError("Проверьте размеры маркера и промежутка в миллиметрах.")
        if board.rotation not in (0, 90, 180, 270):
            raise ValueError("Некорректный поворот маркеров.")
        if board.border_bits not in (1, 2, 3):
            raise ValueError("Толщина чёрной рамки должна быть 1, 2 или 3 ячейки.")
        if board.first_id < 0 or board.first_id + board.rows * board.columns > 587:
            raise ValueError("ID выходят за пределы семейства tag36h11 (0–586).")
        return board

    @property
    def size(self):
        return (self.columns * self.tag_size_mm + (self.columns - 1) * self.gap_mm,
                self.rows * self.tag_size_mm + (self.rows - 1) * self.gap_mm)

    def cell(self, tag_id):
        """Printed (row from top, column from left) of a tag."""
        index = int(tag_id) - self.first_id
        if not 0 <= index < self.rows * self.columns:
            raise ValueError("Маркер не относится к доске.")
        row, col = divmod(index, self.columns)
        if self.ids_bottom_up:
            row = self.rows - 1 - row
        return row, col

    def points(self, tag_id):
        row, col = self.cell(tag_id)
        pitch = self.tag_size_mm + self.gap_mm
        x, y, s = col * pitch, row * pitch, self.tag_size_mm
        points = np.array([[x, y, 0], [x+s, y, 0], [x+s, y+s, 0], [x, y+s, 0]], np.float32)
        return np.roll(points, -(self.rotation // 90), axis=0)


def init_detector_process(threads):
    cv2.setNumThreads(threads)


def detect(frame, board, distant=False, search_roi=None):
    if search_roi is not None:
        x, y, w, h = search_roi
        observation = detect(frame[y:y+h, x:x+w], board, distant=distant)
        observation["image_points"] += np.float32([x, y])
        if observation["polygon"] is not None:
            observation["polygon"] += np.float32([x, y])
        observation["image_size"] = (frame.shape[1], frame.shape[0])
        observation["quality"]["area_percent"] = round(observation["quality"]["area_percent"]*(w*h)/(frame.shape[0]*frame.shape[1]), 1)
        return observation
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters()
    params.errorCorrectionRate = 1.0 if distant else 0.0
    params.markerBorderBits = board.border_bits
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    params.adaptiveThreshWinSizeMax = 53
    params.minMarkerPerimeterRate = .003 if distant else .015
    if distant:
        params.minCornerDistanceRate = .02
        params.minMarkerDistanceRate = .02
        params.perspectiveRemovePixelPerCell = 8
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if distant:
        dictionary.maxCorrectionBits = 1
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    found = {}
    duplicate = False
    rejected_edges = 0
    seeded = 0
    anchors = []
    smooth = cv2.GaussianBlur(gray.astype(np.float32), (3, 3), .6)
    # Upscaling is important for the supplied densely printed target. Refine
    # in original pixels afterwards; calibration always uses native resolution.
    def search_views():
        nonlocal seeded
        yield gray, 1, (0, 0)
        if not distant:
            yield cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC), 2, (0, 0)
            return
        if not found:
            # Distant AprilGrid tags touch their corner squares, so the generic
            # detector often finds no quad at all; use our own quad search.
            for tag_id, points in identify_quads(gray, board, find_quads(gray)).items():
                found[tag_id] = points
                anchors.append(points)
                seeded += 1
        h, w = gray.shape

        def around_anchors():
            points = np.concatenate(anchors)
            x, y, rw, rh = cv2.boundingRect(points)
            edge = max(np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1).mean() for p in anchors)
            margin = int(edge*np.hypot(board.columns, board.rows)*1.4)
            x0, y0 = max(0, x-margin), max(0, y-margin)
            x1, y1 = min(w, x+rw+margin), min(h, y+rh+margin)
            roi = gray[y0:y1, x0:x1]
            for scale in (2, 3):
                if roi.size*scale*scale <= 18_000_000:
                    yield cv2.resize(roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC), scale, (x0, y0)

        if anchors:
            yield from around_anchors()
            return
        # Nothing on the board found: the 2x tile scan that used to follow
        # took ~6-30 s per frame and is superseded by find_quads above.

    first_view = True
    for image, scale, offset in search_views():
        corners, ids, rejected = detector.detectMarkers(image)
        candidates = []
        if distant and not found:
            for corner in rejected:
                points = ((corner.reshape(4, 2) + .5) / scale - .5 + np.array(offset)).astype(np.float32)
                sides = np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)
                if (sides.min() >= 8 and np.all(points >= 4) and np.all(points[:, 0] < gray.shape[1]-4)
                        and np.all(points[:, 1] < gray.shape[0]-4)):
                    candidates.append(points)
        seen = set()
        if ids is None:
            ids, corners = np.empty((0, 1), np.int32), ()
        for corner, tag_id in zip(corners, ids.flatten()):
            tag_id = int(tag_id)
            if not board.first_id <= tag_id < board.first_id + board.rows * board.columns:
                continue
            if tag_id in seen:
                duplicate = True
            seen.add(tag_id)
            points = ((corner.reshape(4, 2) + .5) / scale - .5 + np.array(offset)).astype(np.float32)
            anchors.append(points)
            if tag_id in found:
                continue
            if np.any(points < 4) or np.any(points[:, 0] >= gray.shape[1]-4) or np.any(points[:, 1] >= gray.shape[0]-4):
                continue
            if min(np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)) < (8 if distant else 16):
                continue
            refined = refine_quad(smooth, points, small=distant)
            if refined is None or not matches_code(gray, refined, tag_id, board.border_bits):
                rejected_edges += 1
                continue
            found[tag_id] = refined
        if distant and not found and candidates:
            # Nothing decoded here: try the decoder's rejected quads, whose
            # blurred distant codes it could not read.
            for tag_id, points in identify_quads(gray, board, candidates).items():
                found[tag_id] = points
                anchors.append(points)
                seeded += 1
        if distant and first_view and found and len(found) < board.columns*board.rows:
            # Grow over the layout before paying for upscaled views; boards
            # with a physically missing tag would otherwise never finish early.
            extended = extend_from_tags(gray, board, found)
            found.update(extended)
            seeded += len(extended)
            # The layout growth already tried every missing tag; upscaled views
            # rarely add more once most of the board is measured.
            if len(found) >= .6*board.columns*board.rows:
                break
        first_view = False
        if len(found) == board.columns*board.rows:
            break
    recovered_count = seeded
    if distant and len(found) < board.columns*board.rows*.7:
        for tag_id, points in recover_grid(gray, board, anchors).items():
            if tag_id not in found:
                found[tag_id] = points
                recovered_count += 1
    if distant and 0 < len(found) < board.columns*board.rows:
        extended = extend_from_tags(gray, board, found)
        found.update(extended)
        recovered_count += len(extended)
    ids = sorted(found)
    image_points = np.concatenate([found[i] for i in ids]) if ids else np.empty((0, 2), np.float32)
    object_points = np.concatenate([board.points(i) for i in ids]) if ids else np.empty((0, 3), np.float32)
    h, w = gray.shape
    quality = {"tags": len(ids), "ids": ids, "ready": False, "reason": "Покажите доску камере.",
               "sharpness": 0, "layout_error_px": None, "area_percent": 0,
               "rejected_edges": rejected_edges, "detector_version": DETECTOR_VERSION}
    quality["recovered_tags"] = recovered_count
    quality["min_tag_edge_px"] = round(float(min((np.min(np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1)) for p in found.values()), default=0)), 1)
    polygon = None
    if len(ids) >= 4:
        hull = cv2.convexHull(image_points)
        quality["area_percent"] = round(100 * cv2.contourArea(hull) / (w*h), 1)
        x, y, rw, rh = cv2.boundingRect(hull)
        quality["sharpness"] = round(float(cv2.Laplacian(gray[y:y+rh, x:x+rw], cv2.CV_64F).var()), 1)
        homography, _ = cv2.findHomography(object_points[:, :2], image_points, 0)
        if homography is not None:
            projected = cv2.perspectiveTransform(object_points[:, None, :2], homography)[:, 0]
            error = float(np.median(np.linalg.norm(projected-image_points, axis=1)))
            quality["layout_error_px"] = round(error, 2)
            bw, bh = board.size
            polygon = cv2.perspectiveTransform(np.float32([[[0, 0], [bw, 0], [bw, bh], [0, bh]]]), homography)[0]
        rows = {board.cell(i)[0] for i in ids}
        columns = {board.cell(i)[1] for i in ids}
        if duplicate:
            quality["reason"] = "Повторяются ID: уберите другие маркеры из кадра."
        elif len(ids) < (4 if distant else 8) or len(rows) < 2 or len(columns) < (2 if distant else 3):
            quality["reason"] = "Нужно ≥ 4 маркеров в 2 рядах и 2 столбцах." if distant else "Нужно ≥ 8 маркеров минимум в 2 рядах и 3 столбцах."
        elif polygon is None or quality["layout_error_px"] > max(5, .004*np.hypot(w, h)):
            quality["reason"] = "Раскладка не совпадает: проверьте строки, столбцы и ориентацию ID."
        elif quality["sharpness"] < 25:
            quality["reason"] = "Кадр размыт. Остановите доску и проверьте фокус."
        else:
            quality.update(ready=True, reason="Можно сохранить кадр. Меняйте положение и наклон доски.")
    return {"quality": quality, "image_points": image_points, "object_points": object_points,
            "polygon": polygon, "image_size": (w, h)}


def too_similar(observation, samples):
    polygon = observation["polygon"]
    if polygon is None:
        return False
    diagonal = np.hypot(*observation["image_size"])
    return any(np.mean(np.linalg.norm(polygon-np.array(s["polygon"]), axis=1)) / diagonal < .025
               for s in samples)


def calibrate(samples, image_size, board, camera):
    if len(samples) < 12:
        raise ValueError("Нужно минимум 12 разных кадров; рекомендуется 25–35.")
    objects = [np.array(s["object_points"], np.float32) for s in samples]
    images = [np.array(s["image_points"], np.float32) for s in samples]
    rms, matrix, distortion, rvecs, tvecs, intrinsic_std, _, per_view = cv2.calibrateCameraExtended(
        objects, images, tuple(image_size), None, None,
        flags=cv2.CALIB_USE_LU,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    if not np.isfinite(rms) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ValueError("Расчёт не сошёлся. Наберите кадры с другими наклонами доски.")
    w, h = image_size
    warnings = []
    if rms > 1:
        warnings.append("Ошибка выше 1 px: проверьте размеры и плоскость доски, удалите плохие кадры.")
    if len(samples) < 25:
        warnings.append("Меньше 25 кадров: добавьте разные наклоны и положения по краям изображения.")
    if not (.1*w < matrix[0, 0] < 10*w and .1*h < matrix[1, 1] < 10*w
            and 0 < matrix[0, 2] < w and 0 < matrix[1, 2] < h):
        warnings.append("Необычные intrinsics: проверьте раскладку доски и разнообразие наклонов.")
    coverage = np.zeros((4, 6), dtype=int)
    for points in images:
        cells = np.unique(np.column_stack((np.clip((points[:, 1]*4/h).astype(int), 0, 3),
                                           np.clip((points[:, 0]*6/w).astype(int), 0, 5))), axis=0)
        for row, col in cells:
            coverage[row, col] += 1
    if np.count_nonzero(coverage) < 18:
        warnings.append("Плохо покрыты края кадра: добавьте наблюдения доски по всей площади изображения.")
    return {
        "created_at": datetime.now(timezone.utc).isoformat(), "model": "opencv_pinhole",
        "distortion_model": "plumb_bob", "distortion_order": ["k1", "k2", "p1", "p2", "k3"],
        "image_width": w, "image_height": h, "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.flatten().tolist(), "rms_px": float(rms),
        "intrinsics_std": intrinsic_std.flatten().tolist(), "frame_count": len(samples),
        "per_view_errors": [{"id": sample["id"], "rms_px": float(error)}
                            for sample, error in zip(samples, per_view.flatten())],
        "board": asdict(board), "family": "tag36h11", "camera": camera,
        "coverage": coverage.tolist(), "warnings": warnings,
        "opencv_version": cv2.__version__,
        "detector_version": DETECTOR_VERSION,
    }
