"""Metric camera-to-floor calibration with a freely moving, flat AprilGrid.

The first board placement fixes the otherwise unobservable X/Y/yaw gauge.
Subsequent board placements each have three free in-plane coordinates. Intrinsics
remain fixed. All lengths in this module are metres; source board points are mm.
"""
from datetime import datetime, timezone
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


def transform(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = cv2.Rodrigues(np.asarray(rotation, float).reshape(3))[0]
    matrix[:3, 3] = np.asarray(translation).reshape(3)
    return matrix


def reference_transform(board, thickness_m=0.):
    # World origin: floor directly underneath bottom-left corner of printed grid.
    # X right along printed rows, Y toward top of the printed grid, Z upward.
    matrix = np.diag([1., -1., -1., 1.])
    matrix[:3, 3] = [0., board.size[1]/1000, thickness_m]
    return matrix


def pose_candidates(observation, intrinsics):
    objects = np.asarray(observation["object_points"], np.float64)/1000
    pixels = np.asarray(observation["image_points"], np.float64)
    if len(objects) < 16:
        raise ValueError("Нужно минимум 4 маркера с проверенными углами.")
    k, d = np.array(intrinsics["camera_matrix"]), np.array(intrinsics["distortion_coefficients"])
    undistorted = cv2.undistortPoints(pixels[:, None], k, d, P=k)[:, 0]
    _, mask = cv2.findHomography(objects[:, :2], undistorted, cv2.RANSAC, 2.5)
    if mask is None:
        raise ValueError("Не удалось проверить геометрию доски.")
    # Reject whole tags when any of their four corners disagrees with the plane.
    good = mask.reshape(-1, 4).all(axis=1)
    if np.count_nonzero(good) < 4:
        raise ValueError("Слишком мало согласованных маркеров; приблизьте доску.")
    keep = np.repeat(good, 4)
    objects, pixels = objects[keep], pixels[keep]
    retval, rotations, translations, _ = cv2.solvePnPGeneric(objects, pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if retval:
        for rotation, translation in zip(rotations, translations):
            rotation, translation = cv2.solvePnPRefineLM(objects, pixels, k, d, rotation.copy(), translation.copy())
            matrix = transform(rotation, translation)
            camera_points = objects @ matrix[:3, :3].T + matrix[:3, 3]
            if np.any(camera_points[:, 2] <= .02):
                continue
            projected = cv2.projectPoints(objects, rotation, translation, k, d)[0][:, 0]
            rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))
            if not np.isfinite(rms):
                continue
            candidates.append({"matrix": matrix, "rms_px": rms})
    candidates.sort(key=lambda candidate: candidate["rms_px"])
    if not candidates or candidates[0]["rms_px"] > 3.:
        raise ValueError("Поза доски ненадёжна: ошибка выше 3 px или доска не читается.")
    return candidates, objects, pixels, keep


def _aggregate_window(observations):
    """Average only stationary real image measurements, without board projection."""
    if len(observations) < 3:
        raise ValueError("Подержите доску неподвижно: нужны минимум 3 свежих кадра.")
    observations = observations[-5:]
    if observations[-1]["timestamp"]-observations[0]["timestamp"] < .25:
        raise ValueError("Подержите доску неподвижно ещё немного.")
    common = set(observations[0]["quality"]["ids"])
    for observation in observations[1:]:
        common &= set(observation["quality"]["ids"])
    ids = sorted(common)
    if len(ids) < 4:
        raise ValueError("Меньше 4 общих маркеров в серии; приблизьте или остановите доску.")
    batches = []
    lookup = {}
    for observation in observations:
        lookup = {tag: index for index, tag in enumerate(observation["quality"]["ids"])}
        batches.append(np.array([observation["image_points"].reshape(-1, 4, 2)[lookup[tag]] for tag in ids]))
    batches = np.array(batches)
    average = np.median(batches, axis=0)
    jitter = float(np.max(np.median(np.linalg.norm(batches-average, axis=3), axis=(1, 2))))
    if jitter > 1.2:
        raise ValueError("Доска движется. Остановите её на полу перед сохранением.")
    last = observations[-1]
    objects = last["object_points"].reshape(-1, 4, 3)
    result = dict(last)
    result.update(image_points=average.reshape(-1, 2),
                  object_points=np.array([objects[lookup[tag]] for tag in ids]).reshape(-1, 3),
                  quality={**last["quality"], "ids": ids, "tags": len(ids)},
                  burst_frames=len(observations), jitter_px=jitter)
    return result


def aggregate(observations):
    error = None
    for count in range(min(5, len(observations)), 2, -1):
        try:
            return _aggregate_window(observations[-count:])
        except ValueError as failure:
            error = failure
    raise error or ValueError("Подержите доску неподвижно: нужны минимум 3 свежих кадра.")


def fit_floor(samples, intrinsics, board, thickness_mm=0.):
    if len(samples) < 6:
        raise ValueError("Нужно минимум 6 разных положений доски на полу, включая опорное.")
    k, d = np.array(intrinsics["camera_matrix"]), np.array(intrinsics["distortion_coefficients"])
    reference = reference_transform(board, thickness_mm/1000)
    models = []
    for sample in samples:
        candidates, objects, pixels, _ = pose_candidates(sample, intrinsics)
        models.append((candidates, objects, pixels))

    def unpack(parameters, index):
        if index == 0:
            return np.zeros(3)
        return parameters[6+(index-1)*3:9+(index-1)*3]

    def world_points(objects, placement):
        points = objects @ reference[:3, :3].T + reference[:3, 3]
        x, y, yaw = placement
        c, s = np.cos(yaw), np.sin(yaw)
        points[:, :2] = points[:, :2] @ np.array([[c, s], [-s, c]]) + [x, y]
        return points

    def residual(parameters):
        output = []
        for index, (_, objects, pixels) in enumerate(models):
            points = world_points(objects, unpack(parameters, index))
            projected = cv2.projectPoints(points, parameters[:3], parameters[3:6], k, d)[0][:, 0]
            output.append((projected-pixels).ravel())
        return np.concatenate(output)

    sparsity = lil_matrix((sum(2*len(m[1]) for m in models), 6+3*(len(models)-1)), dtype=int)
    row = 0
    for index, (_, objects, _) in enumerate(models):
        end = row+2*len(objects)
        sparsity[row:end, :6] = 1
        if index:
            sparsity[row:end, 6+3*(index-1):9+3*(index-1)] = 1
        row = end

    solutions = []
    for first in models[0][0][:2]:
        camera_world = first["matrix"] @ np.linalg.inv(reference)
        world_camera = np.linalg.inv(camera_world)
        if world_camera[2, 3] <= thickness_mm/1000:
            continue
        values = [*cv2.Rodrigues(camera_world[:3, :3])[0].flatten(), *camera_world[:3, 3]]
        for candidates, _, _ in models[1:]:
            best = min(candidates, key=lambda candidate:
                       np.linalg.norm((world_camera @ candidate["matrix"] @ np.linalg.inv(reference))[:3, :3][2, :2])
                       + abs((world_camera @ candidate["matrix"] @ np.linalg.inv(reference))[2, 3]))
            relative = world_camera @ best["matrix"] @ np.linalg.inv(reference)
            values.extend([relative[0, 3], relative[1, 3], np.arctan2(relative[1, 0], relative[0, 0])])
        fit = least_squares(residual, values, loss="soft_l1", f_scale=1., max_nfev=160, x_scale="jac", jac_sparsity=sparsity)
        if not fit.success or not np.isfinite(fit.x).all():
            continue
        camera_world = transform(fit.x[:3], fit.x[3:6])
        world_camera = np.linalg.inv(camera_world)
        if world_camera[2, 3] <= thickness_mm/1000:
            continue
        errors, per_view, placements = [], [], []
        for index, (_, objects, pixels) in enumerate(models):
            placement = unpack(fit.x, index)
            points = world_points(objects, placement)
            if np.any((points @ camera_world[:3, :3].T + camera_world[:3, 3])[:, 2] <= 0):
                break
            projected = cv2.projectPoints(points, fit.x[:3], fit.x[3:6], k, d)[0][:, 0]
            squared = np.sum((projected-pixels)**2, axis=1)
            errors.extend(squared)
            per_view.append({"id": samples[index]["id"], "rms_px": float(np.sqrt(np.mean(squared)))})
            placements.append({"id": samples[index]["id"], "x_m": float(placement[0]),
                               "y_m": float(placement[1]), "yaw_deg": float(np.degrees(placement[2]))})
        if len(per_view) != len(models):
            continue
        rms = float(np.sqrt(np.mean(errors)))
        # Conservative corner-noise floor. This covariance excludes errors in
        # intrinsics, printed dimensions, board flatness and floor planarity.
        noise = max(.7, rms/np.sqrt(2))
        jacobian = fit.jac.toarray() if hasattr(fit.jac, "toarray") else fit.jac
        normal_matrix = jacobian.T @ jacobian
        condition = float(np.linalg.cond(normal_matrix))
        covariance = np.linalg.pinv(normal_matrix, rcond=1e-12)*noise**2
        solutions.append((rms, fit, camera_world, world_camera, per_view, placements, covariance[:6, :6], condition))
    if not solutions:
        raise ValueError("Не удалось согласовать одну плоскость пола. Проверьте неподвижность камеры и положение доски.")
    solutions.sort(key=lambda value: value[0])
    rms, fit, camera_world, world_camera, per_view, placements, covariance, condition = solutions[0]
    ambiguous = False
    for other in solutions[1:]:
        distance = np.linalg.norm(other[3][:3, 3]-world_camera[:3, 3])
        angle = np.linalg.norm(cv2.Rodrigues(other[2][:3, :3] @ camera_world[:3, :3].T)[0])
        if other[0]-rms < .15 and (distance > .05 or angle > np.radians(1)):
            ambiguous = True
    warnings = []
    if condition > 1e12:
        warnings.append("Геометрия плохо определена: добавьте разные положения по всей площади пола.")
    if ambiguous:
        warnings.append("Две близкие по ошибке позы: добавьте положения доски ближе к камере и по краям зоны.")
    if rms > 2:
        warnings.append("Ошибка выше 2 px: возможны наклон/подъём доски, движение камеры или неточные intrinsics.")
    xy = np.array([[p["x_m"], p["y_m"]] for p in placements])
    spread = np.linalg.svd(xy-xy.mean(axis=0), compute_uv=False)/np.sqrt(len(xy))
    if spread[-1] < .1:
        warnings.append("Положения почти на одной линии: добавьте точки поперёк неё.")
    parameters = fit.x[:6]
    def height_at(values):
        return np.linalg.inv(transform(values[:3], values[3:6]))[2, 3]
    gradient = np.zeros(6)
    for i in range(6):
        changed = parameters.copy(); changed[i] += 1e-5
        gradient[i] = (height_at(changed)-height_at(parameters))/1e-5
    height_std = float(np.sqrt(max(0., gradient @ covariance @ gradient)))
    if height_std > .02:
        warnings.append("Высота камеры слабо определена: приблизьте доску или используйте более крупный макет.")
    homography = k @ np.column_stack((camera_world[:3, 0], camera_world[:3, 1], camera_world[:3, 3]))
    return {
        "created_at": datetime.now(timezone.utc).isoformat(), "units": "metres",
        "method": "fixed_intrinsics_shared_floor_free_SE2_board_placements",
        "valid": bool(not ambiguous and rms <= 3 and spread[-1] >= .05 and condition < 1e12 and height_std < .05),
        "rms_px": rms, "frame_count": len(samples), "board_thickness_mm": thickness_mm,
        "T_camera_from_floor": camera_world.tolist(), "T_floor_from_camera": world_camera.tolist(),
        "camera_position_floor_m": world_camera[:3, 3].tolist(),
        "camera_height_m": float(world_camera[2, 3]), "height_std_conditional_m": height_std,
        "floor_normal_camera": camera_world[:3, 2].tolist(),
        "plane_equation_camera": [*camera_world[:3, 2].tolist(), float(-camera_world[:3, 2] @ camera_world[:3, 3])],
        "H_floor_to_undistorted_pixels": homography.tolist(),
        "H_undistorted_pixels_to_floor": np.linalg.inv(homography).tolist(),
        "camera_pose_parameters": parameters.tolist(), "camera_pose_covariance_conditional": covariance.tolist(),
        "uncertainty_scope": "conditional_on_fixed_intrinsics_board_dimensions_and_flat_floor",
        "reference_id": samples[0]["id"], "reference_axes": "origin below bottom-left printed grid corner; X right, Y toward printed top, Z up",
        "per_view_errors": per_view, "placements": placements, "warnings": warnings,
        "intrinsics": intrinsics,
    }


def pixel_to_floor(pixel, result, height_m=0.):
    intrinsics = result["intrinsics"]
    k, d = np.array(intrinsics["camera_matrix"]), np.array(intrinsics["distortion_coefficients"])
    ray = cv2.undistortPoints(np.float64(pixel).reshape(1, 1, 2), k, d)[0, 0]
    camera_ray = np.r_[ray, 1.]
    def intersection(parameters):
        matrix = np.linalg.inv(transform(parameters[:3], parameters[3:6]))
        origin, direction = matrix[:3, 3], matrix[:3, :3] @ camera_ray
        if abs(direction[2]) < .015:
            raise ValueError("Точка слишком близко к горизонту; координаты ненадёжны.")
        distance = (height_m-origin[2])/direction[2]
        if distance <= 0:
            raise ValueError("Луч не пересекает выбранную плоскость перед камерой.")
        return origin + distance*direction
    parameters = np.array(result["camera_pose_parameters"])
    point = intersection(parameters)
    jacobian = np.zeros((2, 6))
    for i in range(6):
        changed = parameters.copy(); changed[i] += 1e-5
        jacobian[:, i] = (intersection(changed)[:2]-point[:2])/1e-5
    covariance = jacobian @ np.array(result["camera_pose_covariance_conditional"]) @ jacobian.T
    return {"x_m": float(point[0]), "y_m": float(point[1]), "z_m": float(height_m),
            "xy_std_conditional_m": np.sqrt(np.maximum(np.diag(covariance), 0)).tolist()}
