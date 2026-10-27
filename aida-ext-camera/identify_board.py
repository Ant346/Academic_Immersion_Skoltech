#!/usr/bin/env python3
"""Compare standard AprilTag and ArUco dictionaries against an image.

Writes decoded IDs/corners to JSON and an annotated image. This identifies
marker families, not physical dimensions or a board's complete layout.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def identify(image, scales=(1, 2, 4)):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = []
    # Dictionaries with 50/100/250/1000 entries share prefixes; compare their
    # largest variants and explicitly avoid claiming a particular subset.
    aruco_names = [
        "DICT_4X4_1000", "DICT_5X5_1000", "DICT_6X6_1000",
        "DICT_7X7_1000", "DICT_ARUCO_ORIGINAL", "DICT_ARUCO_MIP_36h12",
        "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
        "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
    ]
    aruco_detectors = []
    for name in aruco_names:
        if not hasattr(cv2.aruco, name):
            continue
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        for border in (1, 2, 3):
            params = cv2.aruco.DetectorParameters()
            params.markerBorderBits = border
            params.errorCorrectionRate = 0.0
            params.detectInvertedMarker = True
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            aruco_detectors.append((name, border, cv2.aruco.ArucoDetector(dictionary, params)))

    for mirrored in (False, True):
        original = cv2.flip(gray, 1) if mirrored else gray
        best_by_family = {}
        for scale in scales:
            view = cv2.resize(original, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_CUBIC)
            for family, border, detector in aruco_detectors:
                corners, ids, _ = detector.detectMarkers(view)
                if ids is None:
                    continue
                detections = []
                for corner, tag_id in zip(corners, ids.flatten()):
                    # OpenCV resize maps pixel centers with a half-pixel offset.
                    points = (corner.reshape(4, 2) + 0.5) / scale - 0.5
                    if mirrored:
                        points[:, 0] = image.shape[1] - 1 - points[:, 0]
                    detections.append({"id": int(tag_id), "corners": points.tolist()})
                result = {
                    "family": family, "mirrored": mirrored, "scale": scale,
                    "border_bits": border,
                    "unique_ids": len({d["id"] for d in detections}),
                    "count": len(detections), "detections": detections,
                }
                key = (family, border)
                previous = best_by_family.get(key)
                if previous is None or result["unique_ids"] > previous["unique_ids"]:
                    best_by_family[key] = result
        results.extend(best_by_family.values())
    return sorted(results, key=lambda r: (r["unique_ids"], r["count"]), reverse=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Photo/scan of the board")
    parser.add_argument("--output", type=Path, default=Path("board_identification"))
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f"Cannot read image: {args.image}")
    results = identify(image)
    report = {
        "image": str(args.image), "opencv_version": cv2.__version__,
        "error_correction_rate": 0.0,
        "candidates": results,
        "notes": [
            "No detections does not exclude a family: try a sharper/larger image.",
            "Only standard families available in OpenCV are compared; custom families may be missed.",
            "Aruco subset size cannot be distinguished from shared marker IDs.",
            "Physical dimensions and complete board layout are not inferred.",
            "Multiple matching tags support identification; one match is insufficient.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    json_path = args.output.with_suffix(".json")
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not results:
        print(f"No supported markers decoded. Details: {json_path}")
        return 2
    for result in results:
        print(f"{result['family']}: {result['count']} detections, "
              f"{result['unique_ids']} unique IDs, mirrored={result['mirrored']}, "
              f"scale={result['scale']}, border_bits={result['border_bits']}")
    best = results[0]
    annotated = image.copy()
    for detection in best["detections"]:
        corners = np.rint(detection["corners"]).astype(np.int32)
        cv2.polylines(annotated, [corners], True, (0, 255, 0), 2)
        position = tuple(np.rint(corners.mean(axis=0)).astype(int))
        cv2.putText(annotated, str(detection["id"]), position,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    cv2.putText(annotated, f"{best['family']}; mirrored={best['mirrored']}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    image_path = args.output.with_suffix(".png")
    if not cv2.imwrite(str(image_path), annotated):
        raise OSError(f"Cannot write image: {image_path}")
    print(f"Report: {json_path}\nAnnotated image: {image_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
