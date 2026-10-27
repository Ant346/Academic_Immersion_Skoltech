"""Measure each decoded tag's edges independently in the original image.

Fit four black-to-white transitions and intersect their lines. Never move corners
using board projections or an unconstrained search among nearby code features.
"""
from functools import lru_cache
import cv2
import numpy as np


def edge_line(gray, a, b, centre, small=False):
    tangent = b-a
    length = np.linalg.norm(tangent)
    if length < (8 if small else 12):
        return None
    normal = np.array([-tangent[1], tangent[0]]) / length
    if np.dot((a+b)/2-centre, normal) < 0:
        normal = -normal
    radius = min(10., max(1., length*.085)) if small else min(10., max(3., length*.085))
    offsets = np.arange(-radius, radius+.25, .5, dtype=np.float32)
    positions = a + np.linspace(.18, .82, 24, dtype=np.float32)[:, None] * tangent
    grid = positions[:, None, :] + offsets[None, :, None]*normal
    values = cv2.remap(gray, grid[:, :, 0].astype(np.float32), grid[:, :, 1].astype(np.float32),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    gradients = np.diff(values, axis=1)
    peaks = np.argmax(gradients, axis=1)
    strengths = gradients[np.arange(len(peaks)), peaks]
    valid = strengths > (4 if small else 7)
    if np.count_nonzero(valid) < 16:
        return None
    displacement = (offsets[peaks]+offsets[peaks+1])/2
    # Subpixel peak of the derivative; no unconstrained 2-D corner search.
    interior = (peaks > 0) & (peaks < gradients.shape[1]-1)
    rows = np.flatnonzero(interior)
    left = gradients[rows, peaks[rows]-1]
    middle = gradients[rows, peaks[rows]]
    right = gradients[rows, peaks[rows]+1]
    denominator = left-2*middle+right
    shift = np.divide(.5*(left-right), denominator, out=np.zeros_like(left), where=np.abs(denominator)>1e-5)
    displacement[rows] += np.clip(shift, -.5, .5)*.5
    points = positions[valid] + displacement[valid, None]*normal
    vx, vy, x, y = cv2.fitLine(points.astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).flatten()
    line = np.array([-vy, vx, vy*x-vx*y], dtype=np.float64)
    residuals = np.abs(points @ line[:2] + line[2])
    if percentiles(residuals, 80)[0] > .8:
        return None
    return line


def refine_quad(gray, proposed, small=False):
    centre = proposed.mean(axis=0)
    lines = [edge_line(gray, proposed[i], proposed[(i+1)%4], centre, small=small) for i in range(4)]
    if any(line is None for line in lines):
        return None
    corners = []
    for i in range(4):
        point = np.cross(lines[i-1], lines[i])
        if abs(point[2]) < .1:
            return None
        corners.append(point[:2]/point[2])
    corners = np.array(corners, dtype=np.float32)
    if not np.isfinite(corners).all() or not cv2.isContourConvex(corners):
        return None
    side = np.min(np.linalg.norm(proposed-np.roll(proposed, 1, axis=0), axis=1))
    if np.max(np.linalg.norm(corners-proposed, axis=1)) > min(14, side*.2):
        return None
    return corners


@lru_cache(maxsize=1800)
def code_bits(tag_id, border_bits):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    modules = 6+2*border_bits
    image = cv2.aruco.generateImageMarker(dictionary, tag_id, modules*10, borderBits=border_bits)
    return image.reshape(modules, 10, modules, 10).transpose(0, 2, 1, 3)[:, :, 5, 5] > 127


def percentiles(values, *qs):
    """np.percentile(..., method="linear") without its per-call overhead."""
    ordered = np.sort(values, axis=None)
    result = []
    for q in qs:
        position = q/100*(ordered.size-1)
        low = int(position)
        high = min(low+1, ordered.size-1)
        result.append(float(ordered[low] + (ordered[high]-ordered[low])*(position-low)))
    return result


def read_bits(gray, corners, border_bits):
    """Module grid (True = white) of a quad, or None when it has no contrast."""
    modules = 6+2*border_bits
    size = modules*10
    destination = np.float32([[0, 0], [size, 0], [size, size], [0, size]])
    transform = cv2.getPerspectiveTransform(corners, destination)
    patch = cv2.warpPerspective(gray, transform, (size, size))
    # Sample only the centre of each module, away from anti-aliased boundaries.
    cells = patch.reshape(modules, 10, modules, 10).transpose(0, 2, 1, 3)[:, :, 3:7, 3:7].mean(axis=(2, 3))
    black, white = percentiles(cells, 20, 90)
    if white-black < 35:
        return None
    return cells > (white+black)/2


def border_errors(bits, border_bits):
    b = border_bits
    border_mask = np.ones_like(bits, dtype=bool)
    border_mask[b:-b, b:-b] = False
    return np.count_nonzero(bits[border_mask])


def matches_code(gray, corners, tag_id, border_bits, max_bit_errors=1, max_border_errors=2):
    bits = read_bits(gray, corners, border_bits)
    if bits is None:
        return False
    b = border_bits
    expected = code_bits(tag_id, border_bits)
    return (border_errors(bits, border_bits) <= max_border_errors
            and np.count_nonzero(bits[b:-b, b:-b] != expected[b:-b, b:-b]) <= max_bit_errors)


def find_quads(gray, min_side=10, max_side=400):
    """Dark convex quads, including grid tags whose corners touch neighbours.

At a distance the black border of an AprilGrid tag blurs into the corner
squares, so a contour detector sees one connected blob. A small erosion of the
dark mask cuts those diagonal links; edges are re-measured later anyway.
"""
    quads = []
    for block in (21, 41):
        dark = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, 7)
        for erosion in (1, 2):
            mask = cv2.erode(dark, np.ones((3, 3), np.uint8), iterations=erosion)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if len(contour) < 4 or cv2.contourArea(contour) < min_side*min_side*.5:
                    continue
                quad = cv2.approxPolyDP(contour, .06*cv2.arcLength(contour, True), True)
                if len(quad) != 4 or not cv2.isContourConvex(quad):
                    continue
                quad = quad[:, 0].astype(np.float32)
                sides = np.linalg.norm(quad-np.roll(quad, 1, axis=0), axis=1)
                if sides.min() < min_side or sides.max() > max_side or sides.max() > 4*sides.min():
                    continue
                # Undo the erosion: push corners outwards from the centre.
                centre = quad.mean(axis=0)
                direction = quad-centre
                quad = quad + direction/np.linalg.norm(direction, axis=1, keepdims=True)*erosion*1.41
                # Clockwise in image coordinates, like the ArUco corner order.
                if cv2.contourArea(quad, oriented=True) < 0:
                    quad = quad[::-1]
                quads.append(quad.astype(np.float32))
    return quads


def identify_quads(gray, board, quads, limit=400):
    """Seed tags from quads the generic decoder rejected.

Edges are measured in the image first; the code is then compared with every
tag of this board in all four orientations. Only <=1 wrong bit is accepted:
tag36h11 codes differ in >=11 bits, so a chance match is negligible.
"""
    smooth = cv2.GaussianBlur(gray.astype(np.float32), (3, 3), .6)
    b = board.border_bits
    ids = list(range(board.first_id, board.first_id+board.columns*board.rows))
    codes = np.stack([code_bits(i, b)[b:-b, b:-b] for i in ids])
    found = {}
    for quad in quads[:limit]:
        refined = refine_quad(smooth, quad, small=True)
        if refined is None:
            continue
        for roll in range(4):
            corners = np.roll(refined, -roll, axis=0)
            bits = read_bits(gray, corners, b)
            if bits is None or border_errors(bits, b) > 2:
                break
            errors = np.count_nonzero(codes != bits[None, b:-b, b:-b], axis=(1, 2))
            best = int(np.argmin(errors))
            if errors[best] <= 1:
                found.setdefault(ids[best], corners)
                break
    return found


def count_code_matches(gray, homography, board, tag_ids, max_bit_errors, max_border_errors):
    """Vectorized matches_code over a board hypothesis: one warp instead of one per tag."""
    modules = 6+2*board.border_bits
    size = modules*10
    pixels_per_mm = size/board.tag_size_mm
    bw, bh = board.size
    width, height = int(np.ceil(bw*pixels_per_mm))+1, int(np.ceil(bh*pixels_per_mm))+1
    canvas_to_board = np.diag([1/pixels_per_mm, 1/pixels_per_mm, 1.])
    canvas = cv2.warpPerspective(gray, homography @ canvas_to_board, (width, height),
                                 flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    patches, expected = [], []
    for tag_id in tag_ids:
        x, y = np.rint(board.points(tag_id)[:, :2].min(axis=0)*pixels_per_mm).astype(int)
        # One extra row/column so that a rotated pixel j samples size-j exactly.
        patch = canvas[y:y+size+1, x:x+size+1]
        if patch.shape != (size+1, size+1):
            continue
        # board.points() rolls the corners by the board rotation; rot90 matches that roll.
        patches.append(np.rot90(patch, board.rotation//90)[:size, :size])
        expected.append(code_bits(tag_id, board.border_bits))
    if not patches:
        return 0
    patches = np.stack(patches).astype(np.float32)
    cells = patches.reshape(-1, modules, 10, modules, 10).transpose(0, 1, 3, 2, 4)[..., 3:7, 3:7].mean(axis=(3, 4))
    ordered = np.sort(cells.reshape(len(cells), -1), axis=1)
    def percentile(q):
        position = q/100*(ordered.shape[1]-1)
        low = int(position)
        return ordered[:, low] + (ordered[:, low+1]-ordered[:, low])*(position-low)
    black, white = percentile(20), percentile(90)
    bits = cells > ((white+black)/2)[:, None, None]
    b = board.border_bits
    border_mask = np.ones((modules, modules), dtype=bool)
    border_mask[b:-b, b:-b] = False
    expected = np.stack(expected)
    border_errors = np.count_nonzero(bits[:, border_mask], axis=1)
    bit_errors = np.count_nonzero(bits[:, b:-b, b:-b] != expected[:, b:-b, b:-b], axis=(1, 2))
    return int(np.count_nonzero((white-black >= 35) & (border_errors <= max_border_errors)
                                & (bit_errors <= max_bit_errors)))


def extend_from_tags(gray, board, found, max_bit_errors=1, max_border_errors=2):
    """Grow a partial detection over the known board layout.

Predict missing tags from the homography of already verified ones, then measure
their edges in the image (refine_quad) and verify the expected code at that
place with the usual <=1 wrong bit. Corners are never taken from the prediction.
This recovers distant tags whose blurred codes the generic decoder misses.
"""
    smooth = cv2.GaussianBlur(gray.astype(np.float32), (3, 3), .6)
    h, w = gray.shape
    found = dict(found)
    added = {}
    while found:
        ids = sorted(found)
        homography, _ = cv2.findHomography(np.concatenate([board.points(i)[:, :2] for i in ids]),
                                           np.concatenate([found[i] for i in ids]), 0)
        if homography is None:
            break
        grown = False
        for tag_id in range(board.first_id, board.first_id+board.columns*board.rows):
            if tag_id in found:
                continue
            proposed = cv2.perspectiveTransform(board.points(tag_id)[None, :, :2], homography)[0]
            if np.any(proposed < 4) or np.any(proposed[:, 0] >= w-4) or np.any(proposed[:, 1] >= h-4):
                continue
            corners = refine_quad(smooth, proposed, small=True)
            if corners is not None and matches_code(gray, corners, tag_id, board.border_bits,
                                                    max_bit_errors, max_border_errors):
                found[tag_id] = added[tag_id] = corners
                grown = True
        if not grown:
            break
    return added


def recover_grid(gray, board, anchors):
    """Propose missing quads from this board's connected black grid outline.

Only use proposals to search for independently measured image edges. Every
returned marker must also pass the known code check; projected corners are never
used as calibration measurements. This handles distant grids whose touching
corner squares prevent a generic per-marker contour detector from finding tags.
"""
    if (board.columns, board.rows, board.border_bits) != (11, 8, 2):
        return {}
    h, w = gray.shape
    offset = np.zeros(2, dtype=np.float32)
    roi = gray
    if anchors:
        points = np.concatenate(anchors)
        x, y, rw, rh = cv2.boundingRect(points)
        edge = max(np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1).mean() for p in anchors)
        margin = int(edge*np.hypot(board.columns, board.rows)*1.5)
        x0, y0 = max(0, x-margin), max(0, y-margin)
        x1, y1 = min(w, x+rw+margin), min(h, y+rh+margin)
        roi = gray[y0:y1, x0:x1]
        offset[:] = [x0, y0]
    if roi.size == 0:
        return {}
    binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)[1]
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bw, bh = board.size
    # The outer checker squares on the supplied calib.io grid have the width
    # of the gap between tags, not the width of one code module.
    pad = board.gap_mm
    object_quad = np.float32([[-pad, -pad], [bw+pad, -pad], [bw+pad, bh+pad], [-pad, bh+pad]])
    smooth = cv2.GaussianBlur(gray.astype(np.float32), (3, 3), .6)
    test_ids = range(board.first_id, board.first_id+board.columns*board.rows)
    proposals = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:10]:
        hull = cv2.convexHull(contour)
        area = cv2.contourArea(hull)
        if area < 2000 or area > .6*gray.size:
            continue
        quad = cv2.approxPolyDP(hull, .025*cv2.arcLength(hull, True), True)
        if len(quad) != 4:
            continue
        quad = quad[:, 0].astype(np.float32)+offset
        centre = quad.mean(axis=0)
        quad = quad[np.argsort(np.arctan2(quad[:, 1]-centre[1], quad[:, 0]-centre[0]))]
        for rotation in range(4):
            homography = cv2.getPerspectiveTransform(object_quad, np.roll(quad, rotation, axis=0))
            # A loose template score only selects a search hypothesis.
            # Returned measurements below still require <=1 wrong code bit.
            count = count_code_matches(gray, homography, board, test_ids, max_bit_errors=8, max_border_errors=12)
            if count >= 3:
                proposals.append((count, homography))
    if not proposals:
        return {}
    homography = max(proposals, key=lambda value: value[0])[1]
    found = {}
    for tag_id in range(board.first_id, board.first_id+board.columns*board.rows):
        proposed = cv2.perspectiveTransform(board.points(tag_id)[None, :, :2], homography)[0]
        if (np.any(proposed < 4) or np.any(proposed[:, 0] >= w-4)
                or np.any(proposed[:, 1] >= h-4)):
            continue
        corners = refine_quad(smooth, proposed, small=True)
        if corners is not None and matches_code(gray, corners, tag_id, board.border_bits):
            found[tag_id] = corners
    return found
