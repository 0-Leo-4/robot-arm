# modules/vision.py
import cv2
import numpy as np
import time
import math
from scipy.optimize import linear_sum_assignment
from .shared_state import state

def assign_ids_to_circles(new_circles):
    if not state.tracked_circles:
        results = []
        for nc in new_circles:
            cid = state.next_circle_id
            state.next_circle_id += 1
            state.tracked_circles[cid] = nc.copy()
            results.append({"id": cid, **nc})
        return results

    old_ids = list(state.tracked_circles.keys())
    old_pts = [state.tracked_circles[cid] for cid in old_ids]
    new_pts = new_circles

    cost = np.zeros((len(old_pts), len(new_pts)), dtype=float)
    for i, op in enumerate(old_pts):
        for j, np_ in enumerate(new_pts):
            cost[i, j] = math.hypot(op["x"] - np_["x"], op["y"] - np_["y"])

    row_idx, col_idx = linear_sum_assignment(cost)
    assigned = {}
    results = []

    for i, j in zip(row_idx, col_idx):
        if cost[i, j] < state.max_lost_distance:
            cid = old_ids[i]
            nc = new_pts[j]
            state.tracked_circles[cid] = nc.copy()
            assigned[j] = cid
            results.append({"id": cid, **nc})

    for j, nc in enumerate(new_pts):
        if j not in assigned:
            cid = state.next_circle_id
            state.next_circle_id += 1
            state.tracked_circles[cid] = nc.copy()
            results.append({"id": cid, **nc})

    kept_ids = {obj["id"] for obj in results}
    state.tracked_circles = {cid: state.tracked_circles[cid] for cid in kept_ids}
    return results

def capture_and_detect():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        blurred = cv2.GaussianBlur(mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        ret2, jpeg = cv2.imencode('.jpg', frame)
        if ret2:
            state.latest_frame = jpeg.tobytes()

        curr = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                if 0 <= y < gray.shape[0] and 0 <= x < gray.shape[1]:
                    if gray[y, x] < 60:
                        curr.append({"x": float(x), "y": float(y), "r": float(r)})

        tracked = assign_ids_to_circles(curr)
        dt = time.time() - t0
        if dt > 0:
            state.fps = round(1.0 / dt, 1)

        with state.lock:
            state.detections = tracked

        time.sleep(max(0, 0.1 - (time.time() - t0)))