import cv2
import numpy as np
import math
from scipy.optimize import linear_sum_assignment

class CircleDetector:
    def __init__(self):
        self.tracked_circles = {}
        self.next_circle_id = 1
        self.max_lost_distance = 50
        self.CALIBRATION_SCALE_X = 0.25  # mm per pixel in X
        self.CALIBRATION_SCALE_Y = 0.25  # mm per pixel in Y
        self.CALIBRATION_OFFSET_X = -50  # mm offset in X
        self.CALIBRATION_OFFSET_Y = -30  # mm offset in Y

    def assign_ids_to_circles(self, new_circles):
        if not self.tracked_circles:
            results = []
            for nc in new_circles:
                cid = self.next_circle_id
                self.next_circle_id += 1
                self.tracked_circles[cid] = nc.copy()
                results.append({"id": cid, **nc})
            return results

        old_ids = list(self.tracked_circles.keys())
        old_pts = [self.tracked_circles[cid] for cid in old_ids]
        new_pts = new_circles

        cost = np.zeros((len(old_pts), len(new_pts)), dtype=float)
        for i, op in enumerate(old_pts):
            for j, np_ in enumerate(new_pts):
                cost[i, j] = math.hypot(op["x_img"] - np_["x_img"], op["y_img"] - np_["y_img"])

        row_idx, col_idx = linear_sum_assignment(cost)
        assigned = {}
        results = []

        for i, j in zip(row_idx, col_idx):
            if cost[i, j] < self.max_lost_distance:
                cid = old_ids[i]
                nc = new_pts[j]
                self.tracked_circles[cid] = nc.copy()
                assigned[j] = cid
                results.append({"id": cid, **nc})

        for j, nc in enumerate(new_pts):
            if j not in assigned:
                cid = self.next_circle_id
                self.next_circle_id += 1
                self.tracked_circles[cid] = nc.copy()
                results.append({"id": cid, **nc})

        kept_ids = {obj["id"] for obj in results}
        self.tracked_circles = {cid: self.tracked_circles[cid] for cid in kept_ids}
        return results

    def detect_circles(self, frame):
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

        curr = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                if 0 <= y < gray.shape[0] and 0 <= x < gray.shape[1]:
                    if gray[y, x] < 60:
                        # Convert image coordinates to robot coordinates
                        x_robot = x * self.CALIBRATION_SCALE_X + self.CALIBRATION_OFFSET_X
                        y_robot = y * self.CALIBRATION_SCALE_Y + self.CALIBRATION_OFFSET_Y
                        curr.append({
                            "x_img": float(x),
                            "y_img": float(y),
                            "r_img": float(r),
                            "x_robot": x_robot,
                            "y_robot": y_robot
                        })

        return self.assign_ids_to_circles(curr)

# Esempio di utilizzo
if __name__ == "__main__":
    detector = CircleDetector()
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        detections = detector.detect_circles(frame)
        
        # Visualizza risultati
        for det in detections:
            x, y, r = int(det["x_img"]), int(det["y_img"]), int(det["r_img"])
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{det['id']}", (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        cv2.imshow("Circle Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()