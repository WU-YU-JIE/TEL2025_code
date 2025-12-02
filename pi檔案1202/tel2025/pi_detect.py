# pi_detect_grid_v10.py
import cv2
import time
import math
import numpy as np
from collections import OrderedDict
from ultralytics import YOLO

# ==========================================
#  參數設定
# ==========================================
MODEL_PATH = "best.onnx"
CONF_THRESHOLD = 0.5
TARGET_FPS = 10
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

AIM_ZONE_RATIO = 0.6
SLOW_ZONE_RATIO = 3.0
SPEED_NORMAL = 1000
SPEED_SLOW = 1300

TARGET_CLASS_ID = 0 
ROW_LABELS = ["A", "B", "C", "D"]
MAX_COLS = 3

OUTLIER_DIST_RATIO = 0.45
MAX_ROTATION_DEGREE = 30

KEY_MAPPING = {
    ord('7'): 'B1', ord('8'): 'B2', ord('9'): 'B3',
    ord('4'): 'C1', ord('5'): 'C2', ord('6'): 'C3',
    ord('1'): 'D1', ord('2'): 'D2', ord('3'): 'D3',
    ord('0'): 'A2' 
}

# ==========================================
#  數學工具函式
# ==========================================
def calculate_global_tilt(points):
    if len(points) < 3: return 0.0
    pts_array = np.array(points, dtype=np.float32)
    rect = cv2.minAreaRect(pts_array)
    angle = rect[-1]
    width, height = rect[1]
    if width < height: angle -= 90
    while angle > 45: angle -= 90
    while angle < -45: angle += 90
    return angle

def rotate_point(cx, cy, angle_deg, center_x, center_y):
    angle_rad = math.radians(-angle_deg) 
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    tx, ty = cx - center_x, cy - center_y
    rx = tx * cos_a - ty * sin_a
    ry = tx * sin_a + ty * cos_a
    return rx + center_x, ry + center_y

def inverse_rotate_point(rx, ry, angle_deg, center_x, center_y):
    return rotate_point(rx, ry, -angle_deg, center_x, center_y)

def draw_grid_lines(img, grid_origin_x, grid_origin_y, grid_dx, grid_dy, tilt_angle, scx, scy):
    col_boundaries = [-0.5, 0.5, 1.5, 2.5]
    row_boundaries = [-0.5, 0.5, 1.5, 2.5, 3.5]
    for i in col_boundaries:
        virt_x = grid_origin_x + i * grid_dx
        p1_rot = (virt_x, -1000)
        p2_rot = (virt_x, 1000)
        p1_raw = inverse_rotate_point(p1_rot[0], p1_rot[1], tilt_angle, scx, scy)
        p2_raw = inverse_rotate_point(p2_rot[0], p2_rot[1], tilt_angle, scx, scy)
        cv2.line(img, (int(p1_raw[0]), int(p1_raw[1])), (int(p2_raw[0]), int(p2_raw[1])), (0, 255, 255), 1)
    for i in row_boundaries:
        virt_y = grid_origin_y + i * grid_dy
        p1_rot = (-1000, virt_y)
        p2_rot = (1000, virt_y)
        p1_raw = inverse_rotate_point(p1_rot[0], p1_rot[1], tilt_angle, scx, scy)
        p2_raw = inverse_rotate_point(p2_rot[0], p2_rot[1], tilt_angle, scx, scy)
        cv2.line(img, (int(p1_raw[0]), int(p1_raw[1])), (int(p2_raw[0]), int(p2_raw[1])), (0, 255, 255), 1)

# ==========================================
#  簡易質心追蹤器
# ==========================================
class SimpleTracker:
    def __init__(self, max_disappeared=10, max_distance=80):
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.maxDisappeared = max_disappeared
        self.maxDistance = max_distance

    def register(self, centroid, box):
        self.objects[self.nextObjectID] = (centroid, box)
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects):
        if len(rects) == 0:
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        for (i, (cx, cy, _, _, _, _)) in enumerate(rects):
            inputCentroids[i] = (cx, cy)

        if len(self.objects) == 0:
            for i in range(0, len(rects)):
                self.register(inputCentroids[i], rects[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = [self.objects[id][0] for id in objectIDs]
            D = []
            for oc in objectCentroids:
                row = []
                for ic in inputCentroids:
                    dist = math.sqrt((oc[0]-ic[0])**2 + (oc[1]-ic[1])**2)
                    row.append(dist)
                D.append(row)
            D = np.array(D)
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            usedRows = set()
            usedCols = set()

            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols: continue
                if D[row][col] > self.maxDistance: continue
                objectID = objectIDs[row]
                self.objects[objectID] = (inputCentroids[col], rects[col])
                self.disappeared[objectID] = 0
                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            for row in unusedRows:
                objectID = objectIDs[row]
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            for col in unusedCols:
                self.register(inputCentroids[col], rects[col])

        return self.objects

# ==========================================
#  主程式
# ==========================================
def main(ipc_queue=None):
    def send_aim_cmd(cmd_str):
        if ipc_queue:
            ipc_queue.put(f"AIM_RAW {cmd_str}")

    print(f"[Detect] Loading model: {MODEL_PATH} ...")
    try:
        model = YOLO(MODEL_PATH, task='detect')
    except Exception as e:
        print(f"[Detect] Model load failed: {e}")
        return

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    if not cap.isOpened():
        print("[Detect] Camera failed to open.")
        return

    center_x = FRAME_WIDTH // 2
    last_action = "IDLE"
    
    tracker = SimpleTracker(max_disappeared=5, max_distance=100)
    locked_target_id = None
    pending_search_label = None 
    
    prev_frame_time = 0
    new_frame_time = 0

    print("[Detect] Started. Press '0-9' to lock specific grid, 'q' to quit.")

    while True:
        success, frame = cap.read()
        if not success: break

        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time != 0 else 0
        prev_frame_time = new_frame_time

        results = model.predict(frame, conf=CONF_THRESHOLD, verbose=False)
        annotated_frame = frame.copy()

        cv2.putText(annotated_frame, f"FPS: {int(fps)}", (FRAME_WIDTH - 80, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        raw_detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id != TARGET_CLASS_ID: continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            raw_detections.append({'cx': cx, 'cy': cy, 'box': (cx, cy, x1, y1, x2, y2)})

        tracker_input = [d['box'] for d in raw_detections]
        tracked_objects = tracker.update(tracker_input)

        sorted_targets = []
        points_for_rotation = [] 

        for obj_id, (centroid, box_data) in tracked_objects.items():
            cx, cy, x1, y1, x2, y2 = box_data
            points_for_rotation.append((cx, cy))
            sorted_targets.append({
                'id': obj_id,
                'raw_cx': cx,
                'raw_cy': cy,
                'rot_cx': cx,
                'rot_cy': cy,
                'w': x2 - x1,
                'h': y2 - y1,
                'box': box_data,
                'grid_label': "?",
                'valid_grid': False,
                'row_idx': -1,
                'col_idx': -1
            })
        
        if len(sorted_targets) > 0:
            tilt_angle = calculate_global_tilt(points_for_rotation)
            screen_cx, screen_cy = FRAME_WIDTH / 2, FRAME_HEIGHT / 2
            
            rot_cxs = []
            rot_cys = []
            ws = []
            hs = []
            for t in sorted_targets:
                if abs(tilt_angle) > 2.0:
                    rcx, rcy = rotate_point(t['raw_cx'], t['raw_cy'], tilt_angle, screen_cx, screen_cy)
                    t['rot_cx'] = rcx
                    t['rot_cy'] = rcy
                else:
                    t['rot_cx'] = t['raw_cx']
                    t['rot_cy'] = t['raw_cy']
                rot_cxs.append(t['rot_cx'])
                rot_cys.append(t['rot_cy'])
                ws.append(t['w'])
                hs.append(t['h'])

            median_w = np.median(ws)
            median_h = np.median(hs)

            sorted_rot_x = np.sort(rot_cxs)
            diffs_x = np.diff(sorted_rot_x)
            valid_dx = diffs_x[(diffs_x > median_w * 0.8) & (diffs_x < median_w * 4.0)]
            grid_dx = np.median(valid_dx) if len(valid_dx) > 0 else median_w * 1.5

            sorted_rot_y = np.sort(rot_cys)
            diffs_y = np.diff(sorted_rot_y)
            valid_dy = diffs_y[(diffs_y > median_h * 0.8) & (diffs_y < median_h * 4.0)]
            grid_dy = np.median(valid_dy) if len(valid_dy) > 0 else median_h * 1.5

            min_x = np.min(rot_cxs)
            max_x = np.max(rot_cxs)
            min_y = np.min(rot_cys)
            
            span_ratio = (max_x - min_x) / grid_dx
            col_offset = 0

            if 0.5 < span_ratio < 1.8:
                cluster_center_x = (min_x + max_x) / 2
                if cluster_center_x < screen_cx:
                    col_offset = 1 
                else:
                    col_offset = 0 

            for t in sorted_targets:
                col_idx = int(round((t['rot_cx'] - min_x) / grid_dx)) + col_offset
                row_idx = int(round((t['rot_cy'] - min_y) / grid_dy))

                t['col_idx'] = col_idx
                t['row_idx'] = row_idx

                if 0 <= row_idx < len(ROW_LABELS) and 0 <= col_idx < MAX_COLS:
                    t['grid_label'] = f"{ROW_LABELS[row_idx]}{col_idx + 1}"
                    t['valid_grid'] = True
                else:
                    t['grid_label'] = "?" 
                    t['valid_grid'] = False

            d1 = next((t for t in sorted_targets if t['valid_grid'] and t['grid_label']=="D1"), None)
            d2 = next((t for t in sorted_targets if t['valid_grid'] and t['grid_label']=="D2"), None)
            c3 = next((t for t in sorted_targets if t['valid_grid'] and t['grid_label']=="C3"), None)

            if d1 and d2 and c3:
                pred_x = d2['rot_cx'] + (d2['rot_cx'] - d1['rot_cx'])
                pred_y = d2['rot_cy'] + (d2['rot_cy'] - d1['rot_cy'])
                dist = math.sqrt((c3['rot_cx'] - pred_x)**2 + (c3['rot_cy'] - pred_y)**2)
                if dist < (grid_dy * 0.5):
                    c3['grid_label'] = "D3"
                    c3['row_idx'] = 3
                    c3['col_idx'] = 2

            valid_targets_for_calc = [t for t in sorted_targets if t['valid_grid']]
            if len(valid_targets_for_calc) > 0:
                origins_x = [t['rot_cx'] - (t['col_idx'] * grid_dx) for t in valid_targets_for_calc]
                origins_y = [t['rot_cy'] - (t['row_idx'] * grid_dy) for t in valid_targets_for_calc]
                grid_origin_x = np.median(origins_x)
                grid_origin_y = np.median(origins_y)
                draw_grid_lines(annotated_frame, grid_origin_x, grid_origin_y, grid_dx, grid_dy, tilt_angle, screen_cx, screen_cy)

        valid_targets = [t for t in sorted_targets if t['valid_grid']]
        valid_targets.sort(key=lambda k: k['raw_cx']) 

        target_info = None
        
        if pending_search_label:
            found = False
            for t in valid_targets:
                if t['grid_label'] == pending_search_label:
                    locked_target_id = t['id'] 
                    pending_search_label = None 
                    found = True
                    break
            
            if not found:
                try:
                    wanted_col = int(pending_search_label[1]) 
                    visible_cols = [t['col_idx'] + 1 for t in valid_targets] 
                    
                    if len(visible_cols) > 0:
                        min_vis = min(visible_cols)
                        max_vis = max(visible_cols)
                        
                        search_action = "STOP"
                        if wanted_col < min_vis:
                            search_action = "LEFT" 
                        elif wanted_col > max_vis:
                            search_action = "RIGHT" 
                        
                        cv2.putText(annotated_frame, f"SEARCHING {pending_search_label} >> {search_action}", 
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                        
                        if search_action != "STOP":
                            send_aim_cmd(f"{search_action.lower()},{SPEED_NORMAL}")
                            last_action = search_action
                            cv2.imshow("Auto Tracking", annotated_frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'): break
                            if key in KEY_MAPPING:
                                req = KEY_MAPPING[key]
                                pending_search_label = req
                                locked_target_id = None
                            continue 
                except:
                    pass

        if locked_target_id is not None:
            for t in valid_targets:
                if t['id'] == locked_target_id:
                    target_info = t
                    break
        
        cv2.line(annotated_frame, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 1)

        for t in sorted_targets:
            if not t['valid_grid']: continue 
            grid_lbl = t['grid_label']
            _, _, x1, y1, x2, y2 = t['box']
            is_selected = (t['id'] == locked_target_id)
            color = (0, 255, 0) if is_selected else (0, 0, 255)
            thickness = 2 if is_selected else 1
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)
            label_text = f"{grid_lbl}"
            (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated_frame, (x1, y1 - 25), (x1 + tw, y1), color, -1)
            cv2.putText(annotated_frame, label_text, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        action = "STOP"
        current_speed = SPEED_NORMAL

        if target_info is not None:
            cx, _, x1, y1, x2, y2 = target_info['box']
            obj_cy = target_info['raw_cy'] 
            box_width = x2 - x1
            
            # --- 新增：計算並顯示高度 ---
            box_height = y2 - y1
            cv2.putText(annotated_frame, f"H: {box_height}px", (x2, y2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # --------------------------

            zone_width = box_width * AIM_ZONE_RATIO
            zone_left = cx - zone_width / 2
            zone_right = cx + zone_width / 2
            slow_width = box_width * SLOW_ZONE_RATIO
            slow_left = cx - slow_width / 2
            slow_right = cx + slow_width / 2

            cv2.line(annotated_frame, (int(zone_left), 0), (int(zone_left), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.line(annotated_frame, (int(zone_right), 0), (int(zone_right), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.circle(annotated_frame, (cx, obj_cy), 5, (0, 0, 255), -1)

            if zone_left <= center_x <= zone_right:
                action = "STOP"
                status_msg = f"LOCKED: {target_info['grid_label']}"
                cv2.putText(annotated_frame, status_msg, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                if center_x < zone_left:
                    action = "RIGHT"
                else:
                    action = "LEFT"

                if slow_left <= center_x <= slow_right:
                    current_speed = SPEED_SLOW
                    cv2.putText(annotated_frame, "SLOW", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                else:
                    current_speed = SPEED_NORMAL
        else:
            if not pending_search_label:
                action = "STOP"
                cv2.putText(annotated_frame, "NO TARGET", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

        current_state = f"{action}_{current_speed}"
        
        if action == "STOP":
            if last_action != "STOP":
                send_aim_cmd("stop")
                last_action = "STOP"
        else:
            if current_state != last_action:
                if action == "LEFT": send_aim_cmd(f"left,{current_speed}")
                elif action == "RIGHT": send_aim_cmd(f"right,{current_speed}")
                last_action = current_state

        cv2.imshow("Auto Tracking", annotated_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            send_aim_cmd("stop")
            break
        
        if key in KEY_MAPPING:
            req_label = KEY_MAPPING[key]
            found_id = None
            for t in valid_targets:
                if t['grid_label'] == req_label:
                    found_id = t['id']
                    break
            
            if found_id is not None:
                locked_target_id = found_id
                pending_search_label = None
                print(f"[UI] Direct Lock: {req_label}")
            else:
                locked_target_id = None
                pending_search_label = req_label
                print(f"[UI] Searching for: {req_label} ...")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(None)