# pi_detect_grid.py
import cv2
import time
import math
import numpy as np
from collections import OrderedDict
from ultralytics import YOLO

# ==========================================
#  參數設定
# ==========================================
MODEL_PATH = "best.onnx"   # 請確認模型路徑
CONF_THRESHOLD = 0.5
TARGET_FPS = 10            # 目標幀率
FRAME_WIDTH = 320          # 寬度
FRAME_HEIGHT = 240         # 高度

# 自動瞄準參數
AIM_ZONE_RATIO = 0.6       # 瞄準區域：目標寬度的中心 60%
SLOW_ZONE_RATIO = 3.0      # 減速區域：目標寬度的 300%
SPEED_NORMAL = 1000        # 一般速度
SPEED_SLOW = 1300          # 減速/微調速度

# 目標類別 ID
TARGET_CLASS_ID = 0  # hole

# 網格標籤定義
ROW_LABELS = ["A", "B", "C", "D"]

# ==========================================
#  簡易質心追蹤器 (Simple Centroid Tracker)
# ==========================================
class SimpleTracker:
    def __init__(self, max_disappeared=10, max_distance=80):
        self.nextObjectID = 0
        self.objects = OrderedDict() # 儲存 ID -> (centroid, box)
        self.disappeared = OrderedDict()
        self.maxDisappeared = max_disappeared # 允許目標消失多少幀後刪除 ID
        self.maxDistance = max_distance       # 超過此距離則視為不同物件

    def register(self, centroid, box):
        self.objects[self.nextObjectID] = (centroid, box)
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects):
        # rects 格式: [(cx, cy, x1, y1, x2, y2), ...]
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
            objectCentroids = [self.objects[id][0] for id in objectIDs] # 取出 (cx, cy)
            
            # 計算距離矩陣
            D = [] 
            for oc in objectCentroids:
                row = []
                for ic in inputCentroids:
                    dist = math.sqrt((oc[0]-ic[0])**2 + (oc[1]-ic[1])**2)
                    row.append(dist)
                D.append(row)
            D = np.array(D)

            # 找出最小距離的關聯
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

            # 處理未配對的舊 ID (增加消失計數)
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            for row in unusedRows:
                objectID = objectIDs[row]
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            # 處理未配對的新輸入 (註冊新 ID)
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
    
    # 初始化追蹤器
    tracker = SimpleTracker(max_disappeared=5, max_distance=100)
    
    # 鎖定的目標 ID (None 代表未鎖定)
    locked_target_id = None

    print("[Detect] Started. '+/-' to switch ID, 'q' to quit.")

    while True:
        success, frame = cap.read()
        if not success: break

        # --- AI 推論 ---
        results = model.predict(frame, conf=CONF_THRESHOLD, verbose=False)
        annotated_frame = frame.copy()

        # 1. 讀取偵測結果 [(cx, cy, x1, y1, x2, y2), ...]
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id != TARGET_CLASS_ID: continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            detections.append((cx, cy, x1, y1, x2, y2))

        # 2. 更新追蹤器，獲取帶有 ID 的物件
        tracked_objects = tracker.update(detections)

        # 3. 將追蹤物件轉為列表並準備數據
        # 資料結構: {'id': ID, 'cx': cx, 'cy': cy, 'w': w, 'h': h, 'box': ..., 'label': ''}
        sorted_targets = []
        for obj_id, (centroid, box_data) in tracked_objects.items():
            cx, cy, x1, y1, x2, y2 = box_data
            sorted_targets.append({
                'id': obj_id,
                'cx': centroid[0],
                'cy': centroid[1],
                'w': x2 - x1,
                'h': y2 - y1,
                'box': box_data,
                'grid_label': "?" # 預設標籤
            })
        
        # 4. === 智慧網格推算邏輯 (Smart Grid Inference) ===
        if len(sorted_targets) > 0:
            # 提取所有目標的寬高與座標
            ws = [t['w'] for t in sorted_targets]
            hs = [t['h'] for t in sorted_targets]
            cxs = np.array([t['cx'] for t in sorted_targets])
            cys = np.array([t['cy'] for t in sorted_targets])

            # 計算中位數寬高作為參考基準
            median_w = np.median(ws)
            median_h = np.median(hs)

            # A. 估計網格間距 (Pitch)
            # 水平間距 dx
            sorted_cxs = np.sort(cxs)
            diffs_x = np.diff(sorted_cxs)
            # 過濾雜訊：間距需在合理範圍 (例如 0.8倍 ~ 4倍 洞寬)
            valid_dx = diffs_x[(diffs_x > median_w * 0.8) & (diffs_x < median_w * 4.0)]
            grid_dx = np.median(valid_dx) if len(valid_dx) > 0 else median_w * 1.5

            # 垂直間距 dy
            sorted_cys = np.sort(cys)
            diffs_y = np.diff(sorted_cys)
            valid_dy = diffs_y[(diffs_y > median_h * 0.8) & (diffs_y < median_h * 4.0)]
            grid_dy = np.median(valid_dy) if len(valid_dy) > 0 else median_h * 1.5

            # B. 決定原點 (A1 的相對位置)
            # 取畫面中最左上角的洞作為相對 (0,0)
            min_x = np.min(cxs)
            min_y = np.min(cys)

            # C. 為每個目標計算網格座標
            for t in sorted_targets:
                # 計算它是第幾行 (Col) 和第幾列 (Row)
                # 使用 round 四捨五入來處理座標誤差
                col_idx = int(round((t['cx'] - min_x) / grid_dx))
                row_idx = int(round((t['cy'] - min_y) / grid_dy))

                if 0 <= row_idx < len(ROW_LABELS) and col_idx < 9:
                    t['grid_label'] = f"{ROW_LABELS[row_idx]}{col_idx + 1}"
                else:
                    t['grid_label'] = "?" # 超出範圍

        # 5. 排序與選擇邏輯 (依 X 座標排序，方便左右切換)
        sorted_targets.sort(key=lambda k: k['cx'])

        # 檢查鎖定目標是否存在
        target_info = None
        
        # 如果沒有鎖定目標，自動鎖定第一個 (最左邊)
        if locked_target_id is None and len(sorted_targets) > 0:
            locked_target_id = sorted_targets[0]['id']

        # 搜尋鎖定的 ID
        for t in sorted_targets:
            if t['id'] == locked_target_id:
                target_info = t
                break
        
        # 繪製中線
        cv2.line(annotated_frame, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 1)

        # 6. 繪製所有目標與標籤
        for t in sorted_targets:
            tid = t['id']
            grid_lbl = t['grid_label']
            _, _, x1, y1, x2, y2 = t['box']
            
            is_selected = (tid == locked_target_id)
            color = (0, 255, 0) if is_selected else (0, 0, 255) # 綠=鎖定, 紅=未鎖定
            thickness = 2 if is_selected else 1

            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)
            
            # 顯示 ID 和 推算的網格編號 (例如: ID:0 [A1])
            label_text = f"{grid_lbl} (ID:{tid})"
            cv2.putText(annotated_frame, label_text, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        action = "STOP"
        current_speed = SPEED_NORMAL

        # 7. 瞄準邏輯
        if target_info is not None:
            cx, _, x1, y1, x2, y2 = target_info['box']
            obj_cy = target_info['cy']
            box_width = x2 - x1
            
            # 設定區域
            zone_width = box_width * AIM_ZONE_RATIO
            zone_left = cx - zone_width / 2
            zone_right = cx + zone_width / 2
            
            slow_width = box_width * SLOW_ZONE_RATIO
            slow_left = cx - slow_width / 2
            slow_right = cx + slow_width / 2

            # 畫瞄準輔助線
            cv2.line(annotated_frame, (int(zone_left), 0), (int(zone_left), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.line(annotated_frame, (int(zone_right), 0), (int(zone_right), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.circle(annotated_frame, (cx, obj_cy), 5, (0, 0, 255), -1)

            # 判斷動作
            if zone_left <= center_x <= zone_right:
                action = "STOP"
                status_msg = f"LOCKED {target_info['grid_label']}"
                cv2.putText(annotated_frame, status_msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                if center_x < zone_left:
                    action = "RIGHT"
                    cv2.putText(annotated_frame, "RIGHT >>", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                else:
                    action = "LEFT"
                    cv2.putText(annotated_frame, "<< LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                if slow_left <= center_x <= slow_right:
                    current_speed = SPEED_SLOW
                    cv2.putText(annotated_frame, "SLOW", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                else:
                    current_speed = SPEED_NORMAL
        else:
            # 目標遺失
            action = "STOP"
            status_text = f"LOST ID:{locked_target_id}" if locked_target_id is not None else "NO TARGET"
            cv2.putText(annotated_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

        # --- 發送指令 ---
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
        
        # --- 鍵盤控制 ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            send_aim_cmd("stop")
            break
        
        # 切換 ID (在已偵測的目標中循環)
        elif (key == ord('+') or key == ord('=')) and len(sorted_targets) > 0:
            current_idx = -1
            for i, t in enumerate(sorted_targets):
                if t['id'] == locked_target_id:
                    current_idx = i
                    break
            
            new_idx = (current_idx + 1) % len(sorted_targets)
            locked_target_id = sorted_targets[new_idx]['id']
            print(f"[UI] Switched to ID: {locked_target_id} ({sorted_targets[new_idx]['grid_label']})")

        elif (key == ord('-') or key == ord('_')) and len(sorted_targets) > 0:
            current_idx = -1
            for i, t in enumerate(sorted_targets):
                if t['id'] == locked_target_id:
                    current_idx = i
                    break
            
            new_idx = (current_idx - 1 + len(sorted_targets)) % len(sorted_targets)
            locked_target_id = sorted_targets[new_idx]['id']
            print(f"[UI] Switched to ID: {locked_target_id} ({sorted_targets[new_idx]['grid_label']})")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(None)