# pi_detect.py
import cv2
import time
import math
import numpy as np
from collections import OrderedDict
from ultralytics import YOLO

# ==========================================
#  參數設定
# ==========================================
MODEL_PATH = "best.onnx"   # 請確認檔案存在
CONF_THRESHOLD = 0.5
TARGET_FPS = 10            # 目標偵測幀率
FRAME_WIDTH = 320          # 解析度
FRAME_HEIGHT = 240

# 追蹤與控制參數
AIM_ZONE_RATIO = 0.6       # 瞄準鎖定範圍：目標框寬度的中間 60%
SLOW_ZONE_RATIO = 3.0      # 慢速微調範圍：目標框寬度的中間 300%
SPEED_NORMAL = 1000        # 一般速度
SPEED_SLOW = 1300          # 慢速/微調速度

# 目標類別 ID
TARGET_CLASS_ID = 0  # hole

# ==========================================
#  簡易質心追蹤器 (Simple Centroid Tracker)
# ==========================================
class SimpleTracker:
    def __init__(self, max_disappeared=10, max_distance=80):
        self.nextObjectID = 0
        self.objects = OrderedDict() # 儲存 ID -> (centroid, box)
        self.disappeared = OrderedDict()
        self.maxDisappeared = max_disappeared # 物體消失幾幀後才刪除 ID
        self.maxDistance = max_distance       # 超過此距離視為不同物體

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

            # 找出最小距離的索引
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

            # 處理未匹配的舊物件 (增加消失計數)
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            for row in unusedRows:
                objectID = objectIDs[row]
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            # 處理未匹配的新輸入 (註冊新物件)
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
    
    # 鎖定的目標 ID (None 代表未選擇)
    locked_target_id = None

    print("[Detect] Started. '+/-' to switch ID, 'q' to quit.")

    while True:
        success, frame = cap.read()
        if not success: break

        # --- AI 推論 ---
        results = model.predict(frame, conf=CONF_THRESHOLD, verbose=False)
        annotated_frame = frame.copy()

        # 1. 整理偵測結果 [(cx, cy, x1, y1, x2, y2), ...]
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id != TARGET_CLASS_ID: continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            detections.append((cx, cy, x1, y1, x2, y2))

        # 2. 更新追蹤器，取得帶有 ID 的物件
        # objects 格式: { ID: ((cx, cy), (cx, cy, x1, y1, x2, y2)), ... }
        tracked_objects = tracker.update(detections)

        # 3. 將追蹤物件轉為列表並依 X 座標排序 (為了 +/- 切換的順序符合直覺)
        # 列表內容: {'id': ID, 'cx': cx, 'box': ...}
        sorted_targets = []
        for obj_id, (centroid, box_data) in tracked_objects.items():
            sorted_targets.append({
                'id': obj_id,
                'cx': centroid[0],
                'cy': centroid[1],
                'box': box_data # (cx, cy, x1, y1, x2, y2)
            })
        
        # 依照畫面 X 座標由左至右排序
        sorted_targets.sort(key=lambda k: k['cx'])

        # 4. 檢查目前鎖定的 ID 是否還在畫面中
        target_info = None
        
        # 如果還沒選目標，但畫面有東西，預設選第一個(最左邊)
        if locked_target_id is None and len(sorted_targets) > 0:
            locked_target_id = sorted_targets[0]['id']

        # 嘗試在當前畫面中找到鎖定的 ID
        for t in sorted_targets:
            if t['id'] == locked_target_id:
                target_info = t
                break
        
        # 如果鎖定的 ID 消失了 (不再 sorted_targets 裡)，target_info 會是 None -> 觸發 STOP

        # 繪製畫面中心線
        cv2.line(annotated_frame, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 1)

        # 5. 繪製所有目標與追蹤框
        for t in sorted_targets:
            tid = t['id']
            _, _, x1, y1, x2, y2 = t['box']
            
            is_selected = (tid == locked_target_id)
            color = (0, 255, 0) if is_selected else (0, 0, 255) # 綠色=選中, 紅色=未選
            thickness = 2 if is_selected else 1

            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)
            # 顯示 ID (這是穩定的 ID)
            cv2.putText(annotated_frame, f"ID:{tid}", (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

        action = "STOP"
        current_speed = SPEED_NORMAL

        # 6. 執行瞄準邏輯 (只針對 locked_target_id)
        if target_info is not None:
            cx, _, x1, y1, x2, y2 = target_info['box']
            obj_cy = target_info['cy']
            box_width = x2 - x1
            
            # 計算區域
            zone_width = box_width * AIM_ZONE_RATIO
            zone_left = cx - zone_width / 2
            zone_right = cx + zone_width / 2
            
            slow_width = box_width * SLOW_ZONE_RATIO
            slow_left = cx - slow_width / 2
            slow_right = cx + slow_width / 2

            # 畫輔助線
            cv2.line(annotated_frame, (int(zone_left), 0), (int(zone_left), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.line(annotated_frame, (int(zone_right), 0), (int(zone_right), FRAME_HEIGHT), (0, 255, 255), 1)
            cv2.circle(annotated_frame, (cx, obj_cy), 5, (0, 0, 255), -1)

            # 判斷方向
            if zone_left <= center_x <= zone_right:
                action = "STOP"
                cv2.putText(annotated_frame, f"LOCKED ID:{locked_target_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
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
            # 目標丟失或尚未選擇
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
        
        # 切換 ID 邏輯 (在已排序的列表中循環)
        elif (key == ord('+') or key == ord('=')) and len(sorted_targets) > 0:
            # 找出目前 ID 在排序列表中的索引
            current_idx = -1
            for i, t in enumerate(sorted_targets):
                if t['id'] == locked_target_id:
                    current_idx = i
                    break
            
            # 切換到下一個
            new_idx = (current_idx + 1) % len(sorted_targets)
            locked_target_id = sorted_targets[new_idx]['id']
            print(f"[UI] Switched to ID: {locked_target_id}")

        elif (key == ord('-') or key == ord('_')) and len(sorted_targets) > 0:
            current_idx = -1
            for i, t in enumerate(sorted_targets):
                if t['id'] == locked_target_id:
                    current_idx = i
                    break
            
            # 切換到上一個
            new_idx = (current_idx - 1 + len(sorted_targets)) % len(sorted_targets)
            locked_target_id = sorted_targets[new_idx]['id']
            print(f"[UI] Switched to ID: {locked_target_id}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(None)