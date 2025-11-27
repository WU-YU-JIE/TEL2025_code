# /TEL2025_code/pi檔案1126/tel2025/pi_detect1127.py  # 檔案路徑說明
# 添加顯示FPS，移除孔洞ID顯示
# Written on 251127

import cv2  # 影像擷取與繪圖
import time  # 計時用於 FPS 與節流
import math  # 距離計算
import numpy as np  # 數值運算
from collections import OrderedDict  # 保序字典用於追蹤表
from ultralytics import YOLO  # 物件偵測模型介面

# ==========================================
#  參數設定
# ==========================================
MODEL_PATH = "best.onnx"   # 模型路徑(ONNX/Ultralytics 皆可由 YOLO 載入)
CONF_THRESHOLD = 0.5  # 偵測信心門檻
TARGET_FPS = 10  # 相機目標 FPS
FRAME_WIDTH = 320  # 影像寬
FRAME_HEIGHT = 240  # 影像高

# 自動瞄準參數
AIM_ZONE_RATIO = 0.6  # 目標框寬的比例定義「停止區」
SLOW_ZONE_RATIO = 3.0  # 目標框寬的比例定義「減速區」
SPEED_NORMAL = 1000  # 一般速度(傳送至外部裝置的參數)
SPEED_SLOW = 1300  # 靠近目標時的減速參數(較大代表較慢)

# 目標類別 ID
TARGET_CLASS_ID = 0  # 僅偵測類別 0 (hole)

# 網格標籤定義
ROW_LABELS = ["A", "B", "C", "D"]  # 四行標籤

# ==========================================
#  數字鍵對應洞的位置對照表
#  依你的圖：
#      0      → A2
#    1 2 3    → B1 B2 B3
#   (拍不到)  → 略
#    4 5 6    → C1 C2 C3
#    7 8 9    → D1 D2 D3
# 若真實對應不同，可以直接改這個 dict
# ==========================================
DIGIT_TO_GRID = {  # 0–9 鍵與網格標籤的對應
    "0": "A2",  # 最上單孔
    "1": "B1", "2": "B2", "3": "B3",  # 第二列
    "4": "C1", "5": "C2", "6": "C3",  # 第三列
    "7": "D1", "8": "D2", "9": "D3",  # 第四列
}

# ==========================================
#  簡易質心追蹤器 (Simple Centroid Tracker)
# ==========================================
class SimpleTracker:
    def __init__(self, max_disappeared=10, max_distance=80):  # 初始化追蹤器參數
        self.nextObjectID = 0  # 下一個可用的物件 ID
        self.objects = OrderedDict()  # 物件 ID -> (centroid, box)
        self.disappeared = OrderedDict()  # 物件 ID -> 消失幀數
        self.maxDisappeared = max_disappeared  # 容忍消失幀數
        self.maxDistance = max_distance  # 允許匹配的最大距離

    def register(self, centroid, box):  # 新增物件
        self.objects[self.nextObjectID] = (centroid, box)  # 記錄位置與框
        self.disappeared[self.nextObjectID] = 0  # 消失計數歸零
        self.nextObjectID += 1  # ID 自增

    def deregister(self, objectID):  # 移除物件
        del self.objects[objectID]  # 刪紀錄
        del self.disappeared[objectID]  # 刪消失計數

    def update(self, rects):  # 以新偵測框更新追蹤狀態
        # rects: [(cx, cy, x1, y1, x2, y2), ...]  # 輸入格式說明
        if len(rects) == 0:  # 若本幀無偵測
            for objectID in list(self.disappeared.keys()):  # 走訪所有已知物件
                self.disappeared[objectID] += 1  # 消失計數+1
                if self.disappeared[objectID] > self.maxDisappeared:  # 超出容忍
                    self.deregister(objectID)  # 取消追蹤
            return self.objects  # 回傳當前追蹤表

        inputCentroids = np.zeros((len(rects), 2), dtype="int")  # 建立輸入質心陣列
        for (i, (cx, cy, _, _, _, _)) in enumerate(rects):  # 逐一取出質心
            inputCentroids[i] = (cx, cy)  # 填入陣列

        if len(self.objects) == 0:  # 初次或暫無物件時
            for i in range(0, len(rects)):  # 全部註冊
                self.register(inputCentroids[i], rects[i])  # 新增追蹤
        else:
            objectIDs = list(self.objects.keys())  # 既有 ID 列表
            objectCentroids = [self.objects[id][0] for id in objectIDs]  # 既有質心列表

            D = []  # 距離矩陣(列:既有  行:輸入)
            for oc in objectCentroids:  # 對每個既有質心
                row = []  # 一列距離
                for ic in inputCentroids:  # 對每個新質心
                    dist = math.sqrt((oc[0]-ic[0])**2 + (oc[1]-ic[1])**2)  # 歐氏距離
                    row.append(dist)  # 累加
                D.append(row)  # 加入距離矩陣
            D = np.array(D)  # 轉為 numpy 陣列

            rows = D.min(axis=1).argsort()  # 既有端依最近距離排序
            cols = D.argmin(axis=1)[rows]  # 為排序後的列找到最佳對應輸入欄位

            usedRows = set()  # 已匹配既有列
            usedCols = set()  # 已匹配輸入欄

            for (row, col) in zip(rows, cols):  # 嘗試配對
                if row in usedRows or col in usedCols:  # 已用則跳過
                    continue
                if D[row][col] > self.maxDistance:  # 距離過遠忽略
                    continue

                objectID = objectIDs[row]  # 取得對應 ID
                self.objects[objectID] = (inputCentroids[col], rects[col])  # 更新質心與框
                self.disappeared[objectID] = 0  # 消失歸零
                usedRows.add(row)  # 記錄已用
                usedCols.add(col)  # 記錄已用

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)  # 未匹配既有列
            for row in unusedRows:  # 逐一處理
                objectID = objectIDs[row]  # 取 ID
                self.disappeared[objectID] += 1  # 消失+1
                if self.disappeared[objectID] > self.maxDisappeared:  # 超限即移除
                    self.deregister(objectID)  # 取消追蹤

            unusedCols = set(range(0, D.shape[1])).difference(usedCols)  # 未匹配輸入欄
            for col in unusedCols:  # 對新出現的物件
                self.register(inputCentroids[col], rects[col])  # 註冊新物件

        return self.objects  # 回傳追蹤表

# ==========================================
#  主程式
# ==========================================
def main(ipc_queue=None):  # 主進入點，可選 IPC 佇列
    def send_aim_cmd(cmd_str):  # 封裝發送命令
        if ipc_queue:  # 僅在 IPC 存在時傳送
            ipc_queue.put(f"AIM_RAW {cmd_str}")  # 送出原始命令

    print(f"[Detect] Loading model: {MODEL_PATH} ...")  # 提示載入模型
    try:
        model = YOLO(MODEL_PATH, task='detect')  # 載入 YOLO 偵測模型
    except Exception as e:  # 錯誤處理
        print(f"[Detect] Model load failed: {e}")  # 輸出錯誤
        return  # 中止

    cap = cv2.VideoCapture(0)  # 開啟預設攝影機(索引 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)  # 設定寬
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)  # 設定高
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)  # 設定期望 FPS(實際依硬體)

    if not cap.isOpened():  # 開啟失敗
        print("[Detect] Camera failed to open.")  # 提示
        return  # 中止

    center_x = FRAME_WIDTH // 2  # 畫面水平中心 X
    last_action = "IDLE"  # 上一組動作狀態字串(避免重複傳送)

    tracker = SimpleTracker(max_disappeared=5, max_distance=100)  # 初始化追蹤器
    locked_target_id = None  # 目前鎖定的目標 ID

    # FPS 計算用變數
    prev_time = time.time()  # 上一幀時間戳
    fps = 0.0  # 當前 FPS 值

    print("[Detect] Started. '+/-' to switch ID, '0~9' to select hole, 'q' to quit.")  # 使用提示

    while True:  # 主迴圈
        # --- FPS 計算 ---
        now = time.time()  # 取得當前時間
        dt = now - prev_time  # 與上一幀間隔
        if dt > 0:  # 避免除以零
            fps = 1.0 / dt  # 計算 FPS
        prev_time = now  # 更新上一幀時間

        success, frame = cap.read()  # 擷取一幀
        if not success:  # 讀取失敗
            break  # 跳出

        # --- AI 推論 ---
        results = model.predict(frame, conf=CONF_THRESHOLD, verbose=False)  # 以門檻推論
        annotated_frame = frame.copy()  # 繪製用影像副本

        # 1. 讀取偵測結果
        detections = []  # 蒐集符合類別的偵測
        for box in results[0].boxes:  # 逐框處理
            cls_id = int(box.cls[0])  # 取得類別 ID
            if cls_id != TARGET_CLASS_ID:  # 非目標類別
                continue  # 跳過

            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)  # 框中心
            detections.append((cx, cy, x1, y1, x2, y2))  # 存入偵測列表

        # 2. 更新追蹤器
        tracked_objects = tracker.update(detections)  # 以新偵測更新追蹤

        # 3. 整理成列表
        sorted_targets = []  # 目標清單(便於後續排序/顯示)
        for obj_id, (centroid, box_data) in tracked_objects.items():  # 走訪追蹤表
            cx, cy, x1, y1, x2, y2 = box_data  # 解構框資料
            sorted_targets.append({  # 建立統一結構
                'id': obj_id,  # 內部追蹤 ID(不顯示)
                'cx': centroid[0],  # 以追蹤器質心為準
                'cy': centroid[1],  # 以追蹤器質心為準
                'w': x2 - x1,  # 框寬
                'h': y2 - y1,  # 框高
                'box': box_data,  # 原框資料
                'grid_label': "?"  # 之後填入網格標籤
            })

        # 4. 智慧網格推算
        if len(sorted_targets) > 0:  # 有目標時才估網格
            ws = [t['w'] for t in sorted_targets]  # 收集寬
            hs = [t['h'] for t in sorted_targets]  # 收集高
            cxs = np.array([t['cx'] for t in sorted_targets])  # X 中心
            cys = np.array([t['cy'] for t in sorted_targets])  # Y 中心

            median_w = np.median(ws)  # 寬中位數
            median_h = np.median(hs)  # 高中位數

            sorted_cxs = np.sort(cxs)  # X 排序
            diffs_x = np.diff(sorted_cxs)  # 鄰差
            valid_dx = diffs_x[(diffs_x > median_w * 0.8) & (diffs_x < median_w * 4.0)]  # 合理間距範圍
            grid_dx = np.median(valid_dx) if len(valid_dx) > 0 else median_w * 1.5  # 估計欄距

            sorted_cys = np.sort(cys)  # Y 排序
            diffs_y = np.diff(sorted_cys)  # 鄰差
            valid_dy = diffs_y[(diffs_y > median_h * 0.8) & (diffs_y < median_h * 4.0)]  # 合理列距範圍
            grid_dy = np.median(valid_dy) if len(valid_dy) > 0 else median_h * 1.5  # 估計列距

            min_x = np.min(cxs)  # 最左 X
            min_y = np.min(cys)  # 最上 Y

            for t in sorted_targets:  # 為每個目標推定行列
                col_idx = int(round((t['cx'] - min_x) / grid_dx))  # 欄索引
                row_idx = int(round((t['cy'] - min_y) / grid_dy))  # 行索引

                if 0 <= row_idx < len(ROW_LABELS) and col_idx < 9:  # 邊界檢查
                    t['grid_label'] = f"{ROW_LABELS[row_idx]}{col_idx + 1}"  # 組合標籤
                else:
                    t['grid_label'] = "?"  # 超界以 ? 表示

        # 5. 依 X 排序（左 → 右）
        sorted_targets.sort(key=lambda k: k['cx'])  # 左到右排列

        target_info = None  # 目前被鎖定的目標資訊

        if locked_target_id is None and len(sorted_targets) > 0:  # 初始鎖定
            locked_target_id = sorted_targets[0]['id']  # 鎖定最左者

        for t in sorted_targets:  # 尋找鎖定目標
            if t['id'] == locked_target_id:  # 比對 ID
                target_info = t  # 取得資訊
                break  # 結束查找

        # 中線
        cv2.line(annotated_frame, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 1)  # 畫中央垂直線

        # 6. 畫框與標籤（取消 ID 顯示，只顯示 grid_label）
        for t in sorted_targets:  # 走訪所有目標
            tid = t['id']  # 追蹤 ID(不顯示)
            grid_lbl = t['grid_label']  # 網格標籤
            _, _, x1, y1, x2, y2 = t['box']  # 取框

            is_selected = (tid == locked_target_id)  # 是否為鎖定
            color = (0, 255, 0) if is_selected else (0, 0, 255)  # 鎖定綠、其他紅
            thickness = 2 if is_selected else 1  # 鎖定加粗

            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)  # 畫框

            # *** 這裡只顯示網格標籤，不再顯示 ID ***
            label_text = f"{grid_lbl}"  # 標籤文字
            cv2.putText(annotated_frame, label_text, (x1, y1 - 5),  # 繪製標籤
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        action = "STOP"  # 預設動作
        current_speed = SPEED_NORMAL  # 預設速度

        # 7. 瞄準邏輯
        if target_info is not None:  # 若已鎖定到目標
            cx, _, x1, y1, x2, y2 = target_info['box']  # 取框中心 X 與範圍
            obj_cy = target_info['cy']  # 取中心 Y (僅顯示用)
            box_width = x2 - x1  # 目標寬度

            zone_width = box_width * AIM_ZONE_RATIO  # 停止區寬
            zone_left = cx - zone_width / 2  # 停止區左界
            zone_right = cx + zone_width / 2  # 停止區右界

            slow_width = box_width * SLOW_ZONE_RATIO  # 減速區寬
            slow_left = cx - slow_width / 2  # 減速區左界
            slow_right = cx + slow_width / 2  # 減速區右界

            cv2.line(annotated_frame, (int(zone_left), 0), (int(zone_left), FRAME_HEIGHT), (0, 255, 255), 1)  # 畫停止左界
            cv2.line(annotated_frame, (int(zone_right), 0), (int(zone_right), FRAME_HEIGHT), (0, 255, 255), 1)  # 畫停止右界
            cv2.circle(annotated_frame, (cx, obj_cy), 5, (0, 0, 255), -1)  # 標示目標中心

            if zone_left <= center_x <= zone_right:  # 中心落於停止區
                action = "STOP"  # 停止
                status_msg = f"LOCKED {target_info['grid_label']}"  # 顯示鎖定標籤
                cv2.putText(annotated_frame, status_msg, (10, 30),  # 繪製狀態
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                if center_x < zone_left:  # 在左邊需要向右
                    action = "RIGHT"  # 右移
                    cv2.putText(annotated_frame, "RIGHT >>", (10, 30),  # 顯示方向
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                else:  # 在右邊需要向左
                    action = "LEFT"  # 左移
                    cv2.putText(annotated_frame, "<< LEFT", (10, 30),  # 顯示方向
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                if slow_left <= center_x <= slow_right:  # 進入減速區
                    current_speed = SPEED_SLOW  # 改用減速參數
                    cv2.putText(annotated_frame, "SLOW", (10, 60),  # 顯示減速
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                else:
                    current_speed = SPEED_NORMAL  # 否則一般速度
        else:
            action = "STOP"  # 無目標時停止
            status_text = f"LOST {locked_target_id}" if locked_target_id is not None else "NO TARGET"  # 狀態訊息
            cv2.putText(annotated_frame, status_text, (10, 30),  # 顯示狀態
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

        # --- 發送指令 ---
        current_state = f"{action}_{current_speed}"  # 打包目前狀態字串

        if action == "STOP":  # 需要停止
            if last_action != "STOP":  # 僅在狀態改變時送出
                send_aim_cmd("stop")  # 發送停止
                last_action = "STOP"  # 更新狀態
        else:
            if current_state != last_action:  # 只有在方向/速度改變時傳送
                if action == "LEFT":  # 左移
                    send_aim_cmd(f"left,{current_speed}")  # 發送左移與速度
                elif action == "RIGHT":  # 右移
                    send_aim_cmd(f"right,{current_speed}")  # 發送右移與速度
                last_action = current_state  # 更新狀態

        # 顯示 FPS（左下角）
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, FRAME_HEIGHT - 10),  # 於左下顯示 FPS
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("Auto Tracking", annotated_frame)  # 顯示視窗

        # --- 鍵盤控制 ---
        key = cv2.waitKey(1) & 0xFF  # 讀取鍵盤
        if key == ord('q'):  # q 離開
            send_aim_cmd("stop")  # 結束前停止
            break  # 跳出迴圈

        # 下 / 上一個目標（依左右排序）
        elif (key == ord('+') or key == ord('=')) and len(sorted_targets) > 0:  # + 或 = 切到下一個
            current_idx = -1  # 目前索引預設
            for i, t in enumerate(sorted_targets):  # 尋找鎖定索引
                if t['id'] == locked_target_id:
                    current_idx = i
                    break

            new_idx = (current_idx + 1) % len(sorted_targets)  # 循環下一個
            locked_target_id = sorted_targets[new_idx]['id']  # 更新鎖定
            print(f"[UI] Switched to {sorted_targets[new_idx]['grid_label']}")  # 提示

        elif (key == ord('-') or key == ord('_')) and len(sorted_targets) > 0:  # - 或 _ 切到上一個
            current_idx = -1  # 目前索引預設
            for i, t in enumerate(sorted_targets):  # 尋找鎖定索引
                if t['id'] == locked_target_id:
                    current_idx = i
                    break

            new_idx = (current_idx - 1 + len(sorted_targets)) % len(sorted_targets)  # 循環上一個
            locked_target_id = sorted_targets[new_idx]['id']  # 更新鎖定
            print(f"[UI] Switched to {sorted_targets[new_idx]['grid_label']}")  # 提示

        # === 數字鍵 0~9 選洞 ===
        elif ord('0') <= key <= ord('9'):  # 偵測 0–9 鍵
            digit = chr(key)  # 轉字元
            if digit in DIGIT_TO_GRID:  # 落於對照表
                want_label = DIGIT_TO_GRID[digit]  # 取得目標標籤
                found = False  # 是否找到
                for t in sorted_targets:  # 走訪可見目標
                    if t['grid_label'] == want_label:  # 比對標籤
                        locked_target_id = t['id']  # 鎖定該目標
                        found = True  # 標記找到
                        print(f"[UI] Key {digit} → {want_label} (locked)")  # 提示
                        break  # 中止搜尋
                if not found:  # 找不到對應標籤
                    print(f"[UI] Key {digit} → {want_label}, but not visible / detected.")  # 顯示未見

    cap.release()  # 釋放攝影機
    cv2.destroyAllWindows()  # 關閉所有視窗

if __name__ == "__main__":  # 直執入口
    main(None)  # 執行主程式(無 IPC)
