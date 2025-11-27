import cv2
import numpy as np
from ultralytics import YOLO

# ================= 設定區域 =================
model_path = r"C:\Users\user\OneDrive - 中原大學\桌面\大學作品集\東京威力\TEL.v2i\runs\detect\train16\weights\best.pt"
ROW_NAMES = ["A", "B", "C", "D"] # 定義列名
CAMERA_ID = 0
# ===========================================

def main():
    print(f"正在載入模型：{model_path}...")
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"錯誤：無法載入模型。請確認路徑。\n{e}")
        return

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("錯誤：無法開啟攝影機")
        return

    print("精準模式已啟動。按 'q' 鍵離開。")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 1. YOLO 偵測
        results = model(frame, conf=0.8, verbose=False, classes=[0])
        
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                w, h = x2 - x1, y2 - y1
                detections.append({'cx': cx, 'cy': cy, 'w': w, 'h': h, 'box': [int(x1), int(y1), int(x2), int(y2)]})

        # 2. 執行精準標記邏輯
        if len(detections) > 0:
            label_grid(frame, detections)

        cv2.imshow("Precision Grid Labeling", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def label_grid(img, detections):
    """
    核心演算法：分層聚類與動態填補
    """
    # 1. 取得全域統計數據 (用來當作一把尺)
    # 計算所有洞的中位數寬高，作為「標準單位」
    median_h = np.median([d['h'] for d in detections])
    median_w = np.median([d['w'] for d in detections])
    
    # 垂直容忍度：如果兩個洞的 Y 差小於這個值，視為同一排
    # 通常設為高度的 0.5 ~ 0.7 倍
    row_tolerance = median_h * 0.6

    # 2. 分排 (Row Clustering)
    # 先依照 Y 座標排序所有洞
    sorted_by_y = sorted(detections, key=lambda k: k['cy'])
    
    rows = [] # 存放每一排的洞列表 [[hole1, hole2], [hole3, hole4]]
    if sorted_by_y:
        current_row = [sorted_by_y[0]]
        rows.append(current_row)
        
        for i in range(1, len(sorted_by_y)):
            hole = sorted_by_y[i]
            prev_hole_avg_y = np.mean([h['cy'] for h in current_row])
            
            # 如果這個洞的 Y 跟這一排平均 Y 差很小，算同一排
            if abs(hole['cy'] - prev_hole_avg_y) < row_tolerance:
                current_row.append(hole)
            else:
                # 否則這是一個新的一排
                current_row = [hole]
                rows.append(current_row)

    # 3. 決定每一排的「列標籤」 (A, B, C, D)
    # 我們需要知道第一排到底是 A 還是 B (如果 A 整排沒抓到)
    # 方法：找出最上面的那一排的 Y 座標，看它離「畫面頂端」或「上一排」多遠
    
    # 計算每一排的平均 Y 座標 (Row Baselines)
    row_baselines = [np.mean([h['cy'] for h in r]) for r in rows]
    
    # 估計垂直間距 (Pitch Y)
    # 如果只有一排，無法估算，就用 1.5 倍洞高當預設
    # 如果有多排，計算排與排之間的距離
    if len(row_baselines) > 1:
        diffs = np.diff(row_baselines)
        # 過濾掉太誇張的距離 (例如雜訊)
        valid_diffs = diffs[(diffs > median_h) & (diffs < median_h * 5)]
        if len(valid_diffs) > 0:
            pitch_y = np.median(valid_diffs)
        else:
            pitch_y = median_h * 1.8
    else:
        pitch_y = median_h * 1.8

    # 找出全域最頂端的 Y (Global Min Y)
    # 注意：這裡假設畫面中最上面的洞，有很高機率是 A 排。
    # 如果要支援「A排全滅，B排變第一排」的狀況，需要假設 A1 應該出現的絕對位置
    # 或是比較各排之間的相對距離。這裡採用「相對距離法」。
    
    # 我們假設第一群抓到的就是起始群 (Row 0 candidate)
    # 接著看後面的群跟第一群差多遠
    start_y = row_baselines[0]
    
    row_mapping = {} # 紀錄: 第 i 群 -> 對應到 A/B/C/D 哪一列 (0,1,2,3)
    
    # 強制第一群為 Row 0 (A)，除非你有固定的 ROI 邊界
    # 如果要更進階，可以檢測 start_y 是否大於 pitch_y * 1.5 (代表 A 排空了)
    first_row_index = 0 
    # 進階：如果第一個洞的 Y 座標很大 (例如 > 200px)，且間距很小，可能第一排是 B
    # 這邊先採取保守策略：偵測到的最上面那排預設為 A (index 0)
    
    for i, baseline in enumerate(row_baselines):
        # 計算這排距離第一排有幾個 pitch_y
        # round((目前Y - 第一排Y) / 間距)
        dist_units = int(round((baseline - start_y) / pitch_y))
        mapped_index = first_row_index + dist_units
        row_mapping[i] = mapped_index

    # 4. 處理每一排內部的行 (Column) 標記
    # 這裡也需要全域的最左 X (Global Min X) 來校正
    # 我們取「所有排中，最左邊那個洞」的 X 作為 Col 1 的基準
    all_cx = [d['cx'] for d in detections]
    global_min_x = np.min(all_cx) if all_cx else 0
    
    # 估計水平間距 (Pitch X)
    all_sorted_x = sorted(all_cx)
    x_diffs = np.diff(all_sorted_x)
    valid_x_diffs = x_diffs[(x_diffs > median_w * 0.8) & (x_diffs < median_w * 3.0)]
    if len(valid_x_diffs) > 0:
        pitch_x = np.median(valid_x_diffs)
    else:
        pitch_x = median_w * 1.5

    for i, row_holes in enumerate(rows):
        # 取得這排是對應到 A, B, C, D 哪一個
        row_char_idx = row_mapping.get(i, 0)
        
        if row_char_idx >= len(ROW_NAMES):
            continue # 超出 D 排，忽略

        row_char = ROW_NAMES[row_char_idx]

        # 針對這排的洞，依照 X 排序
        row_holes.sort(key=lambda k: k['cx'])

        for hole in row_holes:
            # 計算它是第幾行 (1, 2, 3)
            # 公式：(目前X - 全域最左X) / 水平間距
            # 使用全域最左 X 可以防止「這一排剛好缺了第一行」導致第二行被標成 1
            col_idx = int(round((hole['cx'] - global_min_x) / pitch_x))
            col_num = col_idx + 1

            # 繪圖
            if col_num <= 10: # 防呆
                label = f"{row_char}{col_num}"
                draw_label(img, hole['box'], label)

def draw_label(img, box, label):
    x1, y1, x2, y2 = box
    # 畫框
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # 畫標籤
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    cv2.rectangle(img, (x1, y1 - 25), (x1 + tw, y1), (0, 0, 0), -1)
    cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

if __name__ == "__main__":
    main()