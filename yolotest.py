from ultralytics import YOLO
import cv2
import os

# 1. 設定 ONNX 模型路徑
# 這裡直接指向你剛剛轉好的 onnx 檔案
onnx_path = r"C:\Users\user\OneDrive - 中原大學\桌面\大學作品集\東京威力\TEL.v2i\runs\detect\train15\weights\best.onnx"

print(f"正在檢查路徑: {onnx_path}")
if not os.path.exists(onnx_path):
    print("錯誤：找不到 .onnx 檔案！請確認第一步的轉檔是否成功。")
    exit()

# 2. 載入 ONNX 模型
# task='detect' 是選填的，但明確指定可以避免部分警告
print("正在載入 ONNX 模型 (這可能需要幾秒鐘)...")
try:
    model = YOLO(onnx_path, task='detect')
    print("ONNX 模型載入成功！")
except Exception as e:
    print(f"載入失敗: {e}")
    print("請確認你有安裝 onnxruntime: pip install onnx onnxruntime")
    exit()

# 3. 開啟鏡頭
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("無法開啟攝影機")
    exit()

print("=== ONNX 推論測試開始 (按 'q' 離開) ===")

while True:
    success, frame = cap.read()
    if not success:
        break

    # 4. 推論
    # 使用 ONNX 時，Ultralytics 內部會自動呼叫 onnxruntime
    results = model.predict(frame, conf=0.4, verbose=False)

    # 5. 繪圖與顯示
    annotated_frame = results[0].plot()
    
    # 加上標示確認我們正在用 ONNX
    cv2.putText(annotated_frame, "Mode: ONNX", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 ONNX Test", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()