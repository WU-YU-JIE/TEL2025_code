from ultralytics import YOLO

# 1. 設定模型路徑 (使用 raw string r"..." 避免路徑錯誤)
model_path = r"C:\Users\user\OneDrive - 中原大學\桌面\大學作品集\東京威力\TEL.v2i\runs\detect\train16\weights\best.pt"

print(f"正在載入模型：{model_path}")

# 2. 載入 PyTorch 模型
try:
    model = YOLO(model_path)
except Exception as e:
    print(f"找不到模型，請確認路徑。\n錯誤訊息: {e}")
    exit()

print("模型載入成功，開始轉換為 ONNX 格式...")

# 3. 執行轉檔 (export)
# format='onnx': 指定轉出格式
# opset=12: 這是 ONNX 的版本，12 對樹莓派相容性較好，若報錯可改用預設值
# simplify=True: 簡化模型結構，讓推論更快 (需要安裝 onnx-simplifier，若失敗可改 False)
try:
    path = model.export(format='onnx', opset=12)
    print(f"\n轉檔成功！檔案已儲存於: {path}")
except Exception as e:
    print(f"\n轉檔失敗: {e}")
    print("請嘗試安裝必要套件: pip install onnx onnxruntime")