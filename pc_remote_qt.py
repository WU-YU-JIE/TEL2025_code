# pc_remote_qt.py
# -*- coding: utf-8 -*-
import sys
import socket
import threading
import json
import os
import time  # 新增 time 模組
from PyQt6 import uic, QtCore, QtGui
from PyQt6.QtWidgets import QMainWindow, QApplication, QMessageBox, QPushButton
from PyQt6.QtCore import pyqtSignal, Qt

DEFAULT_HOST = "10.254.97.208"  # 請依實際 IP 修改
DEFAULT_PORT = 5000
PRESET_DIR = "presets"

class RemoteClient:
    def __init__(self):
        self.sock = None
        self.lock = threading.Lock()
        self.running = False
        self.on_recv = None

    def connect(self, host, port):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3.0)
        s.connect((host, port))
        s.settimeout(None)
        self.sock = s
        self.running = True
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

    def _recv_loop(self):
        try:
            f = self.sock.makefile("r")
            for line in f:
                line = line.rstrip("\r\n")
                if self.on_recv:
                    self.on_recv(line)
            if self.on_recv:
                self.on_recv("[INFO] Server closed connection")
        except Exception as e:
            if self.on_recv and self.running:
                self.on_recv(f"[ERR] recv: {e}")
        finally:
            self.close()

    def send_line(self, text):
        with self.lock:
            if not self.sock:
                raise RuntimeError("not connected")
            data = (text.strip() + "\n").encode("utf-8")
            self.sock.sendall(data)

    def close(self):
        with self.lock:
            self.running = False
            if self.sock:
                try: self.sock.close()
                except: pass
            self.sock = None

class MainWindow(QMainWindow):
    # 定義信號：用於跨執行緒更新 UI
    log_signal = pyqtSignal(str)
    aim_data_signal = pyqtSignal(float, float) # 用於接收 AIM 角度

    def __init__(self):
        super().__init__()
        # 載入 .ui 檔案
        if not os.path.exists("remote_layout.ui"):
            QMessageBox.critical(self, "錯誤", "找不到 remote_layout.ui")
            sys.exit(1)
        uic.loadUi("remote_layout.ui", self)
        
        # 初始化
        self.client = RemoteClient()
        self.client.on_recv = self._on_client_recv
        
        # 連接信號槽
        self.log_signal.connect(self._append_log)
        self.aim_data_signal.connect(self._update_aim_display)

        self.connected = False
        self.keys = {k: False for k in ["w", "a", "s", "d", "left", "right"]}
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        self.AIM_STEP = 1.0
        self.m1_angle = 0.0
        self.m2_angle = 0.0
        
        # === 新增：Log 過濾相關變數 ===
        self.last_log_msg = ""
        self.last_log_time = 0.0
        self.duplicate_count = 0
        # ===========================

        self.current_file_slot = None
        self._ensure_preset_dir()
        self._setup_ui_logic()
        
        self.txt_log.setPlaceholderText("等待連線中... 系統就緒")
        # 預填 IP
        self.input_ip.setText(DEFAULT_HOST)
        self.input_port.setText(str(DEFAULT_PORT))

    def _setup_ui_logic(self):
        self.btn_connect.clicked.connect(self._toggle_connect)
        self.slider_speed.valueChanged.connect(self._update_speed_label)
        
        self.btn_step_f.clicked.connect(lambda: self._send_step("F"))
        self.btn_step_r.clicked.connect(lambda: self._send_step("R"))
        self.btn_step_s.clicked.connect(lambda: self._send_step("S"))
        
        self.btn_servo_1.clicked.connect(lambda: self._send_servo("S1"))
        self.btn_servo_2.clicked.connect(lambda: self._send_servo("S2"))
        self.btn_servo_3.clicked.connect(lambda: self._send_servo("S3"))
        
        self.btn_neo_0.clicked.connect(lambda: self._send_neo_val(0))
        self.btn_neo_n50.clicked.connect(lambda: self._send_neo_val(-50))
        self.btn_neo_n100.clicked.connect(lambda: self._send_neo_val(-100))
        
        self.btn_m1_dec.clicked.connect(lambda: self._aim_delta(1, -self.AIM_STEP))
        self.btn_m1_inc.clicked.connect(lambda: self._aim_delta(1, self.AIM_STEP))
        self.btn_m1_set0.clicked.connect(lambda: self._aim_reset_origin(1))
        self.btn_m1_go0.clicked.connect(lambda: self._aim_go_origin(1))
        
        self.btn_m2_dec.clicked.connect(lambda: self._aim_delta(2, -self.AIM_STEP))
        self.btn_m2_inc.clicked.connect(lambda: self._aim_delta(2, self.AIM_STEP))
        self.btn_m2_set0.clicked.connect(lambda: self._aim_reset_origin(2))
        self.btn_m2_go0.clicked.connect(lambda: self._aim_go_origin(2))
        
        self.btn_raw.clicked.connect(self._send_raw)
        self.btn_status.clicked.connect(self._send_status)
        self.input_raw.returnPressed.connect(self._send_raw)

        for i in range(9):
            btn = QPushButton(f"槽{i+1}")
            btn.clicked.connect(lambda ch, idx=i+1: self._select_file_slot(idx))
            self.grid_files.addWidget(btn, i // 3, i % 3)
            
        for j in range(10):
            btn = QPushButton(f"P{j+1}")
            btn.clicked.connect(lambda ch, idx=j+1: self._param_click(idx))
            self.grid_params.addWidget(btn, j // 5, j % 5)

    def _ensure_preset_dir(self):
        if not os.path.isdir(PRESET_DIR):
            try: os.makedirs(PRESET_DIR, exist_ok=True)
            except: pass

    # --- 接收與解析 (已修改過濾邏輯) ---
    def _on_client_recv(self, msg):
        # 1. 處理 AIM 角度回傳 "degree:10.5,20.0"
        if msg.startswith("degree:"):
            try:
                content = msg.split(":", 1)[1]
                parts = content.split(",")
                if len(parts) >= 2:
                    m1 = float(parts[0])
                    m2 = float(parts[1])
                    self.aim_data_signal.emit(m1, m2)
            except ValueError:
                pass # 忽略解析錯誤，避免 Log 刷頻
            return # 角度資料已處理，不需進 Log

        # 2. 處理 Log (MEGA_LOG / AIM_LOG)
        is_telemetry = msg.startswith("MEGA_LOG:") or msg.startswith("AIM_LOG:")
        
        if is_telemetry:
            now = time.time()
            # 如果訊息與上一條完全相同
            if msg == self.last_log_msg:
                # 只有超過 5 秒才允許重複顯示一次 (Heartbeat 視覺確認用)
                if now - self.last_log_time < 5.0:
                    self.duplicate_count += 1
                    return
            
            # 訊息不同，或已超時 -> 更新狀態並顯示
            self.last_log_msg = msg
            self.last_log_time = now
            self.duplicate_count = 0
            self.log_signal.emit(f"[Pi] {msg}")

        # 3. 其他回應 (如 [Rx] OK, [INFO] ...)
        else:
            # 非遙測資料，通常很重要，不特別過濾
            self.log_signal.emit(f"[Rx] {msg}")

    def _update_aim_display(self, m1, m2):
        """當收到 Arduino 回傳數值時執行"""
        self.m1_angle = m1
        self.m2_angle = m2
        
        style = "color: #ffeb3b; font-weight: bold;" # 黃色亮字
        self.lbl_m1.setText(f"M1 = <span style='{style}'>{self.m1_angle:.2f}°</span>")
        self.lbl_m2.setText(f"M2 = <span style='{style}'>{self.m2_angle:.2f}°</span>")

    # --- 鍵盤與發送 ---
    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        self._handle_key(event, True)
    
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        self._handle_key(event, False)

    def _handle_key(self, event, is_press):
        key = event.key()
        txt = event.text().lower()
        updated = False
        
        if txt == 'w': self.keys['w'] = is_press; updated = True
        elif txt == 's': self.keys['s'] = is_press; updated = True
        elif txt == 'a': self.keys['a'] = is_press; updated = True
        elif txt == 'd': self.keys['d'] = is_press; updated = True
        elif key == Qt.Key.Key_Left: self.keys['left'] = is_press; updated = True
        elif key == Qt.Key.Key_Right: self.keys['right'] = is_press; updated = True
        
        if updated:
            self._update_chassis_from_keys()
            return

        if is_press:
            if txt == 'f': self._send_step("F")
            elif txt == 'r': self._send_step("R")
            elif txt == 'x': self._send_step("S")
            elif txt == '1': self._send_servo("S1")
            elif txt == '2': self._send_servo("S2")
            elif txt == '3': self._send_servo("S3")
            elif txt == 'l': self._aim_delta(1, self.AIM_STEP)
            elif txt == 'j': self._aim_delta(1, -self.AIM_STEP)
            elif txt == 'i': self._aim_delta(2, self.AIM_STEP)
            elif txt == 'k': self._aim_delta(2, -self.AIM_STEP)

    def _toggle_connect(self):
        if self.connected:
            self.client.close()
            self.connected = False
            self.lbl_status.setText("未連線")
            self.lbl_status.setStyleSheet("color: #888888; font-weight: bold;") 
            self.btn_connect.setText("連線")
            self.btn_connect.setStyleSheet("")
            self._append_log("[INFO] disconnected")
        else:
            host = self.input_ip.text().strip()
            try: port = int(self.input_port.text())
            except: return
            try:
                self.client.connect(host, port)
                self.connected = True
                self.lbl_status.setText(f"已連線: {host}")
                self.lbl_status.setStyleSheet("color: #00e676; font-weight: bold;")
                self.btn_connect.setText("斷線")
                self.btn_connect.setStyleSheet("background-color: #444; border: 1px solid #f44336;") 
                self._append_log("[INFO] connected")
                self.setFocus()
            except Exception as e:
                self._append_log(f"[ERR] Connect fail: {e}")
                QMessageBox.critical(self, "連線失敗", str(e))

    def _append_log(self, txt):
        self.txt_log.append(txt)
        sb = self.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _send_line_safe(self, cmd):
        if not self.connected: return
        try:
            self.client.send_line(cmd)
            # 本地 log 也顯示送出的指令，但過濾高頻 CHASSIS 指令
            if not cmd.startswith("CHASSIS"): 
                self._append_log(f"[PC] {cmd}")
        except Exception as e:
            self._append_log(f"[ERR] send: {e}")
            self._toggle_connect()

    def _update_speed_label(self):
        val = self.slider_speed.value() / 100.0
        self.lbl_speed_val.setText(f"{val:.2f}")
        self._update_chassis_from_keys()
        self.setFocus()

    def _update_chassis_from_keys(self):
        vx = vy = wz = 0.0
        if self.keys['w'] and not self.keys['s']: vx = 1.0
        if self.keys['s'] and not self.keys['w']: vx = -1.0
        if self.keys['d'] and not self.keys['a']: vy = 1.0
        if self.keys['a'] and not self.keys['d']: vy = -1.0
        if self.keys['left'] and not self.keys['right']: wz = -1.0
        if self.keys['right'] and not self.keys['left']: wz = 1.0
        
        sp = self.slider_speed.value() / 100.0
        self.vx, self.vy, self.wz = vx * sp, vy * sp, wz * sp
        
        style = "color: #00e5ff; font-weight: bold;"
        self.lbl_vx.setText(f"vx=<span style='{style}'>{self.vx:.2f}</span>")
        self.lbl_vy.setText(f"vy=<span style='{style}'>{self.vy:.2f}</span>")
        self.lbl_wz.setText(f"wz=<span style='{style}'>{self.wz:.2f}</span>")
        
        if self.connected:
            cmd = f"CHASSIS {self.vx:.2f} {self.vy:.2f} {self.wz:.2f}"
            self._send_line_safe(cmd)

    def _send_step(self, sub): self._send_line_safe(f"STEP {sub}")
    def _send_servo(self, sname): self._send_line_safe(f"SERVO {sname}")
    def _send_neo_val(self, val): self._send_line_safe(f"NEO {val}")
    def _send_raw(self):
        txt = self.input_raw.text().strip()
        if txt: self._send_line_safe(f"RAW {txt}"); self.input_raw.clear()
    def _send_status(self): self._send_line_safe("STATUS")

    def _aim_delta(self, axis, delta):
        target = (self.m1_angle if axis==1 else self.m2_angle) + delta
        self._send_line_safe(f"AIM {axis} {target:.3f}")

    def _aim_reset_origin(self, axis):
        if axis == 1: self.m1_angle = 0.0
        else: self.m2_angle = 0.0
        self._send_line_safe(f"AIM {axis} 0")
        
    def _aim_go_origin(self, axis):
        self._send_line_safe(f"AIM {axis} 0")

    def _select_file_slot(self, slot):
        self.current_file_slot = slot
        fname = f"preset_{slot}.json"
        path = os.path.join(PRESET_DIR, fname)
        if not os.path.exists(path):
            self.lbl_file_info.setText(f"已選: 槽{slot} (空)")
            self.lbl_file_info.setStyleSheet("color: #ff4444")
        else:
            self.lbl_file_info.setText(f"已選: 槽{slot} -> {fname}")
            self.lbl_file_info.setStyleSheet("color: #00e5ff")
        self.lbl_param_info.setText("未選擇參數")

    def _param_click(self, idx):
        if self.current_file_slot is None:
            QMessageBox.warning(self, "提示", "請先選擇檔案槽")
            return
        fname = f"preset_{self.current_file_slot}.json"
        path = os.path.join(PRESET_DIR, fname)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            item = data[idx-1]
            m1 = m2 = 0
            if isinstance(item, (list, tuple)): m1, m2 = float(item[0]), float(item[1])
            elif isinstance(item, dict):
                m1 = float(item.get("m1", item.get("M1", 0)))
                m2 = float(item.get("m2", item.get("M2", 0)))
            
            self._send_line_safe(f"AIM 1 {m1:.3f}")
            self._send_line_safe(f"AIM 2 {m2:.3f}")
            self.lbl_param_info.setText(f"P{idx} sent: {m1:.2f}, {m2:.2f}")
            self.lbl_param_info.setStyleSheet("color: #00e5ff; font-weight: bold;")
        except Exception as e:
            self.lbl_param_info.setText(f"讀取失敗: {e}")
            self.lbl_param_info.setStyleSheet("color: #ff4444;")

    def closeEvent(self, event):
        self.client.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())