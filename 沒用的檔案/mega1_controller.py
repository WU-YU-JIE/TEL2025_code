# -*- coding: utf-8 -*-
"""
mega1_controller.py
- MegaController（維持不動）
- 追加 ControllerWindow（PyQt5 GUI）
  * 預設 BAUD=115200、預選 COM19
  * 送指令：數字（NEO），F/R/S（步進；S=後退）
  * 新增「Arduino 輸出」視窗區塊：持續讀取序列埠並顯示回傳訊息
"""
from __future__ import annotations
import threading, time
from typing import Optional, TYPE_CHECKING

try:
    import serial as pyserial
except Exception:
    pyserial = None  # type: ignore

if TYPE_CHECKING:
    from serial import Serial as SerialT
else:
    class SerialT: pass


class MegaController:
    MODE_CHASSIS = "CHASSIS"
    MODE_SHOOTER = "SHOOTER"

    def __init__(self, port="COM19", baud=115200, send_interval_s=0.08, verbose=True):
        self.port, self.baud, self.verbose = port, baud, verbose
        self._ser: Optional[SerialT] = None
        self._vx = 0.0; self._vy = 0.0; self._wz = 0.0
        self._interval = max(0.01, float(send_interval_s))
        self._mode = self.MODE_CHASSIS
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._connect()
        self._start_tx_loop()

    def _connect(self):
        if pyserial is None:
            if self.verbose: print("[Mega] pyserial 未安裝，dryrun")
            return
        try:
            self._ser = pyserial.Serial(self.port, self.baud, timeout=0.0, write_timeout=0.0)
            if self.verbose: print(f"[Mega] OPEN {self.port} @ {self.baud}")
            time.sleep(1.5)
            self._send_line(f"MODE {self._mode}")
        except Exception as e:
            self._ser = None
            if self.verbose: print(f"[Mega] 開啟失敗：{e}（dryrun）")

    def _close(self):
        if self._ser:
            try: self._ser.close()
            finally: self._ser = None
            if self.verbose: print("[Mega] CLOSED")

    def _send_line(self, s: str):
        if not s.endswith("\n"): s += "\n"
        if self._ser:
            try: self._ser.write(s.encode("ascii"))
            except Exception as e:
                if self.verbose: print(f"[Mega] 寫入失敗：{e}")
        else:
            if self.verbose: print(f"[Mega] (drysend) {s.strip()}")

    def _start_tx_loop(self):
        def loop():
            next_t = 0.0
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_t:
                    with self._lock:
                        mode, vx, vy, wz = self._mode, self._vx, self._vy, self._wz
                    if mode == self.MODE_CHASSIS:
                        self._send_line(f"{vx:.2f},{vy:.2f},{wz:.2f}")
                    next_t = now + self._interval
                time.sleep(0.002)
        threading.Thread(target=loop, daemon=True).start()

    # ---- 公開 API ----
    def move_forward(self, percent: int):
        p = max(-100, min(100, int(percent)))
        with self._lock: self._vx = p / 100.0
        if abs(p) > 0: self.set_mode(self.MODE_CHASSIS)

    def move_XY(self, percent: int):
        p = max(-100, min(100, int(percent)))
        with self._lock: self._vy = p / 100.0
        if abs(p) > 0: self.set_mode(self.MODE_CHASSIS)
    move_xy = move_XY

    def rotate(self, percent: int):
        p = max(-100, min(100, int(percent)))
        with self._lock: self._wz = p / 100.0
        if abs(p) > 0: self.set_mode(self.MODE_CHASSIS)

    # 射手端
    def feed(self):    self.set_mode(self.MODE_SHOOTER); self._send_line("R")
    def reverse(self): self.set_mode(self.MODE_SHOOTER); self._send_line("F")
    def set_neo(self, speed: int):
        v = int(max(-100, min(100, speed)))
        self.set_mode(self.MODE_SHOOTER); self._send_line(str(v))

    def set_mode(self, mode: str):
        if mode not in (self.MODE_CHASSIS, self.MODE_SHOOTER): return
        with self._lock:
            if mode == self._mode: return
            self._mode = mode
        self._send_line(f"MODE {mode}")
        time.sleep(0.2)

    def close(self):
        self._stop.set(); time.sleep(0.05); self._close()


# ============================
# 追加：Controller 視窗 GUI（含 Arduino 輸出顯示）
# ============================
try:
    from PyQt5.QtCore import QTimer, pyqtSignal
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QLineEdit, QGroupBox, QTextEdit
    import serial
    import serial.tools.list_ports as list_ports

    class ControllerWindow(QWidget):
        """
        Arduino 控制視窗
        - 預設 BAUD=115200、預選 COM19
        - 指令：數字（NEO）、F/R/S（步進；S=後退）
        - 下方文字框顯示 Arduino 回傳（非阻塞輪詢）
        """
        BAUD = 115200
        DEFAULT_PORT = "COM19"

        arduino_conn_changed = pyqtSignal(bool, str)
        message = pyqtSignal(str)

        def __init__(self, parent=None):
            super().__init__(parent)
            self.setWindowTitle("Arduino 控制")
            self.resize(520, 360)

            # 連線列
            self.port_combo = QComboBox(); self._refresh_ports(); self._preselect_port(self.DEFAULT_PORT)
            btn_refresh = QPushButton("刷新"); btn_refresh.clicked.connect(self._refresh_ports)
            self.btn_conn = QPushButton("連線"); self.btn_disc = QPushButton("斷線")
            self.btn_conn.clicked.connect(self._connect); self.btn_disc.clicked.connect(self._disconnect)
            self.btn_disc.setEnabled(False)

            top = QHBoxLayout()
            top.addWidget(QLabel("COM：")); top.addWidget(self.port_combo, 1)
            top.addWidget(btn_refresh); top.addWidget(self.btn_conn); top.addWidget(self.btn_disc)

            # 指令列
            self.inp = QLineEdit(); self.inp.setPlaceholderText("輸入數字→Enter 直接送給 Arduino（NEO）")
            self.inp.returnPressed.connect(self._send_text)
            bF = QPushButton("F"); bR = QPushButton("R"); bS = QPushButton("S")
            bF.clicked.connect(lambda: self._send_literal("F"))
            bR.clicked.connect(lambda: self._send_literal("R"))
            bS.clicked.connect(lambda: self._send_literal("S"))
            grp = QGroupBox("指令")
            g = QVBoxLayout(grp)
            g.addWidget(self.inp)
            row = QHBoxLayout(); row.addWidget(bF); row.addWidget(bR); row.addWidget(bS)
            g.addLayout(row)

            # 底部：Arduino 輸出
            out_grp = QGroupBox("Arduino 輸出")
            vg = QVBoxLayout(out_grp)
            self.out_view = QTextEdit(); self.out_view.setReadOnly(True)
            vg.addWidget(self.out_view)

            root = QVBoxLayout(self)
            root.addLayout(top)
            root.addWidget(grp)
            root.addWidget(out_grp, 1)

            self._ser = None; self._cur_port = ""; self._rxbuf = b""

            # 非阻塞輪詢 RX
            self._rx_timer = QTimer(self); self._rx_timer.timeout.connect(self._poll_rx); self._rx_timer.start(50)

        # --- 連線 ---
        def _refresh_ports(self):
            self.port_combo.clear()
            try:
                for p in list_ports.comports():
                    self.port_combo.addItem(p.device)
            except Exception:
                pass

        def _preselect_port(self, name: str):
            idx = self.port_combo.findText(name)
            if idx >= 0: self.port_combo.setCurrentIndex(idx)

        def _connect(self):
            if self._ser: return
            port = self.port_combo.currentText().strip()
            if not port: return
            try:
                self._ser = serial.Serial(port=port, baudrate=self.BAUD, timeout=0.0)  # 非阻塞
                self._cur_port = port
                self.btn_conn.setEnabled(False); self.btn_disc.setEnabled(True)
                self.arduino_conn_changed.emit(True, port)
                self._log(f"[Connected] {port} @ {self.BAUD}")
                self.setWindowTitle(f"Arduino 控制（{port}）")
            except Exception as e:
                self._log(f"[連線失敗] {e}")

        def _disconnect(self):
            if self._ser:
                try: self._ser.close()
                except Exception: pass
            port = self._cur_port
            self._ser = None; self._cur_port = ""; self._rxbuf = b""
            self.btn_conn.setEnabled(True); self.btn_disc.setEnabled(False)
            self.arduino_conn_changed.emit(False, port)
            self._log(f"[Disconnected] {('from '+port) if port else ''}")
            self.setWindowTitle("Arduino 控制（未連線）")

        # --- TX/RX ---
        def _write_line(self, s: str):
            if not self._ser:
                self._log("尚未連線 Arduino"); return
            try:
                self._ser.write((s+"\n").encode("utf-8"))
                self._log(f">> {s}")
            except Exception as e:
                self._log(f"[TX 失敗] {e}")

        def _send_text(self):
            s = self.inp.text().strip()
            if not s: return
            self._write_line(s); self.inp.clear()

        def _send_literal(self, token: str):
            self._write_line(token)

        def _poll_rx(self):
            if not self._ser: return
            try:
                n = self._ser.in_waiting
                if n:
                    data = self._ser.read(n)
                    self._rxbuf += data
                    while b"\n" in self._rxbuf:
                        line, self._rxbuf = self._rxbuf.split(b"\n", 1)
                        text = line.decode("utf-8", "ignore").rstrip("\r")
                        self._log(text)
            except Exception:
                pass

        def _log(self, s: str):
            self.out_view.append(s)
            self.message.emit(s)

        def closeEvent(self, e):
            try:
                if self._ser: self._ser.close()
            except Exception: pass
            super().closeEvent(e)
except Exception:
    pass
