# -*- coding: utf-8 -*-
"""
aim.py
- Aimer（保持不動）
- 追加 AimWindow（PyQt5 GUI），預設 BAUD=115200、預選 COM18
  * 套用：各送一行 '1,<x>'、'2,<y>'
  * 新增「Arduino 輸出」視窗區塊：持續讀取序列埠並顯示回傳訊息
"""
from __future__ import annotations
from typing import Optional, Tuple, TYPE_CHECKING
import time

try:
    import serial as pyserial
except Exception:
    pyserial = None  # type: ignore

if TYPE_CHECKING:
    from serial import Serial as SerialT
else:
    class SerialT: pass

def _clamp(x, a, b): return a if x < a else b if x > b else x


class Aimer:
    def __init__(self, port="COM18", baud=115200, send_interval_ms=80, verbose=True,
                 x_min_deg=-30.0, x_max_deg=+30.0, y_min_deg=+10.0, y_max_deg=+80.0,
                 x_sens_deg_s=60.0, y_sens_deg_s=60.0):
        self.port, self.baud, self.verbose = port, baud, verbose
        self._ser: Optional[SerialT] = None
        self._interval = send_interval_ms / 1000.0
        self._last_send = 0.0
        self._x_rng = (float(x_min_deg), float(x_max_deg))
        self._y_rng = (float(y_min_deg), float(y_max_deg))
        self._x = (x_min_deg + x_max_deg) / 2.0
        self._y = (y_min_deg + y_max_deg) / 2.0
        self._last_xy: Tuple[float, float] = (9999, 9999)
        self._sx = float(x_sens_deg_s)
        self._sy = float(y_sens_deg_s)
        self._connect()

    def _connect(self):
        if pyserial is None:
            if self.verbose: print("[AIM] pyserial 未安裝，dryrun")
            return
        try:
            self._ser = pyserial.Serial(self.port, self.baud, timeout=0.02)
            if self.verbose: print(f"[AIM] OPEN {self.port} @ {self.baud}")
            time.sleep(1.2)
        except Exception as e:
            self._ser = None
            if self.verbose: print(f"[AIM] 開啟失敗：{e}（dryrun）")

    def _send_line(self, s: str):
        if not s.endswith("\n"): s += "\n"
        if self._ser:
            try: self._ser.write(s.encode("utf-8"))
            except Exception as e:
                if self.verbose: print(f"[AIM] 寫入失敗：{e}")
        else:
            if self.verbose: print(f"[AIM] (drysend) {s.strip()}")

    def nudge_from_stick(self, rx: float, ry: float, dt: float):
        if dt <= 0: return
        dx = float(rx) * self._sx * dt
        dy = float(-ry) * self._sy * dt
        self._x = _clamp(self._x + dx, *self._x_rng)
        self._y = _clamp(self._y + dy, *self._y_rng)
        self._maybe_send()

    def _maybe_send(self):
        now = time.time()
        if now - self._last_send < self._interval: return
        cur = (round(self._x, 3), round(self._y, 3))
        if cur == self._last_xy: return
        self._last_send = now; self._last_xy = cur
        self._send_line(f"1,{cur[0]:.3f}")
        self._send_line(f"2,{cur[1]:.3f}")

    def home_center(self):
        self._x = sum(self._x_rng) * 0.5
        self._y = sum(self._y_rng) * 0.5
        self._maybe_send()

    def set_xy_deg(self, x_deg: float, y_deg: float):
        self._x = _clamp(float(x_deg), *self._x_rng)
        self._y = _clamp(float(y_deg), *self._y_rng)
        self._maybe_send()

    def close(self):
        if self._ser:
            try: self._ser.close()
            finally: self._ser = None
            if self.verbose: print("[AIM] CLOSED")


# =======================
# 追加：AimWindow（含 Arduino 輸出顯示）
# =======================
try:
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSpinBox, QPushButton, QGroupBox, QComboBox, QTextEdit
    import serial
    import serial.tools.list_ports as list_ports

    class AimWindow(QWidget):
        BAUD = 115200
        DEFAULT_PORT = "COM18"

        def __init__(self, parent=None):
            super().__init__(parent)
            self.setWindowTitle("Aim 視窗（未連線）")
            self.resize(520, 320)

            # 連線列
            self.port_combo = QComboBox(); self._refresh_ports(); self._preselect_port(self.DEFAULT_PORT)
            btn_refresh = QPushButton("刷新"); btn_refresh.clicked.connect(self._refresh_ports)
            self.btn_conn = QPushButton("連線"); self.btn_disc = QPushButton("斷線")
            self.btn_conn.clicked.connect(self._connect); self.btn_disc.clicked.connect(self._disconnect)
            self.btn_disc.setEnabled(False)
            top = QHBoxLayout(); top.addWidget(QLabel("COM：")); top.addWidget(self.port_combo, 1)
            top.addWidget(btn_refresh); top.addWidget(self.btn_conn); top.addWidget(self.btn_disc)

            # 角度
            self.x_spin = QSpinBox(); self.x_spin.setRange(-180, 180); self.x_spin.setSingleStep(1)
            self.y_spin = QSpinBox(); self.y_spin.setRange(-180, 180); self.y_spin.setSingleStep(1)
            grp = QGroupBox("角度 (°)")
            gl = QHBoxLayout(grp); gl.addWidget(QLabel("X：")); gl.addWidget(self.x_spin); gl.addSpacing(12)
            gl.addWidget(QLabel("Y：")); gl.addWidget(self.y_spin); gl.addStretch(1)

            self.btn_apply = QPushButton("套用"); self.btn_zero = QPushButton("歸零")
            self.btn_apply.clicked.connect(self.on_apply); self.btn_zero.clicked.connect(self.on_zero)
            row = QHBoxLayout(); row.addWidget(self.btn_apply); row.addWidget(self.btn_zero); row.addStretch(1)

            # 底部：Arduino 輸出
            out_grp = QGroupBox("Arduino 輸出")
            vg = QVBoxLayout(out_grp)
            self.out_view = QTextEdit(); self.out_view.setReadOnly(True)
            vg.addWidget(self.out_view)

            root = QVBoxLayout(self); root.addLayout(top); root.addWidget(grp); root.addLayout(row); root.addWidget(out_grp, 1)

            self._ser = None; self._cur_port = ""; self._rxbuf = b""
            self._rx_timer = QTimer(self); self._rx_timer.timeout.connect(self._poll_rx); self._rx_timer.start(50)

        # connect
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
                self._log(f"[Connected] {port} @ {self.BAUD}")
                self.setWindowTitle(f"Aim 視窗（{port}）")
            except Exception as e:
                self._log(f"[連線失敗] {e}")

        def _disconnect(self):
            if self._ser:
                try: self._ser.close()
                except Exception: pass
            self._ser = None; self._cur_port = ""; self._rxbuf = b""
            self.btn_conn.setEnabled(True); self.btn_disc.setEnabled(False)
            self._log("[Disconnected]")
            self.setWindowTitle("Aim 視窗（未連線）")

        # io
        def _write_line(self, s: str):
            if not self._ser: self._log("尚未連線 Arduino"); return
            try:
                self._ser.write((s+"\n").encode("utf-8")); self._log(f">> {s}")
            except Exception as e:
                self._log(f"[TX 失敗] {e}")

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

        # actions
        def on_apply(self):
            x = int(self.x_spin.value()); y = int(self.y_spin.value())
            self._write_line(f"1,{x}"); self._write_line(f"2,{y}")
            if self._cur_port:
                self.setWindowTitle(f"Aim 視窗（{self._cur_port}）  X={x}  Y={y}")

        def on_zero(self):
            self.x_spin.setValue(0); self.y_spin.setValue(0); self.on_apply()

        def closeEvent(self, e):
            try:
                if self._ser: self._ser.close()
            except Exception: pass
            super().closeEvent(e)
except Exception:
    pass
