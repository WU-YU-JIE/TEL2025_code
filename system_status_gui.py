# -*- coding: utf-8 -*-
from PyQt5.QtCore import QTimer, pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QListWidget, QGroupBox, QTextEdit

try:
    import psutil
except Exception:
    psutil = None
try:
    import serial.tools.list_ports as list_ports
except Exception:
    list_ports = None


class SystemStatusWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("System Status / Arduino 狀態")
        self.resize(520, 440)

        self.cpu_label = QLabel("CPU：- %")
        self.mem_label = QLabel("RAM：- %")
        self.ard_label = QLabel("Arduino：未連線")
        self.ard_label.setStyleSheet("color:#dc2626; font-weight:bold;")

        # 系統
        sys_box = QGroupBox("系統資源")
        v = QVBoxLayout(sys_box); v.addWidget(self.cpu_label); v.addWidget(self.mem_label)

        # COM
        com_box = QGroupBox("可用序列埠 (COM)")
        v2 = QVBoxLayout(com_box); self.com_list = QListWidget(); v2.addWidget(self.com_list)

        # Arduino
        ard_box = QGroupBox("Arduino 狀態")
        v3 = QVBoxLayout(ard_box); v3.addWidget(self.ard_label)

        # Log
        log_box = QGroupBox("系統訊息")
        v4 = QVBoxLayout(log_box); self.log = QTextEdit(); self.log.setReadOnly(True); v4.addWidget(self.log)

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        left = QVBoxLayout(); left.addWidget(sys_box); left.addWidget(ard_box)
        top.addLayout(left, 1); top.addWidget(com_box, 1)
        root.addLayout(top, 2); root.addWidget(log_box, 1)

        self.t_sys = QTimer(self); self.t_sys.timeout.connect(self._refresh_system); self.t_sys.start(1000)
        self.t_com = QTimer(self); self.t_com.timeout.connect(self._refresh_coms);  self.t_com.start(2000)
        self._refresh_system(); self._refresh_coms()

    def append_log(self, s: str): self.log.append(s)

    @pyqtSlot(bool, str)
    def update_arduino_status(self, connected: bool, port: str):
        if connected:
            self.ard_label.setText(f"Arduino：已連線（{port}）")
            self.ard_label.setStyleSheet("color:#16a34a; font-weight:bold;")
            self.append_log(f"[Arduino] Connected on {port}")
        else:
            self.ard_label.setText("Arduino：未連線")
            self.ard_label.setStyleSheet("color:#dc2626; font-weight:bold;")
            self.append_log(f"[Arduino] Disconnected{(' from '+port) if port else ''}")

    def _refresh_system(self):
        if not psutil:
            self.cpu_label.setText("CPU：psutil 未安裝"); self.mem_label.setText("RAM：psutil 未安裝"); return
        self.cpu_label.setText(f"CPU：{psutil.cpu_percent():.1f} %")
        self.mem_label.setText(f"RAM：{psutil.virtual_memory().percent:.1f} %")

    def _refresh_coms(self):
        self.com_list.clear()
        if not list_ports:
            self.com_list.addItem("pyserial 未安裝"); return
        for p in list_ports.comports():
            self.com_list.addItem(f"{p.device} | {p.description}")
