# -*- coding: utf-8 -*-
import sys
from PyQt5.QtWidgets import QApplication

from mega1_controller import ControllerWindow
from aim import AimWindow
from xboxGUI import XboxWindow
from system_status_gui import SystemStatusWindow

def main():
    app = QApplication(sys.argv)

    syswin = SystemStatusWindow()
    ctrl   = ControllerWindow()
    aim    = AimWindow()
    xbox   = XboxWindow()

    # 把 Controller 的狀態與訊息同步到 SystemStatus
    try:
        ctrl.arduino_conn_changed.connect(syswin.update_arduino_status)
        ctrl.message.connect(syswin.append_log)
    except Exception:
        pass

    # 一次顯示所有視窗
    syswin.show()
    ctrl.show()
    aim.show()
    xbox.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
