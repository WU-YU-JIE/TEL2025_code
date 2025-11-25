# === 追加：可相容 Qt6 .ui 的 Xbox 視窗（載入失敗時不影響原本 run()） ===
import os, io, re
try:
    from PyQt5 import uic
    from PyQt5.QtCore import QTimer, pyqtSignal
    from PyQt5.QtWidgets import QWidget
    import pygame as _pg

    def _fix_qt6_enums(ui_text: str) -> str:
        rep = {
            "Qt::Orientation::Horizontal": "Qt::Horizontal",
            "Qt::Orientation::Vertical": "Qt::Vertical",
            "Qt::AlignmentFlag::AlignLeft": "Qt::AlignLeft",
            "Qt::AlignmentFlag::AlignRight": "Qt::AlignRight",
            "Qt::AlignmentFlag::AlignHCenter": "Qt::AlignHCenter",
            "Qt::AlignmentFlag::AlignVCenter": "Qt::AlignVCenter",
            "Qt::AlignmentFlag::AlignCenter": "Qt::AlignCenter",
            "Qt::CheckState::Unchecked": "Qt::Unchecked",
            "Qt::CheckState::PartiallyChecked": "Qt::PartiallyChecked",
            "Qt::CheckState::Checked": "Qt::Checked",
        }
        for k, v in rep.items():
            ui_text = ui_text.replace(k, v)
        ui_text = re.sub(r'(Qt|QSizePolicy|QFrame|QAbstractItemView|QTabWidget|QToolButton|QDialogButtonBox)::[A-Za-z_]+::',
                         r'\1::', ui_text)
        return ui_text

    class XboxWindow(QWidget):
        axisChanged = pyqtSignal(float, float, float, float, float, float)
        buttonsChanged = pyqtSignal(bool, bool, bool, bool, bool, bool)

        def __init__(self, parent=None, deadzone=0.15):
            super().__init__(parent)
            self._dz = float(deadzone)
            self._status_lbl = None

            # 優先載入 XboxGUI_V2.ui；若失敗→顯示簡易空殼視窗（不影響功能）
            try:
                ui_path = os.path.join(os.path.dirname(__file__), "XboxGUI_V2.ui")
                with open(ui_path, "r", encoding="utf-8") as f:
                    patched = _fix_qt6_enums(f.read())
                uic.loadUi(io.StringIO(patched), self)
                for name in ("labelStatus","statusLabel","infoLabel","lblStatus"):
                    w = getattr(self, name, None)
                    if w is not None:
                        self._status_lbl = w
                        break
            except Exception as e:
                from PyQt5.QtWidgets import QVBoxLayout, QLabel
                self.setWindowTitle("Xbox 視窗")
                lay = QVBoxLayout(self)
                self._status_lbl = QLabel(f"(未載入 .ui：{e})", self)
                lay.addWidget(self._status_lbl)

            self.setWindowTitle("Xbox 視窗")
            _pg.init(); _pg.joystick.init()
            self._js = None
            if _pg.joystick.get_count() > 0:
                self._js = _pg.joystick.Joystick(0); self._js.init()
                self._set_status("已偵測到搖桿 0 號")
            else:
                self._set_status("找不到搖桿（請插上手把）")

            self._timer = QTimer(self)
            self._timer.timeout.connect(self._tick)
            self._timer.start(100)

        def _set_status(self, text: str):
            if self._status_lbl is not None: self._status_lbl.setText(text)
            else: self.setWindowTitle(f"Xbox 視窗 - {text}")

        def _dead(self, x): 
            x=float(x); return 0.0 if abs(x) < 0.15 else x

        def _tick(self):
            for _ in _pg.event.get(): pass
            if not self._js:
                self._set_status("找不到搖桿"); return

            lx = self._dead(self._js.get_axis(0))
            ly = self._dead(self._js.get_axis(1))
            rx = self._dead(self._js.get_axis(2))
            ry = self._dead(self._js.get_axis(3))
            lt = float(self._js.get_axis(4))
            rt = float(self._js.get_axis(5))

            a = bool(self._js.get_button(0)); b = bool(self._js.get_button(1))
            x = bool(self._js.get_button(2)); y = bool(self._js.get_button(3))
            lb = bool(self._js.get_button(4)); rb = bool(self._js.get_button(5))

            self.axisChanged.emit(lx, ly, rx, ry, lt, rt)
            self.buttonsChanged.emit(a, b, x, y, lb, rb)
            self._set_status(f"L({lx:+.2f},{ly:+.2f}) R({rx:+.2f},{ry:+.2f}) LT:{lt:+.2f} RT:{rt:+.2f} "
                             f"A{int(a)} B{int(b)} X{int(x)} Y{int(y)} LB{int(lb)} RB{int(rb)}")
except Exception:
    # 沒有 PyQt5 時，只保留原本的 pygame 版本 run()，完全不受影響
    pass
