# aim.py
# 方向鍵：
#   ← / →  : M1 (- / + 1°)
#   ↑ / ↓  : M2 (+ / - 1°)
# 傳送格式： "1,<angle>\n" 或 "2,<angle>\n"
# 功能重點：可從下拉選單選取 COM，連線/斷線、重新整理可用埠

import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

BAUD     = 115200
STEP_DEG = 1.0  # 每次按鍵變化角度

class SerialLink:
    def __init__(self):
        self.ser = None

    def open(self, port: str, baud: int = BAUD):
        self.close()
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            self.ser = None
            raise e
        # Arduino 連上會 reset，給點時間
        time.sleep(2.0)

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send_angle(self, axis_id: int, angle: float):
        if not self.is_open():
            return
        msg = f"{axis_id},{angle:.3f}\n"
        try:
            self.ser.write(msg.encode("utf-8"))
        except Exception as e:
            # 送失敗就斷線，避免之後一直報錯
            try:
                self.close()
            finally:
                raise e

    def close(self):
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

def list_serial_ports():
    # 回傳 [(顯示字串, 裝置字串), ...]
    items = []
    for p in serial.tools.list_ports.comports():
        display = f"{p.device} — {p.description}"
        items.append((display, p.device))
    # 若沒找到任何裝置，仍允許手動輸入
    return items

class ControllerApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Arrow Controller (選擇 COM：連線後用方向鍵)")
        self.master.geometry("420x180")

        self.link = SerialLink()
        self.m1_angle = 0.0
        self.m2_angle = 0.0

        # ===== 上方：COM 選擇列 =====
        top = ttk.Frame(master, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="連接埠：").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=35)
        self.port_combo.pack(side="left", padx=6)
        self.port_combo.configure(state="normal")  # 可手動輸入

        self.refresh_btn = ttk.Button(top, text="重新整理", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=4)

        self.connect_btn = ttk.Button(top, text="連線", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=4)

        # ===== 中段：角度顯示 =====
        mid = ttk.Frame(master, padding=8)
        mid.pack(fill="x")
        self.lbl1 = ttk.Label(mid, text=f"M1: {self.m1_angle:.3f}°", font=("Consolas", 16))
        self.lbl2 = ttk.Label(mid, text=f"M2: {self.m2_angle:.3f}°", font=("Consolas", 16))
        self.lbl1.pack(pady=2)
        self.lbl2.pack(pady=2)

        # ===== 底部：提示/狀態 =====
        bot = ttk.Frame(master, padding=8)
        bot.pack(fill="x")
        self.hint = ttk.Label(bot, text="←/→ 控 M1 | ↑/↓ 控 M2 | 每次 ±1°", foreground="#444")
        self.hint.pack(side="left")
        self.status_var = tk.StringVar(value="未連線")
        self.status = ttk.Label(bot, textvariable=self.status_var, foreground="#A00")
        self.status.pack(side="right")

        # 綁定方向鍵（只在已連線時動作）
        master.bind_all("<Left>",  self.on_left)
        master.bind_all("<Right>", self.on_right)
        master.bind_all("<Up>",    self.on_up)
        master.bind_all("<Down>",  self.on_down)

        master.protocol("WM_DELETE_WINDOW", self.on_close)

        # 初始化可用埠
        self.refresh_ports()
        # 自動選第一個（若有）
        if self.port_combo["values"]:
            self.port_combo.current(0)

        # 讓視窗取得焦點，方向鍵才會進來
        master.after(150, lambda: master.focus_force())

    def refresh_ports(self):
        items = list_serial_ports()
        display_list = [d for d, _ in items]
        self.port_map = {d: dev for d, dev in items}
        self.port_combo["values"] = display_list
        if not display_list:
            self.status_var.set("找不到可用連接埠，可手動輸入（如 COM7）")
        else:
            self.status_var.set("請選擇連接埠並點『連線』")

    def toggle_connect(self):
        if self.link.is_open():
            self.link.close()
            self.connect_btn.config(text="連線")
            self.status_var.set("已斷線")
            self.status.config(foreground="#A00")
            return

        # 連線
        sel = self.port_var.get().strip()
        if not sel:
            messagebox.showwarning("提示", "請先選擇或輸入連接埠（例如 COM7）")
            return

        # 若選的是下拉顯示字串，轉回實際裝置名稱
        port = self.port_map.get(sel, sel)  # 若沒對應，視為使用者手動輸入
        try:
            self.link.open(port, BAUD)
        except Exception as e:
            messagebox.showerror("連線失敗", f"無法開啟 {port}\n{e}")
            self.status_var.set("連線失敗")
            self.status.config(foreground="#A00")
            return

        self.connect_btn.config(text="斷線")
        self.status_var.set(f"已連線：{port} @ {BAUD}")
        self.status.config(foreground="#0A0")

    # ===== 鍵盤事件 =====
    def on_left(self, _):
        if not self.link.is_open(): return
        self.m1_angle -= STEP_DEG
        self.send_and_update(1, self.m1_angle)

    def on_right(self, _):
        if not self.link.is_open(): return
        self.m1_angle += STEP_DEG
        self.send_and_update(1, self.m1_angle)

    def on_up(self, _):
        if not self.link.is_open(): return
        self.m2_angle += STEP_DEG
        self.send_and_update(2, self.m2_angle)

    def on_down(self, _):
        if not self.link.is_open(): return
        self.m2_angle -= STEP_DEG
        self.send_and_update(2, self.m2_angle)

    def send_and_update(self, axis_id: int, angle: float):
        try:
            self.link.send_angle(axis_id, angle)
        except Exception as e:
            messagebox.showerror("傳送失敗", f"序列傳送失敗，已自動斷線。\n{e}")
            self.connect_btn.config(text="連線")
            self.status_var.set("傳送失敗，已斷線")
            self.status.config(foreground="#A00")
            return

        if axis_id == 1:
            self.lbl1.config(text=f"M1: {self.m1_angle:.3f}°")
        else:
            self.lbl2.config(text=f"M2: {self.m2_angle:.3f}°")

    def on_close(self):
        try:
            self.link.close()
        finally:
            self.master.destroy()

def main():
    root = tk.Tk()
    # Windows 用者：若字體缺，改成 ("Consolas", 16) 以外的等寬字體亦可
    app = ControllerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
