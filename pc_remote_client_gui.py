# pc_remote_client_gui.py
# 筆電端遙控程式：
# - 透過 TCP 連到 Raspberry Pi 上的 pi_remote_server.py
# - 底盤：W / A / S / D + 左右方向鍵 控制 CHASSIS vx vy wz
# - Shooter：F / R / X 控 STEP F/R/S，1 / 2 / 3 控 SERVO S1/S2/S3，NEO 速度輸入
# - AIM：
#     L / J 控 M1 (軸1) ±1°
#     I / K 控 M2 (軸2) ±1°
#   並提供：
#     M1 設為原點 / M1 回原點
#     M2 設為原點 / M2 回原點
#   預設角度：
#     9 個檔案槽 → presets/preset_1.json ~ preset_9.json
#     10 個參數按鈕 → 每檔第 1~10 筆 [M1,M2]
#     按 P 鍵：直接載入 + 立即送出（若已連線）

import socket
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import os
import json

DEFAULT_HOST = "10.254.97.208"
DEFAULT_PORT = 5000
PRESET_DIR   = "presets"   # JSON 放這裡（與此程式同層的 presets 資料夾）


class RemoteClient:
    def __init__(self):
        self.sock = None
        self.lock = threading.Lock()
        self.running = False
        self.on_recv = None  # callback(str)

    def connect(self, host, port):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
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
            if self.on_recv:
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
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
            self.sock = None
            self.running = False


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Laptop Remote Controller -> Pi")
        self.geometry("820x760")
        self.resizable(False, False)

        self.client = RemoteClient()
        self.client.on_recv = self._on_recv

        # 狀態
        self.connected = False
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        self.speed_scale = tk.DoubleVar(value=0.6)  # 底盤速度倍率 (0~1)

        # 底盤鍵盤狀態
        self.keys = {
            "w": False,
            "a": False,
            "s": False,
            "d": False,
            "left": False,
            "right": False
        }
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Shooter
        self.neo_var = tk.StringVar(value="0")
        self.raw_var = tk.StringVar(value="STATUS")

        # AIM 狀態（手動）
        self.AIM_STEP = 1.0
        self.m1_angle = 0.0
        self.m2_angle = 0.0

        # AIM 預設選擇
        self.current_file_slot   = None  # 1..9
        self.current_param_slot  = None  # 1..10

        self._ensure_preset_dir()
        self._build_ui()
        self._bind_keys()

    # ===== 共用 UI =====
    def _build_ui(self):
        frm = ttk.Frame(self, padding=8)
        frm.pack(fill=tk.BOTH, expand=True)

        row = 0
        # 連線區
        ttk.Label(frm, text="Pi IP：").grid(column=0, row=row, sticky=tk.W)
        ttk.Entry(frm, textvariable=self.host_var, width=20).grid(column=1, row=row, sticky=tk.W)

        ttk.Label(frm, text="Port：").grid(column=2, row=row, sticky=tk.W)
        ttk.Entry(frm, textvariable=self.port_var, width=8).grid(column=3, row=row, sticky=tk.W)

        self.btn_connect = ttk.Button(frm, text="連線", command=self._toggle_connect)
        self.btn_connect.grid(column=4, row=row, padx=8, sticky=tk.W)

        self.status_var = tk.StringVar(value="未連線")
        ttk.Label(frm, textvariable=self.status_var).grid(column=5, row=row, sticky=tk.W)

        row += 1
        # 底盤控制說明
        ttk.Label(
            frm,
            text="底盤：W/S 前後  A/D 左右  ←/→ 旋轉    速度倍率："
        ).grid(column=0, row=row, columnspan=4, sticky=tk.W, pady=(8, 0))

        ttk.Scale(
            frm, from_=0.0, to=1.0, orient=tk.HORIZONTAL,
            variable=self.speed_scale, length=200,
            command=lambda v: self._update_speed_label()
        ).grid(column=4, row=row, columnspan=2, sticky=tk.W, pady=(8, 0))

        self.sp_lbl = ttk.Label(frm, text=f"{self.speed_scale.get():.2f}")
        self.sp_lbl.grid(column=6, row=row, sticky=tk.W, pady=(8, 0))

        row += 1
        # 目前 vx, vy, wz
        cur = ttk.LabelFrame(frm, text="目前底盤指令")
        cur.grid(column=0, row=row, columnspan=7, sticky=tk.EW, pady=(8, 4))
        self.vx_lbl = ttk.Label(cur, text="vx = 0.00")
        self.vy_lbl = ttk.Label(cur, text="vy = 0.00")
        self.wz_lbl = ttk.Label(cur, text="wz = 0.00")
        self.vx_lbl.grid(column=0, row=0, padx=8, pady=4, sticky=tk.W)
        self.vy_lbl.grid(column=1, row=0, padx=8, pady=4, sticky=tk.W)
        self.wz_lbl.grid(column=2, row=0, padx=8, pady=4, sticky=tk.W)

        row += 1
        # Shooter（步進 + Servo + NEO）
        stp = ttk.LabelFrame(frm, text="Shooter 控制")
        stp.grid(column=0, row=row, columnspan=7, sticky=tk.EW, pady=(4, 4))

        ttk.Label(stp, text="步進 (STEP)：F / R / X=停").grid(column=0, row=0, padx=8, pady=4, sticky=tk.W)
        ttk.Button(stp, text="F", command=lambda: self._send_step("F")).grid(column=1, row=0, padx=4, pady=4)
        ttk.Button(stp, text="R", command=lambda: self._send_step("R")).grid(column=2, row=0, padx=4, pady=4)
        ttk.Button(stp, text="S", command=lambda: self._send_step("S")).grid(column=3, row=0, padx=4, pady=4)

        ttk.Label(stp, text="Servo：1 / 2 / 3 -> S1/S2/S3").grid(column=0, row=1, padx=8, pady=4, sticky=tk.W)
        ttk.Button(stp, text="S1", command=lambda: self._send_servo("S1")).grid(column=1, row=1, padx=4, pady=4)
        ttk.Button(stp, text="S2", command=lambda: self._send_servo("S2")).grid(column=2, row=1, padx=4, pady=4)
        ttk.Button(stp, text="S3", command=lambda: self._send_servo("S3")).grid(column=3, row=1, padx=4, pady=4)

        ttk.Label(stp, text="NEO 速度 (-100..100)：").grid(column=0, row=2, padx=8, pady=4, sticky=tk.W)
        ttk.Entry(stp, textvariable=self.neo_var, width=8).grid(column=1, row=2, padx=4, pady=4, sticky=tk.W)
        ttk.Button(stp, text="送出 NEO", command=self._send_neo).grid(column=2, row=2, padx=4, pady=4, sticky=tk.W)

        row += 1
        # AIM 手動控制
        aimf = ttk.LabelFrame(frm, text="AIM 控制（L/J 控 M1，I/K 控 M2，每步 1°）")
        aimf.grid(column=0, row=row, columnspan=7, sticky=tk.EW, pady=(4, 4))

        self.m1_lbl = ttk.Label(aimf, text="M1 = 0.0°")
        self.m2_lbl = ttk.Label(aimf, text="M2 = 0.0°")
        self.m1_lbl.grid(column=0, row=0, padx=8, pady=4, sticky=tk.W)
        self.m2_lbl.grid(column=0, row=1, padx=8, pady=4, sticky=tk.W)

        ttk.Button(aimf, text="M1 -", command=lambda: self._aim_delta(1, -self.AIM_STEP)).grid(column=1, row=0, padx=4, pady=4)
        ttk.Button(aimf, text="M1 +", command=lambda: self._aim_delta(1,  self.AIM_STEP)).grid(column=2, row=0, padx=4, pady=4)
        ttk.Button(aimf, text="M1 設為原點", command=lambda: self._aim_reset_origin(1)).grid(column=3, row=0, padx=4, pady=4)
        ttk.Button(aimf, text="M1 回原點", command=lambda: self._aim_go_origin(1)).grid(column=4, row=0, padx=4, pady=4)

        ttk.Button(aimf, text="M2 -", command=lambda: self._aim_delta(2, -self.AIM_STEP)).grid(column=1, row=1, padx=4, pady=4)
        ttk.Button(aimf, text="M2 +", command=lambda: self._aim_delta(2,  self.AIM_STEP)).grid(column=2, row=1, padx=4, pady=4)
        ttk.Button(aimf, text="M2 設為原點", command=lambda: self._aim_reset_origin(2)).grid(column=3, row=1, padx=4, pady=4)
        ttk.Button(aimf, text="M2 回原點", command=lambda: self._aim_go_origin(2)).grid(column=4, row=1, padx=4, pady=4)

        self._update_aim_labels()

        row += 1
        # AIM 預設角度（9 檔案槽 + 10 參數）
        pf = ttk.LabelFrame(frm, text="AIM 預設角度（9 檔案槽 + 10 參數；preset_1~preset_9.json，每筆 [M1,M2]）")
        pf.grid(column=0, row=row, columnspan=7, sticky=tk.NSEW, pady=(4, 4))

        file_frame = ttk.Frame(pf)
        file_frame.pack(fill="x", pady=(2, 2))
        self.file_buttons = []
        for i in range(9):
            idx = i + 1
            b = ttk.Button(
                file_frame, text=f"槽{idx}",
                command=lambda s=idx: self._select_file_slot(s),
                width=6
            )
            b.grid(row=i // 3, column=i % 3, padx=2, pady=2)
            self.file_buttons.append(b)

        self.file_info_var  = tk.StringVar(value="未選擇檔案槽（預設檔名 preset_1.json~preset_9.json）")
        self.param_info_var = tk.StringVar(value="未選擇參數編號")
        ttk.Label(pf, textvariable=self.file_info_var).pack(anchor="w", padx=4)
        ttk.Label(pf, textvariable=self.param_info_var).pack(anchor="w", padx=4, pady=(0, 2))

        param_frame = ttk.Frame(pf)
        param_frame.pack(fill="x", pady=(2, 2))
        self.param_buttons = []
        for j in range(10):
            idx = j + 1
            b = ttk.Button(
                param_frame, text=f"P{idx}",
                command=lambda k=idx: self._param_click(k),
                width=6
            )
            b.grid(row=j // 5, column=j % 5, padx=2, pady=2)
            self.param_buttons.append(b)

        row += 1
        # Raw 指令區
        raw = ttk.LabelFrame(frm, text="Raw 指令（直接傳給 Pi）")
        raw.grid(column=0, row=row, columnspan=7, sticky=tk.EW, pady=(4, 4))
        ttk.Entry(raw, textvariable=self.raw_var, width=50).grid(column=0, row=0, padx=4, pady=4)
        ttk.Button(raw, text="送出 RAW", command=self._send_raw).grid(column=1, row=0, padx=4, pady=4)
        ttk.Button(raw, text="STATUS", command=self._send_status).grid(column=2, row=0, padx=4, pady=4)

        row += 1
        # Log
        lf = ttk.LabelFrame(frm, text="Log (Pi 回覆 & 狀態)")
        lf.grid(column=0, row=row, columnspan=7, sticky=tk.NSEW, pady=(8, 0))
        self.log = scrolledtext.ScrolledText(lf, height=10, width=94, state="disabled", wrap="none")
        self.log.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _bind_keys(self):
        self.bind("<KeyPress>", self._on_key_press)
        self.bind("<KeyRelease>", self._on_key_release)
        self.after(200, lambda: self.focus_force())

    def _ensure_preset_dir(self):
        if not os.path.isdir(PRESET_DIR):
            try:
                os.makedirs(PRESET_DIR, exist_ok=True)
            except Exception:
                pass

    # ===== 連線控制 =====
    def _toggle_connect(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        host = self.host_var.get().strip()
        port = int(self.port_var.get())
        if not host:
            messagebox.showwarning("提示", "請輸入 Pi IP")
            return
        try:
            self.client.connect(host, port)
        except Exception as e:
            messagebox.showerror("連線失敗", str(e))
            self.status_var.set("連線失敗")
            return
        self.connected = True
        self.status_var.set(f"已連線：{host}:{port}")
        self.btn_connect.config(text="斷線")
        self._append_log("[INFO] connected")

    def _disconnect(self):
        self.client.close()
        self.connected = False
        self.status_var.set("未連線")
        self.btn_connect.config(text="連線")
        self._append_log("[INFO] disconnected")

    # ===== Log =====
    def _append_log(self, txt):
        self.log.config(state="normal")
        self.log.insert(tk.END, txt + "\n")
        self.log.see(tk.END)
        self.log.config(state="disabled")

    def _on_recv(self, line: str):
        self.after(0, lambda: self._append_log("[Pi] " + line))

    # ===== 底盤控制 =====
    def _update_speed_label(self):
        self.sp_lbl.config(text=f"{self.speed_scale.get():.2f}")
        self._update_chassis_from_keys()

    def _on_key_press(self, ev):
        if not self.connected:
            return
        k = ev.keysym.lower()

        # 底盤
        if k in ("w", "a", "s", "d", "left", "right"):
            if not self.keys.get(k, False):
                self.keys[k] = True
                self._update_chassis_from_keys()
            return

        # Shooter
        if k == "f":
            self._send_step("F"); return
        if k == "r":
            self._send_step("R"); return
        if k == "x":
            self._send_step("S"); return

        # Servo
        if k == "1":
            self._send_servo("S1"); return
        if k == "2":
            self._send_servo("S2"); return
        if k == "3":
            self._send_servo("S3"); return

        # AIM 手動
        if k == "l":
            self._aim_delta(1, self.AIM_STEP); return
        if k == "j":
            self._aim_delta(1, -self.AIM_STEP); return
        if k == "i":
            self._aim_delta(2, self.AIM_STEP); return
        if k == "k":
            self._aim_delta(2, -self.AIM_STEP); return

    def _on_key_release(self, ev):
        if not self.connected:
            return
        k = ev.keysym.lower()
        if k in ("w", "a", "s", "d", "left", "right"):
            if self.keys.get(k, False):
                self.keys[k] = False
                self._update_chassis_from_keys()

    def _update_chassis_from_keys(self):
        vx = vy = wz = 0.0
        if self.keys["w"] and not self.keys["s"]:
            vx = 1.0
        elif self.keys["s"] and not self.keys["w"]:
            vx = -1.0

        if self.keys["d"] and not self.keys["a"]:
            vy = 1.0
        elif self.keys["a"] and not self.keys["d"]:
            vy = -1.0

        if self.keys["left"] and not self.keys["right"]:
            wz = -1.0
        elif self.keys["right"] and not self.keys["left"]:
            wz = 1.0

        sp = float(self.speed_scale.get())
        self.vx = vx * sp
        self.vy = vy * sp
        self.wz = wz * sp

        self.vx_lbl.config(text=f"vx = {self.vx:.2f}")
        self.vy_lbl.config(text=f"vy = {self.vy:.2f}")
        self.wz_lbl.config(text=f"wz = {self.wz:.2f}")

        if self.connected:
            try:
                cmd = f"CHASSIS {self.vx:.2f} {self.vy:.2f} {self.wz:.2f}"
                self.client.send_line(cmd)
                self._append_log("[PC] " + cmd)
            except Exception as e:
                self._append_log(f"[ERR] send CHASSIS: {e}")
                self._disconnect()

    # ===== Shooter / RAW =====
    def _send_step(self, sub):
        if not self.connected:
            return
        cmd = f"STEP {sub}"
        try:
            self.client.send_line(cmd)
            self._append_log("[PC] " + cmd)
        except Exception as e:
            self._append_log(f"[ERR] send STEP: {e}")
            self._disconnect()

    def _send_servo(self, sname):
        if not self.connected:
            return
        cmd = f"SERVO {sname}"
        try:
            self.client.send_line(cmd)
            self._append_log("[PC] " + cmd)
        except Exception as e:
            self._append_log(f"[ERR] send SERVO: {e}")
            self._disconnect()

    def _send_neo(self):
        if not self.connected:
            return
        s = self.neo_var.get().strip()
        try:
            v = int(s)
        except:
            messagebox.showwarning("警告", "NEO 速度必須為整數 -100..100")
            return
        if v < -100: v = -100
        if v > 100:  v = 100
        cmd = f"NEO {v}"
        try:
            self.client.send_line(cmd)
            self._append_log("[PC] " + cmd)
        except Exception as e:
            self._append_log(f"[ERR] send NEO: {e}")
            self._disconnect()

    def _send_raw(self):
        if not self.connected:
            return
        txt = self.raw_var.get().strip()
        if not txt:
            return
        cmd = f"RAW {txt}"
        try:
            self.client.send_line(cmd)
            self._append_log("[PC] " + cmd)
        except Exception as e:
            self._append_log(f"[ERR] send RAW: {e}")
            self._disconnect()

    def _send_status(self):
        if not self.connected:
            return
        try:
            self.client.send_line("STATUS")
            self._append_log("[PC] STATUS")
        except Exception as e:
            self._append_log(f"[ERR] send STATUS: {e}")
            self._disconnect()

    # ===== AIM 手動 =====
    def _update_aim_labels(self):
        self.m1_lbl.config(text=f"M1 = {self.m1_angle:.1f}°")
        self.m2_lbl.config(text=f"M2 = {self.m2_angle:.1f}°")

    def _aim_send(self, axis, angle):
        if not self.connected:
            return
        cmd = f"AIM {axis} {angle:.3f}"
        try:
            self.client.send_line(cmd)
            self._append_log("[PC] " + cmd)
        except Exception as e:
            self._append_log(f"[ERR] send AIM: {e}")
            self._disconnect()

    def _aim_delta(self, axis, delta):
        if axis == 1:
            self.m1_angle += delta
            angle = self.m1_angle
        else:
            self.m2_angle += delta
            angle = self.m2_angle
        self._update_aim_labels()
        self._aim_send(axis, angle)

    def _aim_reset_origin(self, axis):
        if axis == 1:
            self.m1_angle = 0.0
        else:
            self.m2_angle = 0.0
        self._update_aim_labels()
        self._append_log(f"[INFO] AIM M{axis} 設為原點 (UI 角度歸零，不送命令)")

    def _aim_go_origin(self, axis):
        if axis == 1:
            self.m1_angle = 0.0
        else:
            self.m2_angle = 0.0
        self._update_aim_labels()
        self._aim_send(axis, 0.0)
        self._append_log(f"[INFO] AIM M{axis} 回原點 (送 AIM {axis} 0)")

    # ===== AIM 預設：檔案槽 + 參數 =====
    def _select_file_slot(self, slot):
        self.current_file_slot = slot
        self.current_param_slot = None
        fname = f"preset_{slot}.json"
        path = os.path.join(PRESET_DIR, fname)
        if os.path.exists(path):
            self.file_info_var.set(f"檔案槽 {slot} → {fname}")
        else:
            self.file_info_var.set(f"檔案槽 {slot} → {fname}（檔案不存在）")
        self.param_info_var.set("未選擇參數編號")

    def _load_preset_value(self, file_slot, param_slot, verbose=True):
        fname = f"preset_{file_slot}.json"
        path = os.path.join(PRESET_DIR, fname)
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            if verbose:
                messagebox.showwarning("讀取 JSON 失敗", f"{path}\n{e}")
            return None

        if not isinstance(data, list):
            if verbose:
                messagebox.showwarning("JSON 格式錯誤", f"{path}\n最外層必須為 list。")
            return None

        idx = param_slot - 1
        if idx < 0 or idx >= len(data):
            if verbose:
                messagebox.showwarning("索引錯誤", f"{path}\n沒有第 {param_slot} 筆資料。")
            return None

        item = data[idx]
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            m1, m2 = item[0], item[1]
        elif isinstance(item, dict):
            m1 = item.get("m1", item.get("M1"))
            m2 = item.get("m2", item.get("M2"))
        else:
            if verbose:
                messagebox.showwarning("資料格式錯誤", f"{path}\n第 {param_slot} 筆無法解析。")
            return None

        try:
            m1 = float(m1)
            m2 = float(m2)
        except Exception:
            if verbose:
                messagebox.showwarning("資料格式錯誤", f"{path}\n第 {param_slot} 筆不是數值。")
            return None

        return m1, m2

    def _param_click(self, idx):
        if self.current_file_slot is None:
            messagebox.showwarning("尚未選擇檔案槽", "請先按上面的其中一個『槽1~槽9』。")
            return

        self.current_param_slot = idx
        res = self._load_preset_value(self.current_file_slot, idx, verbose=True)
        if res is None:
            self.param_info_var.set(f"參數編號 P{idx}（讀取失敗）")
            return

        m1, m2 = res
        self.m1_angle = m1
        self.m2_angle = m2
        self._update_aim_labels()
        self.param_info_var.set(f"參數編號 P{idx} → M1={m1:.3f}, M2={m2:.3f}")

        if not self.connected:
            self._append_log(
                f"[INFO] 槽{self.current_file_slot} P{idx} → 更新 UI，未連線：M1={m1:.3f}, M2={m2:.3f}"
            )
            return

        self._aim_send(1, m1)
        self._aim_send(2, m2)
        self._append_log(
            f"[INFO] 槽{self.current_file_slot} P{idx} → 已送出：M1={m1:.3f}, M2={m2:.3f}"
        )

    # ===== 關閉 =====
    def _on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
