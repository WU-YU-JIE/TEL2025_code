# mega1_controller.py
# Mecanum Serial 遙控器（多進程；GUI 不阻塞；自動切 MODE；穩定版）
# Run: python mega1_controller.py
# pip install pyserial

import time, sys, queue
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial, serial.tools.list_ports

import multiprocessing as mp

# ====== 基本設定 ======
DEFAULT_BAUD   = 115200
SEND_INTERVAL  = 0.08   # 先用 12.5Hz，穩了可改回 0.05
MODE_CHASSIS   = "CHASSIS"
MODE_SHOOTER   = "SHOOTER"

# ====== 串列子程序 ======
def serial_worker(cmd_q: mp.Queue, log_q: mp.Queue):
    ser = None
    port = None; baud = None
    mode = MODE_CHASSIS
    vx = vy = wz = 0.0
    interval = SEND_INTERVAL
    pause_until = 0.0
    next_tx = 0.0
    buf = b""

    def log(txt): 
        try: log_q.put_nowait(txt)
        except queue.Full: pass

    def open_port(p, b):
        nonlocal ser, port, baud
        try:
            if ser: 
                try: ser.close()
                except: pass
            ser = serial.Serial(p, b, timeout=0, write_timeout=0)  # 非阻塞
            port, baud = p, b
            time.sleep(1.5)           # 等 Arduino 自動重置
            try: ser.reset_input_buffer()
            except: pass
            log(f"[OPEN] {p} @ {b}")
            # 同步目前模式
            send_line(f"MODE {mode}")
        except Exception as e:
            ser = None; log(f"[ERROR] open failed: {e}")

    def close_port():
        nonlocal ser
        if ser:
            try: ser.close()
            except: pass
            ser = None
            log("[CLOSED] serial")

    def send_line(s):
        nonlocal pause_until
        if not ser: return
        try:
            if not s.endswith("\n"): s += "\n"
            ser.write(s.encode("ascii"))
            log(f"[SEND] {s.strip()}")
        except Exception as e:
            log(f"[ERROR] write: {e}")

    last_poll = 0.0
    while True:
        # 1) 處理來自 GUI 的指令
        drained = 0
        while drained < 50:
            try:
                msg = cmd_q.get_nowait()
            except queue.Empty:
                break

            t = msg.get("type")
            if t == "quit":
                close_port(); return
            elif t == "open":
                open_port(msg["port"], msg["baud"])
            elif t == "close":
                close_port()
            elif t == "set_interval":
                interval = max(0.01, float(msg["interval"]))
            elif t == "set_mode":
                mode = msg["mode"]
                send_line(f"MODE {mode}")
                pause_until = time.monotonic() + 0.2
            elif t == "velocity":
                vx, vy, wz = float(msg["vx"]), float(msg["vy"]), float(msg["wz"])
            elif t == "raw":
                send_line(msg["text"])
                pause = float(msg.get("pause_ms", 0)) / 1000.0
                if pause > 0: pause_until = time.monotonic() + pause
            drained += 1

        # 2) 讀取序列輸入（非阻塞）
        if ser:
            try:
                n = ser.in_waiting
                if n:
                    buf += ser.read(n)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try: text = line.decode("utf-8", "replace").rstrip("\r")
                        except: text = str(line)
                        log(text)
            except Exception as e:
                log(f"[ERROR] read: {e}")

        # 3) 週期性廣播 vx,vy,wz（只在 CHASSIS 且未暫停）
        now = time.monotonic()
        if ser and mode == MODE_CHASSIS and now >= next_tx and now >= pause_until:
            payload = f"{vx:.2f},{vy:.2f},{wz:.2f}\n"
            try:
                ser.write(payload.encode("ascii"))
            except Exception as e:
                log(f"[ERROR] write (bg): {e}")
            next_tx = now + interval

        # 4) 微睡避免吃滿 CPU
        time.sleep(0.002)


# ====== GUI 主程序 ======
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mecanum Serial 遙控器（多進程；不阻塞）")
        self.geometry("760x700"); self.resizable(False, False)

        # 與子程序的 IPC 佇列
        self.cmd_q = mp.Queue(maxsize=500)
        self.log_q = mp.Queue(maxsize=1000)
        self.worker = None

        # 狀態
        self.mode_wanted = MODE_CHASSIS
        self.mode_current = None
        self.last_mode_change = 0.0
        self.speed_scale = tk.DoubleVar(value=0.6)
        self._moving_prev = False

        # 鍵盤旗標
        self.k = {"w":False,"a":False,"s":False,"d":False,"left":False,"right":False,"up":False,"down":False}
        self.vx = self.vy = self.wz = 0.0

        # UI
        self._build_ui()
        self._bind_keys()

        # 定時器
        self.after(80,  self._pump_logs)
        self.after(50,  self._ui_tick)

        # 速度調整連按相關
        self._speed_step = 0.05
        self._speed_repeat_job = None
        self._speed_repeat_dir = 0

    # --- UI ---
    def _build_ui(self):
        frm = ttk.Frame(self, padding=8); frm.pack(fill=tk.BOTH, expand=True)

        row=0
        ttk.Label(frm, text="Serial Port：").grid(column=0,row=row,sticky=tk.W)
        self.port_var = tk.StringVar(value="")
        self.port_combo = ttk.Combobox(frm, width=20, textvariable=self.port_var, state="readonly")
        self._rescan_ports(); self.port_combo.grid(column=1,row=row,sticky=tk.W)
        ttk.Button(frm, text="重新掃描", command=self._rescan_ports).grid(column=2,row=row,sticky=tk.W)

        row+=1
        ttk.Label(frm, text="Baud Rate：").grid(column=0,row=row,sticky=tk.W)
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        ttk.Entry(frm, textvariable=self.baud_var, width=10).grid(column=1,row=row,sticky=tk.W)

        row+=1
        ttk.Label(frm, text="送訊間隔 (s)：").grid(column=0,row=row,sticky=tk.W)
        self.interval_var = tk.DoubleVar(value=SEND_INTERVAL)
        ttk.Entry(frm, textvariable=self.interval_var, width=8).grid(column=1,row=row,sticky=tk.W)
        ttk.Button(frm, text="套用間隔", command=self._apply_interval).grid(column=2,row=row,sticky=tk.W)

        row+=1
        ttk.Label(frm, text="速度倍率 (Scale)：").grid(column=0,row=row,sticky=tk.W)
        ttk.Scale(frm, from_=0.0,to=1.0,variable=self.speed_scale,orient=tk.HORIZONTAL,length=320,
                  command=lambda v:self._on_scale()).grid(column=1,row=row,columnspan=2,sticky=tk.W)

        row+=1
        self.mode_lbl = ttk.Label(frm, text="控制模式：CHASSIS（自動）")
        self.mode_lbl.grid(column=0,row=row,columnspan=3,sticky=tk.W,pady=(6,6))

        row+=1
        ttk.Label(
            frm,
            text="操作：W/S 前後  A/D 左右  ←/→ 旋轉  ↑/↓ 調速   F/R/S=步進   1/2/3=Servo 60/90/120   NEO=-100..100"
        ).grid(column=0,row=row,columnspan=3,sticky=tk.W)

        row+=1
        btn = ttk.Frame(frm); btn.grid(column=0,row=row,columnspan=3,sticky=tk.W,pady=(6,6))
        ttk.Button(btn, text="啟動", command=self._start_worker).grid(column=0,row=0,padx=6)
        ttk.Button(btn, text="停止", command=self._stop_worker).grid(column=1,row=0,padx=6)
        ttk.Button(btn, text="歸零(vx,vy,wz)", command=self._zero_motion).grid(column=2,row=0,padx=6)
        ttk.Button(btn, text="STATUS", command=lambda:self._send_raw("STATUS", pause_ms=400)).grid(column=3,row=0,padx=6)

        row+=1
        cur = ttk.LabelFrame(frm, text="目前數值（已乘速度倍率）"); cur.grid(column=0,row=row,columnspan=3,sticky=tk.EW)
        self.vx_lbl = ttk.Label(cur,text="vx: 0.00"); self.vx_lbl.grid(column=0,row=0,padx=8,pady=6,sticky=tk.W)
        self.vy_lbl = ttk.Label(cur,text="vy: 0.00"); self.vy_lbl.grid(column=1,row=0,padx=8,pady=6,sticky=tk.W)
        self.wz_lbl = ttk.Label(cur,text="wz: 0.00"); self.wz_lbl.grid(column=2,row=0,padx=8,pady=6,sticky=tk.W)
        self.sp_lbl = ttk.Label(cur,text=f"速度倍率: {self.speed_scale.get():.2f}"); self.sp_lbl.grid(column=3,row=0,padx=8,pady=6,sticky=tk.W)

        row+=1
        stp = ttk.LabelFrame(frm, text="步進控制"); stp.grid(column=0,row=row,columnspan=3,sticky=tk.W+tk.E)
        ttk.Button(stp,text="F",command=lambda:self._stepper("F")).grid(column=0,row=0,padx=6,pady=6)
        ttk.Button(stp,text="R",command=lambda:self._stepper("R")).grid(column=1,row=0,padx=6,pady=6)
        ttk.Button(stp,text="S(抱死)",command=lambda:self._stepper("S")).grid(column=2,row=0,padx=6,pady=6)
        ttk.Label(stp,text="（鍵盤 F / R / X=停）").grid(column=3,row=0,padx=8,pady=6,sticky=tk.W)

        row+=1
        # 新增 Servo 控制區
        srv = ttk.LabelFrame(frm, text="Servo（pin10：S1/S2/S3）")
        srv.grid(column=0,row=row,columnspan=3,sticky=tk.W+tk.E)
        ttk.Button(srv, text="S1 (60°)",  command=lambda:self._servo_angle("S1")).grid(column=0,row=0,padx=6,pady=6)
        ttk.Button(srv, text="S2 (90°)",  command=lambda:self._servo_angle("S2")).grid(column=1,row=0,padx=6,pady=6)
        ttk.Button(srv, text="S3 (120°)", command=lambda:self._servo_angle("S3")).grid(column=2,row=0,padx=6,pady=6)
        ttk.Label(srv,text="（鍵盤 1 / 2 / 3）").grid(column=3,row=0,padx=8,pady=6,sticky=tk.W)

        row+=1
        neo = ttk.LabelFrame(frm, text="NEO 速度（-100..100）"); neo.grid(column=0,row=row,columnspan=3,sticky=tk.W+tk.E)
        self.neo_var = tk.StringVar(value="0")
        ttk.Entry(neo, textvariable=self.neo_var, width=8).grid(column=0,row=0,padx=(6,6),pady=6,sticky=tk.W)
        ttk.Button(neo,text="送出",command=self._send_neo).grid(column=1,row=0,padx=(0,6),pady=6,sticky=tk.W)
        ttk.Label(neo,text="（輸入後按 Enter 也可送）").grid(column=2,row=0,padx=6,pady=6,sticky=tk.W)

        row+=1
        self.info_lbl = ttk.Label(frm, text="狀態：未連線"); self.info_lbl.grid(column=0,row=row,sticky=tk.W)
        self.freq_lbl = ttk.Label(frm, text=f"頻率: {1.0/self.interval_var.get():.1f} Hz"); self.freq_lbl.grid(column=1,row=row,sticky=tk.W,padx=12)

        row+=1
        lf = ttk.LabelFrame(frm, text="Serial Log (MEGA -> PC)"); lf.grid(column=0,row=row,columnspan=3,sticky=tk.NSEW,pady=(8,0))
        self.log = scrolledtext.ScrolledText(lf,height=12,width=88,state="disabled",wrap="none")
        self.log.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        row+=1
        raw = ttk.LabelFrame(frm, text="Raw 指令（STATUS / STOP / F / R / S / S1 / S2 / S3 / -100..100 / vx,vy,wz / MODE ...）")
        raw.grid(column=0,row=row,columnspan=3,sticky=tk.EW,pady=(6,0))
        self.raw_var = tk.StringVar(value="STATUS")
        ttk.Entry(raw, textvariable=self.raw_var, width=56).grid(column=0,row=0,padx=(6,6))
        ttk.Button(raw, text="送出 Raw", command=lambda:self._send_raw(self.raw_var.get())).grid(column=1,row=0)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        # Entry 內按 Enter 送 NEO
        self.bind_class("TEntry","<Return>", lambda e:self._send_neo())

    def _rescan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] or ["(none)"]
        self.port_combo["values"] = ports
        if ports and ports[0] != "(none)":
            self.port_var.set(ports[0])

    def _apply_interval(self):
        val = max(0.01, float(self.interval_var.get()))
        self.interval_var.set(val)
        self.freq_lbl.config(text=f"頻率: {1.0/val:.1f} Hz")
        self._send_cmd({"type":"set_interval","interval":val})

    # --- Worker 控制 ---
    def _start_worker(self):
        if self.worker and self.worker.is_alive():
            self._append("[INFO] 已在執行")
            return
        self.worker = mp.Process(target=serial_worker, args=(self.cmd_q, self.log_q), daemon=True)
        self.worker.start()
        self._append("[INFO] worker 啟動")
        # 開 port
        p = self.port_var.get().strip()
        if not p or p == "(none)":
            self._append("[WARN] 請先選擇 Serial Port")
        else:
            self._send_cmd({"type":"open","port":p,"baud":int(self.baud_var.get())})
            self.info_lbl.config(text=f"狀態：連線中 → {p} @ {self.baud_var.get()}")
            # 1.8s 後同步模式
            self.after(1800, lambda:self._ensure_mode(self.mode_wanted, force=True))

    def _stop_worker(self):
        self._send_cmd({"type":"close"})
        self.info_lbl.config(text="狀態：未連線")

    def _on_close(self):
        self._send_cmd({"type":"quit"})
        try:
            if self.worker: self.worker.join(timeout=0.2)
        except: pass
        self.destroy()

    def _send_cmd(self, msg):
        try: self.cmd_q.put_nowait(msg)
        except queue.Full: self._append("[WARN] cmd queue full")

    # --- 動作／鍵盤 ---
    def _bind_keys(self):
        def press(ev):
            k = ev.keysym.lower()
            # 速度調整（↑ / ↓）
            if k == "up":
                self.k["up"] = True
                self._change_speed(self._speed_step)
                self._start_speed_repeat(+1)
                return
            if k == "down":
                self.k["down"] = True
                self._change_speed(-self._speed_step)
                self._start_speed_repeat(-1)
                return

            # 步進快捷鍵（F/R/S=X）
            if k == "f": self._stepper("F"); return
            if k == "r": self._stepper("R"); return
            if k == "x": self._stepper("S"); return

            # Servo 快捷鍵（1/2/3）
            if k == "1": self._servo_angle("S1"); return
            if k == "2": self._servo_angle("S2"); return
            if k == "3": self._servo_angle("S3"); return

            # 移動鍵
            if k in ("w","a","s","d","left","right"):
                self.k[k] = True
                self._update_motion_from_keys()

        def release(ev):
            k = ev.keysym.lower()
            if k in ("up","down"):
                self.k[k] = False
                self._stop_speed_repeat()
                return
            if k in ("w","a","s","d","left","right"):
                self.k[k] = False
                self._update_motion_from_keys()

        self.bind("<KeyPress>",  press)
        self.bind("<KeyRelease>", release)

        # NEO 輸入框按 Enter 直接送出
        self.bind_class("TEntry", "<Return>", lambda e: self._send_neo())

        # 關窗
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # --- 速度倍率連按用 ---
    def _change_speed(self, delta):
        v = float(self.speed_scale.get())
        v = max(0.0, min(1.0, v + delta))
        self.speed_scale.set(v)
        self.sp_lbl.config(text=f"速度倍率: {v:.2f}")

    def _start_speed_repeat(self, direction):
        if self._speed_repeat_job is not None:
            return
        self._speed_repeat_dir = direction
        def _repeat():
            if (self._speed_repeat_dir > 0 and not self.k["up"]) or \
               (self._speed_repeat_dir < 0 and not self.k["down"]):
                self._speed_repeat_job = None
                return
            self._change_speed(self._speed_step * self._speed_repeat_dir)
            self._speed_repeat_job = self.after(120, _repeat)
        self._speed_repeat_job = self.after(400, _repeat)

    def _stop_speed_repeat(self):
        if self._speed_repeat_job is not None:
            self.after_cancel(self._speed_repeat_job)
            self._speed_repeat_job = None

    def _on_scale(self):
        self.sp_lbl.config(text=f"速度倍率: {self.speed_scale.get():.2f}")

    def _update_motion_from_keys(self):
        vx = vy = wz = 0.0
        if self.k["w"] and not self.k["s"]: vx = 1.0
        elif self.k["s"] and not self.k["w"]: vx = -1.0
        if self.k["d"] and not self.k["a"]: vy = 1.0
        elif self.k["a"] and not self.k["d"]: vy = -1.0
        if self.k["left"] and not self.k["right"]: wz = -1.0
        elif self.k["right"] and not self.k["left"]: wz = 1.0

        moving = (vx!=0.0 or vy!=0.0 or wz!=0.0)
        self.vx, self.vy, self.wz = vx, vy, wz

        # 傳給子程序當前速度（子程序自行週期廣播）
        sp = float(self.speed_scale.get())
        self._send_cmd({"type":"velocity","vx":vx*sp,"vy":vy*sp,"wz":wz*sp})

        # 由「靜止→開始動」那一刻切到 CHASSIS
        if moving and not self._moving_prev:
            self._ensure_mode(MODE_CHASSIS)
        self._moving_prev = moving

    def _zero_motion(self):
        self.vx = self.vy = self.wz = 0.0
        self._send_cmd({"type":"velocity","vx":0.0,"vy":0.0,"wz":0.0})

    def _ensure_mode(self, target, force=False):
        now = time.monotonic()
        if not force and self.mode_current == target: return
        if not force and (now - self.last_mode_change) < 0.35: return
        self.last_mode_change = now
        self.mode_wanted = target
        # 樂觀更新 UI，並通知子程序
        self.mode_current = target
        self.mode_lbl.config(text=f"控制模式：{target}（自動）")
        self._send_cmd({"type":"set_mode","mode":target})

    # --- 命令 ---
    def _stepper(self, cmd):
        if cmd not in ("F","R","S"): return
        self._ensure_mode(MODE_SHOOTER)
        self._send_raw(cmd)

    def _servo_angle(self, cmd):
        # cmd: "S1"/"S2"/"S3"
        if cmd not in ("S1","S2","S3"):
            return
        self._ensure_mode(MODE_SHOOTER)
        self._send_raw(cmd)

    def _send_neo(self):
        s = self.neo_var.get().strip()
        try: v = int(s)
        except: 
            self._append("[WARN] NEO 速度需為整數 -100..100"); return
        v = max(-100, min(100, v))
        self._ensure_mode(MODE_SHOOTER)
        self._send_raw(str(v))

    def _send_raw(self, txt, pause_ms=0):
        up = txt.upper().strip()
        # F/R/S/S1/S2/S3 或 純數字 → SHOOTER
        if up in ("F","R","S","S1","S2","S3") or (txt.lstrip("-").isdigit() and up!="MODE"):
            self._ensure_mode(MODE_SHOOTER)
        elif "," in txt and not up.startswith("MODE "):
            # vx,vy,wz 類 → CHASSIS
            self._ensure_mode(MODE_CHASSIS)
        elif up.startswith("MODE "):
            tgt = up.split(None,1)[1].strip()
            if tgt in (MODE_CHASSIS, MODE_SHOOTER):
                self._ensure_mode(tgt, force=True)
        self._send_cmd({"type":"raw","text":txt,"pause_ms":pause_ms})

    # --- 日誌/畫面更新 ---
    def _append(self, txt):
        self.log.configure(state="normal")
        self.log.insert(tk.END, txt + "\n")
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    def _pump_logs(self):
        try:
            while True:
                line = self.log_q.get_nowait()
                self._append(line)
                if line.startswith("[OPEN]"):
                    self.info_lbl.config(text=line.replace("[OPEN]","狀態：已連線"))
                if line.startswith("[CLOSED]"):
                    self.info_lbl.config(text="狀態：未連線")
        except queue.Empty:
            pass
        self.after(80, self._pump_logs)

    def _ui_tick(self):
        sp = float(self.speed_scale.get())
        self.vx_lbl.config(text=f"vx: {self.vx*sp:.2f}")
        self.vy_lbl.config(text=f"vy: {self.vy*sp:.2f}")
        self.wz_lbl.config(text=f"wz: {self.wz*sp:.2f}")
        self.after(50, self._ui_tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
