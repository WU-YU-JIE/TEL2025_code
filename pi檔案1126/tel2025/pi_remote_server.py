# pi_remote_server.py
# -*- coding: utf-8 -*-
import socket
import threading
import multiprocessing as mp
import time
import queue

from mega1_controller import serial_worker, MODE_CHASSIS, MODE_SHOOTER, DEFAULT_BAUD
from aim import SerialLink, BAUD as AIM_BAUD

# === 設定區 ===
MEGA1_PORT = "/dev/ttyUSB0"   # 底盤控制板
AIM_PORT   = "/dev/ttyACM0"   # 瞄準控制板 (請確認你的實際 Port)
TCP_HOST   = "0.0.0.0"
TCP_PORT   = 5000

# === 全域連線管理 (用於主動回傳資料給 PC) ===
CURRENT_CONN = None
SEND_LOCK = threading.Lock()

def send_to_pc(msg):
    """將訊息轉送給目前連線的 PC"""
    global CURRENT_CONN
    with SEND_LOCK:
        if CURRENT_CONN:
            try:
                if not msg.endswith("\n"): msg += "\n"
                CURRENT_CONN.sendall(msg.encode("utf-8"))
            except Exception:
                CURRENT_CONN = None 

# === 背景執行緒：讀取 AIM Arduino 回傳 ===
def aim_read_loop(link):
    print("[AIM] 啟動背景讀取執行緒...")
    while True:
        try:
            if link.ser and link.ser.is_open and link.ser.in_waiting:
                line = link.ser.readline().decode("utf-8", errors="replace").strip()
                if not line: continue

                if line.startswith("degree:"):
                    send_to_pc(line) 
                else:
                    send_to_pc(f"AIM_LOG:{line}")
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[AIM] Read Error: {e}")
            time.sleep(1.0)

# === 背景執行緒：讀取 MEGA 底盤回傳 ===
def mega_log_loop(log_q):
    print("[MEGA] 啟動 Log 轉送執行緒...")
    while True:
        try:
            msg = log_q.get(timeout=0.1)
            send_to_pc(f"MEGA_LOG:{msg}")
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[MEGA] Log Error: {e}")

# === 【新增】本地 IPC 監聽執行緒 (接收 pi_detect 的指令) ===
def local_ipc_loop(ipc_queue, aim_link):
    """監聽來自 pi_detect 的 Queue 指令"""
    if ipc_queue is None:
        return

    print("[IPC] 啟動本地指令監聽 (Queue)...")
    while True:
        try:
            # 這是阻塞式讀取，不會吃 CPU 資源
            cmd_str = ipc_queue.get()
            
            # 解析指令 (格式: "AIM_RAW left,1000")
            if cmd_str.startswith("AIM_RAW"):
                raw_cmd = cmd_str[8:].strip()
                if aim_link.is_open():
                    aim_link.ser.write((raw_cmd + "\n").encode("utf-8"))
                    # 可以在這裡 print 來 debug
                    # print(f"[IPC] 執行: {raw_cmd}")
                else:
                    print("[IPC] 錯誤: AIM Serial 未連線")
            
        except Exception as e:
            print(f"[IPC] Error: {e}")

# === TCP Client 處理 (維持原樣，供 PC 連線) ===
def handle_client(conn, addr, cmd_q, aim_link):
    global CURRENT_CONN
    print(f"[CLIENT] {addr} 連線進來")
    with SEND_LOCK:
        CURRENT_CONN = conn

    mode_current = MODE_CHASSIS
    def set_mode(new_mode, force=False):
        nonlocal mode_current
        if not force and mode_current == new_mode: return
        mode_current = new_mode
        cmd_q.put({"type": "set_mode", "mode": new_mode})

    f = conn.makefile("r")
    try:
        for line in f:
            line = line.strip()
            if not line: continue
            parts = line.split()
            cmd = parts[0].upper()
            try:
                if cmd == "CHASSIS":
                    if len(parts) != 4: continue
                    vx, vy, wz = float(parts[1]), float(parts[2]), float(parts[3])
                    set_mode(MODE_CHASSIS)
                    cmd_q.put({"type": "velocity", "vx": vx, "vy": vy, "wz": wz})
                    conn.sendall(b"OK CHASSIS\n")
                elif cmd == "MODE":
                    if len(parts) != 2: continue
                    tgt = parts[1].upper()
                    if tgt in (MODE_CHASSIS, MODE_SHOOTER):
                        set_mode(tgt, force=True)
                        conn.sendall(b"OK MODE\n")
                elif cmd == "STEP":
                    if len(parts) != 2: continue
                    sub = parts[1].upper()
                    set_mode(MODE_SHOOTER)
                    cmd_q.put({"type": "raw", "text": sub})
                    conn.sendall(b"OK STEP\n")
                elif cmd == "SERVO":
                    if len(parts) != 2: continue
                    sub = parts[1].upper()
                    set_mode(MODE_SHOOTER)
                    cmd_q.put({"type": "raw", "text": sub})
                    conn.sendall(b"OK SERVO\n")
                elif cmd == "NEO":
                    if len(parts) != 2: continue
                    val = int(parts[1])
                    set_mode(MODE_SHOOTER)
                    cmd_q.put({"type": "raw", "text": str(val)})
                    conn.sendall(b"OK NEO\n")
                elif cmd == "AIM":
                    if len(parts) != 3: continue
                    axis = int(parts[1])
                    angle = float(parts[2])
                    aim_link.send_angle(axis, angle)
                    conn.sendall(b"OK AIM\n")
                elif cmd == "AIM_RAW":
                    if len(parts) < 2: continue
                    raw_cmd = line[8:].strip()
                    if aim_link.is_open():
                        aim_link.ser.write((raw_cmd + "\n").encode("utf-8"))
                        conn.sendall(b"OK AIM_RAW\n")
                    else:
                        conn.sendall(b"ERR AIM CLOSED\n")
                elif cmd == "RAW":
                    if len(parts) < 2: continue
                    txt = line[4:].strip()
                    cmd_q.put({"type": "raw", "text": txt})
                    conn.sendall(b"OK RAW\n")
                elif cmd == "STATUS":
                    conn.sendall(f"MODE={mode_current}\n".encode("utf-8"))
                elif cmd == "QUIT":
                    break
                else:
                    conn.sendall(b"ERR UNKNOWN\n")
            except Exception as e:
                conn.sendall(f"ERR EXEC: {e}\n".encode("utf-8"))
    finally:
        print(f"[CLIENT] {addr} 離線")
        with SEND_LOCK:
            if CURRENT_CONN == conn:
                CURRENT_CONN = None
        conn.close()

def start_mega_worker():
    cmd_q = mp.Queue(maxsize=500)
    log_q = mp.Queue(maxsize=1000)
    p = mp.Process(target=serial_worker, args=(cmd_q, log_q), daemon=True)
    p.start()
    cmd_q.put({"type": "open", "port": MEGA1_PORT, "baud": DEFAULT_BAUD})
    cmd_q.put({"type": "set_interval", "interval": 0.08})
    return cmd_q, log_q, p

def start_aim_link():
    link = SerialLink()
    try:
        link.open(AIM_PORT, AIM_BAUD)
    except Exception as e:
        print(f"[WARN] AIM 連線失敗: {e}")
    return link

# === 修改 main 接收 queue 參數 ===
def main(ipc_queue=None):
    mp.freeze_support()

    print("[INFO] 啟動 MEGA1 ...")
    cmd_q, log_q, worker = start_mega_worker()

    print("[INFO] 啟動 AIM ...")
    aim_link = start_aim_link()

    t1 = threading.Thread(target=aim_read_loop, args=(aim_link,), daemon=True)
    t1.start()

    t2 = threading.Thread(target=mega_log_loop, args=(log_q,), daemon=True)
    t2.start()

    # ★ 啟動本地 IPC 監聽 (如果有傳入 queue)
    if ipc_queue:
        t3 = threading.Thread(target=local_ipc_loop, args=(ipc_queue, aim_link), daemon=True)
        t3.start()

    print(f"[INFO] TCP Server 監聽 {TCP_HOST}:{TCP_PORT}")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_HOST, TCP_PORT))
    srv.listen(5)

    try:
        while True:
            conn, addr = srv.accept()
            th = threading.Thread(
                target=handle_client,
                args=(conn, addr, cmd_q, aim_link),
                daemon=True
            )
            th.start()
    finally:
        srv.close()
        cmd_q.put({"type": "quit"})
        aim_link.close()

if __name__ == "__main__":
    main()