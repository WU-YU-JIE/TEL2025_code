#start_system.py
import multiprocessing
import time
import os
import signal
import sys

# 匯入另外兩個模組
import pi_remote_server
import pi_detect

def run_server(ipc_queue):
    """啟動硬體控制伺服器"""
    print("[System] 啟動硬體控制進程...")
    pi_remote_server.main(ipc_queue)

def run_detect(ipc_queue):
    """啟動視覺辨識進程"""
    # 等待一下讓 Server 先把 Serial Port 佔用好
    time.sleep(2.0)
    print("[System] 啟動視覺辨識進程...")
    pi_detect.main(ipc_queue)

if __name__ == "__main__":
    # 建立一個跨進程的佇列 (Queue) 用來傳遞指令
    # 視覺端 put() -> 佇列 -> 控制端 get()
    ipc_queue = multiprocessing.Queue()

    # 建立兩個進程
    p_server = multiprocessing.Process(target=run_server, args=(ipc_queue,))
    p_detect = multiprocessing.Process(target=run_detect, args=(ipc_queue,))

    try:
        p_server.start()
        p_detect.start()

        print("="*40)
        print("系統已啟動！按 Ctrl+C 可一次關閉所有程式")
        print("="*40)

        # 主程式只需等待
        p_server.join()
        p_detect.join()

    except KeyboardInterrupt:
        print("\n[System] 正在關閉系統...")
        p_server.terminate()
        p_detect.terminate()
        p_server.join()
        p_detect.join()
        print("[System] 系統已安全關閉。")