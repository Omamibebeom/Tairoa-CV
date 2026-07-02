"""
gripper.py —— 氣動夾爪控制(筆電 + Arduino + L298N,透過 USB 序列)

架構:
  筆電 Python(本檔) --USB序列--> Arduino --> L298N --> 兩顆馬達
    L298N OUT1/OUT2 → 充氣馬達(inflate) → 夾爪【關閉/夾取】
    L298N OUT3/OUT4 → 洩氣馬達(deflate) → 夾爪【鬆開/放開】

  Arduino 端請先燒錄 gripper_arduino.ino。
  本檔負責:計時、鎖(同時只驅動一顆馬達)、HOLD_PRESSURE 維壓邏輯;
  Arduino 端只負責收指令、開關 L298N。

  ★ L298N 的 +12V 用外部電源,GND 與 Arduino「共地」。馬達電源不要用 Arduino 供電。

找不到 pyserial 或開埠失敗(例如純測 socket 流程)時,自動切換成「模擬模式」,
只印出動作、不真的送指令,方便先在電腦測流程再接實體。
"""
import time
import threading

# ========================= 序列埠設定 =========================
# Windows: "COM3" / "COM4" ...(裝置管理員或 Arduino IDE「工具→連接埠」可查)
# Linux/樹莓派: "/dev/ttyUSB0" 或 "/dev/ttyACM0"
PORT = "COM6"
BAUD = 115200
SERIAL_TIMEOUT = 0.3    # 讀取逾時(秒)。ACK 只是參考,不能設太長:
                        # 若 Arduino 沒回覆,每個指令會空等滿這個時間。
                        # 之前設 2.0 導致 RELEASE 總耗時 ~7.5s,超過 TM 腳本
                        # 8000ms 讀取窗口,手臂讀不到 $OK 而中断(只夾一個就結束)。
RESET_WAIT = 2.0        # 開埠後等 Arduino 自動重置開機的秒數

USE_ENABLE = False      # True: 用 ENA/ENB 控制出力(需拔 L298N 跳線帽);False: 跳線帽常態致能
SPEED = 1.0             # 馬達出力 0.0~1.0(USE_ENABLE=True 時有效),氣泵通常給滿

# ========================= 動作時間 / 行為 =========================
INFLATE_SEC = 2       # 充氣多久才夾緊(依你的氣泵/夾爪實測調整)
DEFLATE_SEC = 2       # 洩氣多久才確實鬆開

# HOLD_PRESSURE:
#   True  → 夾取後「充氣馬達持續運轉」維持壓力,直到 open() 才停。
#           適合「沒有止回閥、一停就漏氣」的夾爪(搬運途中不會鬆手)。
#   False → 充氣 INFLATE_SEC 秒後就停,靠止回閥或困住的空氣維持夾持。
#           若你發現搬運途中物件會掉,就改成 True。
HOLD_PRESSURE = True

# ========================= 序列後端(自動偵測) =========================
try:
    import serial   # pip install pyserial(注意:套件名是 pyserial,import 是 serial)
    _HAS_SERIAL = True
except Exception as _e:
    _HAS_SERIAL = False
    print(f"[gripper] 找不到 pyserial({_e});進入模擬模式(不驅動實體馬達)")


class _FakeSerial:
    """筆電上沒接 Arduino / 沒裝 pyserial 時的模擬序列埠,只印出寫入內容。"""
    def __init__(self, *a, **k): pass
    def write(self, data): print(f"[gripper][SIM] → {data.decode(errors='ignore').strip()}")
    def readline(self): return b"OK\n"
    def reset_input_buffer(self): pass
    def close(self): pass
    @property
    def is_open(self): return True


class Gripper:
    """氣動夾爪:close()=充氣夾取, open()=洩氣鬆開。
    內含鎖,保證同一時間只會有一顆馬達動作(絕不同時充氣又洩氣)。"""

    def __init__(self, port=PORT, baud=BAUD):
        self._lock = threading.Lock()
        self._sim = not _HAS_SERIAL
        if _HAS_SERIAL:
            try:
                self.ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
                time.sleep(RESET_WAIT)          # 等 Arduino 重置(開埠會觸發 reset)
                self.ser.reset_input_buffer()
                self._send("PING")
            except Exception as e:
                print(f"[gripper] 開啟序列埠 {port} 失敗({e});改用模擬模式")
                self.ser = _FakeSerial()
                self._sim = True
        else:
            self.ser = _FakeSerial()

        # 設定出力(僅在用 GPIO/PWM 控制致能時)
        if USE_ENABLE:
            v = int(max(0.0, min(1.0, SPEED)) * 255)
            self._send(f"SA:{v}")
            self._send(f"SB:{v}")

        self.stop_all()
        mode = "模擬" if self._sim else f"Arduino @ {port}"
        print(f"[gripper] 初始化完成({mode});"
              f"充氣 ION/IOFF,洩氣 DON/DOFF,HOLD_PRESSURE={HOLD_PRESSURE}")

    # ---- 低階:送指令給 Arduino,回傳 Arduino 的回應字串 ----
    def _send(self, cmd):
        line = (cmd + "\n").encode()
        self.ser.write(line)
        try:
            resp = self.ser.readline().decode(errors="ignore").strip()
        except Exception:
            resp = ""
        return resp

    def _inflate_on(self):  self._send("ION")
    def _inflate_off(self): self._send("IOFF")
    def _deflate_on(self):  self._send("DON")
    def _deflate_off(self): self._send("DOFF")

    def stop_all(self):
        """兩顆馬達全停(安全狀態)。"""
        self._send("STOP")

    # ---- 高階:給 arm_server 呼叫(介面與原版相同)----
    def close(self, seconds=INFLATE_SEC):
        """充氣 → 夾爪關閉(夾取)。回傳後代表已夾好。"""
        t0 = time.time()
        with self._lock:
            self._deflate_off()          # 先確保沒在洩氣
            self._inflate_on()           # 開始充氣
            print(f"[gripper] GRIP  充氣 {seconds}s ...")
            time.sleep(seconds)          # 建立夾持壓力
            if not HOLD_PRESSURE:
                self._inflate_off()      # 停泵,靠止回閥/困住的空氣維持
                print("[gripper] GRIP  充氣停止(靠止回閥維持)")
            else:
                print("[gripper] GRIP  持續充氣維持壓力")
        print(f"[gripper] GRIP  總耗時 {time.time()-t0:.2f}s"
              f"(必須遠小於 TM 腳本的讀取逾時)")

    def open(self, seconds=DEFLATE_SEC):
        """洩氣 → 夾爪鬆開(放開)。回傳後代表已放開。"""
        t0 = time.time()
        with self._lock:
            self._inflate_off()          # 先停充氣(HOLD 模式下在此才停)
            self._deflate_on()           # 開始洩氣
            print(f"[gripper] RELEASE 洩氣 {seconds}s ...")
            time.sleep(seconds)
            self._deflate_off()
            print("[gripper] RELEASE 完成")
        print(f"[gripper] RELEASE 總耗時 {time.time()-t0:.2f}s"
              f"(必須遠小於 TM 腳本的讀取逾時)")

    def cleanup(self):
        """程式結束時呼叫:停馬達並關閉序列埠。"""
        try:
            self.stop_all()
        finally:
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
        print("[gripper] 已關閉序列埠")


# 單獨執行可做手動測試:python gripper.py
if __name__ == "__main__":
    g = Gripper()
    try:
        print(">> 測試夾取(close)")
        g.close()
        time.sleep(1.0)
        print(">> 測試鬆開(open)")
        g.open()
    finally:
        g.cleanup()
