"""
gripper.py —— 氣動夾爪控制(樹莓派 GPIO + L298N)

硬體接線(你的配置):
  L298N OUT1, OUT2  →  充氣馬達(inflate)  →  夾爪【關閉 / 夾取】
  L298N OUT3, OUT4  →  洩氣馬達(deflate)  →  夾爪【鬆開 / 放開】

  L298N 控制腳 → 樹莓派 GPIO(BCM 編號):
    充氣(馬達 A, OUT1/2):IN1, IN2 決定方向;ENA 決定開/關(或轉速)
    洩氣(馬達 B, OUT3/4):IN3, IN4 決定方向;ENB 決定開/關(或轉速)

  ★ L298N 的 ENA / ENB:
    - 若你把板子上的 ENA、ENB 「跳線帽(jumper)」留著 → 兩顆馬達永遠全速致能,
      這時 ENA_PIN / ENB_PIN 可不接,程式裡把 USE_ENABLE 設成 False 即可。
    - 若你想用樹莓派控制致能/轉速(建議) → 拔掉跳線帽,把 ENA、ENB 接到
      下面設定的 GPIO,USE_ENABLE 設 True。
  ★ L298N 的 +12V 接馬達電源、GND 與樹莓派 GND「共地」,邏輯 +5V 視模組而定。
    馬達電源「不要」用樹莓派供電,請用外部電源,只把 GND 接在一起。

在筆電(沒有 GPIO)上執行時會自動切換成「模擬模式」,只印出動作、不真的驅動,
方便你先在電腦測 socket 流程,再部署到樹莓派。
"""
import time
import threading

# ========================= 接線設定(BCM 編號) =========================
IN1_PIN, IN2_PIN, ENA_PIN = 17, 27, 12   # 充氣 inflate(OUT1/OUT2)
IN3_PIN, IN4_PIN, ENB_PIN = 22, 23, 13   # 洩氣 deflate(OUT3/OUT4)

USE_ENABLE = False     # True: 用 GPIO 控制 ENA/ENB;False: ENA/ENB 用跳線帽常態致能
SPEED = 1.0           # 馬達出力 0.0~1.0(USE_ENABLE=True 時有效),氣泵通常給滿

# ========================= 動作時間 / 行為 =========================
INFLATE_SEC = 1.5     # 充氣多久才夾緊(依你的氣泵/夾爪實測調整)
DEFLATE_SEC = 1.2     # 洩氣多久才確實鬆開

# HOLD_PRESSURE:
#   True  → 夾取後「充氣馬達持續運轉」維持壓力,直到 RELEASE 才停。
#           適合「沒有止回閥、一停就漏氣」的夾爪(搬運途中不會鬆手)。
#   False → 充氣 INFLATE_SEC 秒後就停,靠止回閥或困住的空氣維持夾持。
#           若你發現搬運途中物件會掉,就改成 True。
HOLD_PRESSURE = True

# ========================= GPIO 後端(自動偵測) =========================
try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
    _HAS_GPIO = True
except Exception as _e:                       # 筆電上沒有 gpiozero → 模擬模式
    _HAS_GPIO = False
    print(f"[gripper] 找不到 gpiozero({_e});進入模擬模式(不驅動實體馬達)")

    class DigitalOutputDevice:                 # 模擬:只記錄狀態
        def __init__(self, pin): self.pin, self.value = pin, 0
        def on(self):  self.value = 1
        def off(self): self.value = 0
        def close(self): pass

    class PWMOutputDevice:                      # 模擬:只記錄數值
        def __init__(self, pin): self.pin, self.value = pin, 0.0
        def close(self): pass


class Gripper:
    """氣動夾爪:close()=充氣夾取, open()=洩氣鬆開。
    內含鎖,保證同一時間只會有一顆馬達動作(絕不同時充氣又洩氣)。"""

    def __init__(self):
        self.in1 = DigitalOutputDevice(IN1_PIN)
        self.in2 = DigitalOutputDevice(IN2_PIN)
        self.in3 = DigitalOutputDevice(IN3_PIN)
        self.in4 = DigitalOutputDevice(IN4_PIN)
        self.ena = PWMOutputDevice(ENA_PIN) if USE_ENABLE else None
        self.enb = PWMOutputDevice(ENB_PIN) if USE_ENABLE else None
        self._lock = threading.Lock()
        self.stop_all()
        mode = "實體 GPIO" if _HAS_GPIO else "模擬"
        print(f"[gripper] 初始化完成({mode});"
              f"充氣 IN1={IN1_PIN},IN2={IN2_PIN},ENA={ENA_PIN} / "
              f"洩氣 IN3={IN3_PIN},IN4={IN4_PIN},ENB={ENB_PIN}")

    # ---- 低階:啟停各馬達 ----
    def _inflate_on(self):
        # 方向:IN1 高、IN2 低。若馬達轉向相反(變成抽氣),把 OUT1/OUT2 對調,
        # 或把這兩行的 on/off 對調即可。
        self.in1.on(); self.in2.off()
        if self.ena is not None: self.ena.value = SPEED

    def _inflate_off(self):
        if self.ena is not None: self.ena.value = 0.0
        self.in1.off(); self.in2.off()

    def _deflate_on(self):
        self.in3.on(); self.in4.off()
        if self.enb is not None: self.enb.value = SPEED

    def _deflate_off(self):
        if self.enb is not None: self.enb.value = 0.0
        self.in3.off(); self.in4.off()

    def stop_all(self):
        """兩顆馬達全停(安全狀態)。"""
        self._inflate_off()
        self._deflate_off()

    # ---- 高階:給 arm_server 呼叫 ----
    def close(self, seconds=INFLATE_SEC):
        """充氣 → 夾爪關閉(夾取)。回傳後代表已夾好。"""
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

    def open(self, seconds=DEFLATE_SEC):
        """洩氣 → 夾爪鬆開(放開)。回傳後代表已放開。"""
        with self._lock:
            self._inflate_off()          # 先停充氣(HOLD 模式下在此才停)
            self._deflate_on()           # 開始洩氣
            print(f"[gripper] RELEASE 洩氣 {seconds}s ...")
            time.sleep(seconds)
            self._deflate_off()
            print("[gripper] RELEASE 完成")

    def cleanup(self):
        """程式結束時呼叫:停馬達並釋放 GPIO。"""
        self.stop_all()
        for d in (self.in1, self.in2, self.in3, self.in4, self.ena, self.enb):
            if d is not None:
                try: d.close()
                except Exception: pass
        print("[gripper] 已釋放 GPIO")


# 單獨執行可做手動測試:python3 gripper.py
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
