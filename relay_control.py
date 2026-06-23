"""
繼電器 GPIO 控制 (Raspberry Pi 5)
=================================
用 gpiozero 控制繼電器 ON/OFF。繼電器的乾接點 (COM / NO) 接進手臂機櫃的
數位輸入,用「接點閉合/斷開」送訊號給手臂——電氣完全隔離,不必煩惱
Pi 的 3.3V 和手臂機櫃 24V 對不對得上。

【最重要的設定:ACTIVE_HIGH】
繼電器模組分兩種:
  * 高電位觸發 (active-high):GPIO 給 HIGH → 繼電器吸合 (ON)
  * 低電位觸發 (active-low) :GPIO 給 LOW  → 繼電器吸合 (ON)
    ↑ 很多藍色光耦模組是這種
不確定是哪種?先跑本程式的測試模式:
  打 on ch1 → 聽到「喀」一聲 → 就是對的 (active-high)
  打 on ch1 沒反應,off ch1 才喀 → 改成 ACTIVE_HIGH = False
設定對之後,呼叫 .on() 一律代表「吸合 / 送訊號」,不用再記電位方向。

【接線 (4-relay module 為例)】
  繼電器模組      樹莓派 Pi 5
  ──────────      ────────────────────────────
  VCC         →   5V     (實體腳位 2)
  GND         →   GND    (實體腳位 34,或任一 GND)
  IN1         →   GPIO5  (實體腳位 29)
  IN2         →   GPIO6  (實體腳位 31)
  IN3         →   GPIO13 (實體腳位 33)
  IN4         →   GPIO16 (實體腳位 36)

  繼電器輸出端:
  COM + NO    →   接到手臂機櫃的數位輸入端子兩端 (哪兩端看手臂手冊)
  (繼電器吸合時 COM-NO 導通 = 送訊號給手臂)

【供電提醒】
  單路用 Pi 的 5V 沒問題;4 路同時動作的瞬間電流較大,
  如果 Pi 不穩 (重開或畫面閃),建議繼電器板改用外接 5V 電源。
  有些模組有 JD-VCC 跳帽可拆開做隔離供電。

【GPIO busy 排錯】
  出現 "lgpio.error: GPIO busy" → GPIO 被別的程式佔住。
  最常見原因:上次程式沒正常結束 (Ctrl-Z 只是暫停不是結束)。
  解法:sudo reboot  或  ps aux | grep relay → kill <PID>
  避免再踩:測完一定打 quit 結束;不要 Ctrl-Z。

執行測試:python relay_control.py
"""

import time
from gpiozero import OutputDevice

# =====================================================================
#  設定區
# =====================================================================

ACTIVE_HIGH = True   # 低電位觸發的模組請改成 False

# 名稱 → BCM 腳位。要接幾路繼電器就列幾路。
# ★ 這裡的腳位要和你實際接線一致
RELAY_PINS = {
    "ch1": 5,    # IN1 → GPIO5  (實體腳位 29)
    "ch2": 6,    # IN2 → GPIO6  (實體腳位 31)
    "ch3": 13,   # IN3 → GPIO13 (實體腳位 33)
    "ch4": 16,   # IN4 → GPIO16 (實體腳位 36)
}

PULSE_SEC = 0.3      # 送脈衝 (吸合一下再放開) 的預設長度


# =====================================================================
#  繼電器類別
# =====================================================================
class Relay:
    """
    單一繼電器。
    .on()  = 吸合 (送訊號給手臂)
    .off() = 釋放 (斷開訊號)
    """

    def __init__(self, pin, active_high=True):
        # initial_value=False → 開機就是「釋放」狀態,不會誤觸發手臂
        self._dev = OutputDevice(pin, active_high=active_high, initial_value=False)

    def on(self):
        """吸合繼電器 (COM-NO 導通)。"""
        self._dev.on()

    def off(self):
        """釋放繼電器 (COM-NO 斷開)。"""
        self._dev.off()

    def set(self, state):
        """True=吸合, False=釋放。"""
        self._dev.on() if state else self._dev.off()

    def pulse(self, seconds=PULSE_SEC):
        """吸合一下再釋放,常用來送一個觸發脈衝給手臂。"""
        self._dev.on()
        time.sleep(seconds)
        self._dev.off()

    @property
    def is_on(self):
        """目前是否吸合。"""
        return self._dev.value == 1

    def close(self):
        """釋放 GPIO 資源 (程式結束時呼叫)。"""
        self._dev.close()


class RelayBank:
    """
    多路繼電器,用名稱 (ch1, ch2…) 存取。
    方便整批管理、一鍵全部釋放。
    """

    def __init__(self, pins, active_high=True):
        self.relays = {name: Relay(pin, active_high) for name, pin in pins.items()}

    def on(self, name):
        self.relays[name].on()

    def off(self, name):
        self.relays[name].off()

    def pulse(self, name, seconds=PULSE_SEC):
        self.relays[name].pulse(seconds)

    def all_off(self):
        """全部釋放。"""
        for r in self.relays.values():
            r.off()

    def close(self):
        """釋放所有 GPIO 資源。"""
        for r in self.relays.values():
            r.close()


# =====================================================================
#  互動測試 (直接打指令切繼電器,確認接線與手臂反應)
# =====================================================================
#  指令範例:
#    on ch1     → ch1 吸合 (應該聽到「喀」一聲)
#    off ch1    → ch1 釋放
#    pulse ch1  → ch1 吸合 0.3 秒再釋放 (送脈衝)
#    status     → 顯示所有通道狀態
#    alloff     → 全部釋放
#    quit       → 結束 (★ 不要用 Ctrl-Z,會導致 GPIO busy)
#
def _interactive_test():
    bank = RelayBank(RELAY_PINS, active_high=ACTIVE_HIGH)
    names = list(RELAY_PINS)
    print("繼電器測試。可用通道:", ", ".join(names))
    print("指令:on <ch> / off <ch> / pulse <ch> / status / alloff / quit")
    try:
        while True:
            parts = input("> ").strip().split()
            if not parts:
                continue
            op = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else None

            if op == "quit":
                break
            elif op == "status":
                for n in names:
                    print(f"  {n}: {'ON (吸合)' if bank.relays[n].is_on else 'OFF (釋放)'}")
            elif op == "alloff":
                bank.all_off()
                print("全部釋放")
            elif op in ("on", "off", "pulse") and arg in bank.relays:
                if op == "on":
                    bank.on(arg); print(f"{arg} 吸合 (ON)")
                elif op == "off":
                    bank.off(arg); print(f"{arg} 釋放 (OFF)")
                else:
                    bank.pulse(arg); print(f"{arg} 送出 {PULSE_SEC}s 脈衝")
            else:
                print("格式錯誤。例:on ch1 / off ch1 / pulse ch1 / status / alloff / quit")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        bank.all_off()
        bank.close()
        print("\n已全部釋放並關閉 GPIO。")


if __name__ == "__main__":
    _interactive_test()
