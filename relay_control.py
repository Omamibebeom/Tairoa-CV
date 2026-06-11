"""
繼電器 GPIO 控制 (Raspberry Pi 5)
=================================
用 gpiozero 控制繼電器 ON/OFF。繼電器的乾接點 (COM / NO) 接進手臂機櫃的
數位輸入,用「接點閉合/斷開」送訊號給手臂——電氣完全隔離,不必煩惱
Pi 的 3.3V 和手臂機櫃 24V 對不對得上。

★ 最重要的設定:ACTIVE_HIGH ★
繼電器模組分兩種:
  * 高電位觸發 (active-high):GPIO 給 HIGH → 繼電器吸合 (ON)
  * 低電位觸發 (active-low) :GPIO 給 LOW  → 繼電器吸合 (ON)  ← 很多藍色光耦模組是這種
不確定是哪種?先跑本程式的測試模式,聽繼電器「喀」一聲時是 on 還是 off 指令就知道。
設定對之後,呼叫 .on() 一律代表「吸合 / 送訊號」,不用再記電位方向。

接線 (單路為例):
  Pi GPIO 腳位 ──→ 繼電器模組 IN
  Pi 5V        ──→ 繼電器模組 VCC      (單路用 Pi 5V 沒問題;多路建議外接 5V)
  Pi GND       ──→ 繼電器模組 GND
  繼電器 COM / NO ──→ 手臂機櫃的數位輸入端子兩端 (哪兩端看手臂手冊)

執行測試:python relay_control.py
"""

import time
from gpiozero import OutputDevice

# =====================================================================
#  設定區
# =====================================================================

ACTIVE_HIGH = True   # 低電位觸發的模組請改成 False

# 名稱 → BCM 腳位。要接幾路繼電器就列幾路 (對應手臂機櫃的各個輸入)。
RELAY_PINS = {
    "ch1": 5,
    "ch2": 6,
    "ch3": 13,
}

PULSE_SEC = 0.3      # 送脈衝 (吸合一下再放開) 的預設長度


# =====================================================================
#  繼電器類別
# =====================================================================
class Relay:
    """單一繼電器。.on()=吸合(送訊號),.off()=釋放。"""

    def __init__(self, pin, active_high=True):
        # initial_value=False → 開機就是「釋放」狀態,不會誤觸發手臂
        self._dev = OutputDevice(pin, active_high=active_high, initial_value=False)

    def on(self):
        self._dev.on()

    def off(self):
        self._dev.off()

    def set(self, state):
        self._dev.on() if state else self._dev.off()

    def pulse(self, seconds=PULSE_SEC):
        """吸合一下再釋放,常用來送一個觸發脈衝給手臂。"""
        self._dev.on()
        time.sleep(seconds)
        self._dev.off()

    @property
    def is_on(self):
        return self._dev.value == 1

    def close(self):
        self._dev.close()


class RelayBank:
    """多路繼電器,用名稱存取。"""

    def __init__(self, pins, active_high=True):
        self.relays = {name: Relay(pin, active_high) for name, pin in pins.items()}

    def on(self, name):
        self.relays[name].on()

    def off(self, name):
        self.relays[name].off()

    def pulse(self, name, seconds=PULSE_SEC):
        self.relays[name].pulse(seconds)

    def all_off(self):
        for r in self.relays.values():
            r.off()

    def close(self):
        for r in self.relays.values():
            r.close()


# =====================================================================
#  互動測試 (直接打指令切繼電器,確認接線與手臂反應)
# =====================================================================
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
