"""
手指數量偵測 → 機械手臂取放  主程式 (Raspberry Pi 5)
=====================================================
辨識:MediaPipe Hand Landmarker(21 關節點)+ 幾何規則數手指 (1~5)
流程:偵測手勢 → 倒數 N 秒鎖定(手勢一變就重來)→ 手臂作動(背景執行緒)→ 完成回待機
視窗:綠色=可偵測 / 黃色=倒數鎖定中 / 紅色=手臂作動中
通訊:GPIO 數位訊號 (one-hot 握手);GPIO 用 gpiozero (Pi 5 勿用 RPi.GPIO)

★ 視窗需要顯示器:用 HDMI 螢幕或 VNC 才看得到;純 SSH 看不到視窗。
★ 中文標示需要字型:sudo apt install fonts-noto-cjk;沒有就自動退回英文。
★ 套件:pip install mediapipe opencv-python numpy pillow
"""

import time
import os
import threading

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from picamera2 import Picamera2
from gpiozero import OutputDevice, InputDevice

try:
    from PIL import Image, ImageDraw, ImageFont
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

# =====================================================================
#  設定區
# =====================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "hand_landmarker.task")
COUNTDOWN_SEC = 5.0       # 鎖定前的倒數秒數,要改幾秒改這裡

# 中文字型路徑 (找不到就自動用英文)。Pi 裝好 fonts-noto-cjk 後通常是這個:
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

# 手指數 → 取料點 GPIO 腳位 (BCM)。要幾個點就留幾個,one-hot 各一條線。
PICK_PINS = {1: 5, 2: 6, 3: 13, 4: 16, 5: 26}
PLACE_PIN = 19
DONE_PIN  = 21
PUMP_PIN  = 17
VALVE_PIN = 27

INFLATE_SEC = 2.0
DEFLATE_SEC = 2.0
PULSE_SEC   = 0.2
MOVE_TIMEOUT_SEC = 30


# =====================================================================
#  數手指 (幾何規則,不需訓練)
# =====================================================================
def count_fingers(landmarks, handedness_label):
    fingers = 0
    for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)):
        if landmarks[tip].y < landmarks[pip].y:
            fingers += 1
    # 拇指:左右手 x 方向相反;若常差 1 就把 < 跟 > 對調
    if handedness_label == "Right":
        if landmarks[4].x < landmarks[3].x:
            fingers += 1
    else:
        if landmarks[4].x > landmarks[3].x:
            fingers += 1
    return fingers


# =====================================================================
#  相機 + 手指辨識
# =====================================================================
class FingerCam:
    def __init__(self, model_path, size=(640, 480)):
        self.landmarker = mp_vision.HandLandmarker.create_from_options(
            mp_vision.HandLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=model_path),
                running_mode=mp_vision.RunningMode.VIDEO,
                num_hands=1,
            )
        )
        self.cam = Picamera2()
        cfg = self.cam.create_preview_configuration(
            main={"format": "RGB888", "size": size}
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(1.0)
        self.frame_idx = 0

    def read(self):
        """回傳 (RGB frame, 手指數或 None)。"""
        frame = self.cam.capture_array()
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        ts_ms = self.frame_idx * 33
        self.frame_idx += 1
        result = self.landmarker.detect_for_video(mp_image, ts_ms)
        count = None
        if result.hand_landmarks:
            label = result.handedness[0][0].category_name
            count = count_fingers(result.hand_landmarks[0], label)
        return frame, count

    def close(self):
        self.cam.stop()


# =====================================================================
#  畫面橫幅 (中文優先,無字型時退回英文)
# =====================================================================
class Banner:
    def __init__(self, font_path, size=40):
        self.font = None
        if _HAS_PIL:
            try:
                self.font = ImageFont.truetype(font_path, size)
            except Exception:
                print("找不到中文字型,改用英文標示 (可 sudo apt install fonts-noto-cjk)")

    def draw(self, frame_bgr, text_zh, text_en, color_bgr):
        h, w = frame_bgr.shape[:2]
        cv2.rectangle(frame_bgr, (0, 0), (w, 70), color_bgr, -1)
        if self.font is None:
            cv2.putText(frame_bgr, text_en, (20, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            return frame_bgr
        img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        ImageDraw.Draw(img).text((20, 12), text_zh, font=self.font, fill=(255, 255, 255))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


# =====================================================================
#  與手臂的數位握手
# =====================================================================
class ArmIO:
    def __init__(self, pick_pins, place_pin, done_pin):
        self.pick = {n: OutputDevice(p, active_high=True, initial_value=False)
                     for n, p in pick_pins.items()}
        self.place = OutputDevice(place_pin, active_high=True, initial_value=False)
        self.done = InputDevice(done_pin, pull_up=False)

    def _pulse_then_wait(self, dev):
        dev.on(); time.sleep(PULSE_SEC); dev.off()
        return self._wait_done()

    def _wait_done(self):
        start = time.time()
        while not self.done.value:
            if time.time() - start > MOVE_TIMEOUT_SEC:
                return False
            time.sleep(0.01)
        return True

    def goto_pick(self, count):
        return self._pulse_then_wait(self.pick[count])

    def goto_place(self):
        return self._pulse_then_wait(self.place)


# =====================================================================
#  氣動夾爪
# =====================================================================
class Gripper:
    def __init__(self, pump_pin, valve_pin):
        self.pump = OutputDevice(pump_pin, active_high=True, initial_value=False)
        self.valve = OutputDevice(valve_pin, active_high=True, initial_value=False)

    def inflate(self):
        self.valve.on(); self.pump.on()
        time.sleep(INFLATE_SEC)
        self.pump.off()

    def deflate(self):
        self.pump.off(); self.valve.off()
        time.sleep(DEFLATE_SEC)

    def off(self):
        self.pump.off(); self.valve.off()


# =====================================================================
#  一次完整取放循環 (在背景執行緒裡跑,所以視窗不會卡)
# =====================================================================
def run_cycle(arm, gripper, count):
    print(f"[流程] 鎖定 {count} 指 → 送出取料點 {count} 指令")
    if not arm.goto_pick(count):
        print("[警告] 等手臂取料移動逾時,取消本次循環")
        return
    print("[流程] 手臂到取料點,開始充氣夾取")
    gripper.inflate()
    print("[流程] 夾取完成,通知手臂前往放置點")
    if not arm.goto_place():
        print("[警告] 等手臂放置移動逾時,洩氣後回待機")
        gripper.deflate()
        return
    print("[流程] 手臂到放置點,開始洩氣放開")
    gripper.deflate()
    print("[流程] 完成一個循環,回到待機\n")


# =====================================================================
#  主程式 (狀態機 + 倒數鎖定 + 顯示視窗)
# =====================================================================
def main():
    cam = FingerCam(MODEL_PATH)
    gripper = Gripper(PUMP_PIN, VALVE_PIN)
    arm = ArmIO(PICK_PINS, PLACE_PIN, DONE_PIN)
    banner = Banner(FONT_PATH, size=40)

    state = "DETECTING"          # DETECTING / RUNNING
    candidate = None             # 目前正在倒數的手指數
    countdown_start = None
    worker = None

    print("待機中,伸出手指 (1~5)。視窗:綠=待機 黃=倒數 紅=作動中。按 q 結束。")
    try:
        while True:
            frame_rgb, count = cam.read()
            now = time.time()

            if state == "DETECTING":
                if count in PICK_PINS:
                    if count == candidate:
                        # 手勢沒變,倒數歸零就鎖定並啟動手臂 (背景執行緒)
                        if now - countdown_start >= COUNTDOWN_SEC:
                            worker = threading.Thread(
                                target=run_cycle, args=(arm, gripper, count), daemon=True)
                            worker.start()
                            state = "RUNNING"
                    else:
                        # 剛偵測到或手勢改變 → 以新手勢重新開始倒數
                        candidate = count
                        countdown_start = now
                else:
                    # 手不見了 → 取消倒數
                    candidate = None
                    countdown_start = None

            elif state == "RUNNING":
                if worker is None or not worker.is_alive():
                    state = "DETECTING"
                    candidate = None
                    countdown_start = None

            # ---------- 畫面 ----------
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            if state == "RUNNING":
                frame = banner.draw(frame, "機器手臂作動中,請勿靠近",
                                    "ARM RUNNING - KEEP CLEAR", (0, 0, 255))
            elif candidate is not None:
                remaining = COUNTDOWN_SEC - (now - countdown_start)
                sec = max(0, int(remaining) + 1)
                frame = banner.draw(frame, f"鎖定 {candidate} 指中… {sec}",
                                    f"LOCKING {candidate} IN {sec}", (0, 180, 255))
            else:
                frame = banner.draw(frame, "可偵測手勢", "READY", (0, 150, 0))

            label = f"fingers: {count}" if count is not None else "no hand"
            cv2.putText(frame, label, (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            cv2.imshow("Gesture -> Arm", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    except KeyboardInterrupt:
        pass
    finally:
        gripper.off()
        cam.close()
        cv2.destroyAllWindows()
        print("結束程式。")


if __name__ == "__main__":
    main()
