"""
手指數量偵測 → 機械手臂取放  主程式 (Raspberry Pi 5)
=====================================================
TAIROA「視覺與機器人整合實作練習」

【整體流程】
  1. Pi 5 的 CSI 相機拍攝畫面
  2. MediaPipe Hand Landmarker 辨識手部 21 個關節點
  3. 幾何規則數出伸直的手指數 (1~5)
  4. 同一手指數維持 N 秒 → 鎖定手勢
  5. GPIO 數位訊號 (one-hot) 通知手臂移到對應取料點
  6. 手臂到位回傳 DONE → Pi 控制氣動夾爪充氣夾取
  7. 通知手臂移到放置點 → 手臂到位 → 洩氣放開
  8. 回到步驟 1,繼續偵測下一個手勢

【視窗顯示】
  綠色 = 可偵測手勢 (READY)
  黃色 = 倒數鎖定中 (LOCKING N IN X)
  紅色 = 手臂作動中,請勿靠近 (ARM RUNNING)

【GPIO 訊號約定 (Pi ↔ 手臂機櫃)】
  Pi 輸出 → 手臂:PICK_n (脈衝=去第n個取料點), PLACE (脈衝=去放置點)
  Pi 輸入 ← 手臂:DONE (高電位=移動完成, 低電位=移動中)
  Pi 輸出 → 夾爪:PUMP (氣泵), VALVE (電磁閥)

【電氣注意】
  Pi GPIO = 3.3V;手臂機櫃可能是 5V/12V/24V。
  中間務必經「繼電器」或「光耦合器」隔離,並且兩端 GND 要共地。
  切勿將手臂的高壓訊號直接接到 Pi 的 GPIO,會燒毀 Pi。

【執行環境】
  ★ 系統:Raspberry Pi OS 12 Bookworm (64-bit),不要用 Trixie
  ★ GPIO:使用 gpiozero (Pi 5 不支援 RPi.GPIO)
  ★ 相機:Picamera2 (CSI 相機模組);USB webcam 請改用 cv2.VideoCapture
  ★ 視窗:需要接 HDMI 螢幕或開 VNC;純 SSH 沒有畫面會報錯
  ★ 中文:sudo apt install fonts-noto-cjk;沒裝會自動退回英文
  ★ 套件:pip install mediapipe opencv-python numpy pillow
"""

# =====================================================================
#  匯入套件
# =====================================================================
import time
import os
import threading          # 手臂動作放背景執行緒,避免視窗凍住

import cv2                # OpenCV:影像處理、顯示視窗
import numpy as np        # 陣列運算 (PIL↔OpenCV 轉換用)
import mediapipe as mp    # Google MediaPipe:手部關節點偵測
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

# Pi 5 專用套件 (筆電上沒有,所以筆電測試請用 test_webcam.py)
from picamera2 import Picamera2                  # Pi 官方相機模組
from gpiozero import OutputDevice, InputDevice    # Pi 5 的 GPIO 控制

# Pillow:用來在 OpenCV 影像上畫中文字 (OpenCV 的 putText 不支援中文)
try:
    from PIL import Image, ImageDraw, ImageFont
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

# =====================================================================
#  設定區 (依你的硬體修改這一段就好,其他程式碼不用動)
# =====================================================================

# --- 模型路徑 (用 __file__ 取程式檔所在資料夾,不管從哪裡執行都找得到) ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "hand_landmarker.task")

COUNTDOWN_SEC = 5.0       # 同一手勢要維持幾秒才鎖定 (避免誤觸)

# 中文字型 (Pi 上裝 fonts-noto-cjk 後的路徑;找不到自動退回英文)
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

# --- GPIO 腳位 (BCM 編號) ---
# 手指數 → 取料點:one-hot,每個手指數一條線
# 例:比 1 指 → GPIO5 送脈衝 → 手臂跑到第 1 個取料點
PICK_PINS = {
    1: 5,     # 1 指 → GPIO5  (實體腳位 29)
    2: 6,     # 2 指 → GPIO6  (實體腳位 31)
    3: 13,    # 3 指 → GPIO13 (實體腳位 33)
    4: 16,    # 4 指 → GPIO16 (實體腳位 36)
    5: 26,    # 5 指 → GPIO26 (實體腳位 37)  ← 只用 4 個取料點就刪這行
}
PLACE_PIN = 19   # Pi → 手臂:「去放置點」 (GPIO19, 實體腳位 35)
DONE_PIN  = 21   # 手臂 → Pi:「移動完成」 (GPIO21, 實體腳位 40) [輸入]
PUMP_PIN  = 17   # Pi → 繼電器 → 氣泵 (GPIO17, 實體腳位 11)
VALVE_PIN = 27   # Pi → 繼電器 → 電磁閥 (GPIO27, 實體腳位 13)

# --- 時間參數 ---
INFLATE_SEC      = 2.0    # 充氣時間 (秒)
DEFLATE_SEC      = 2.0    # 洩氣時間 (秒)
PULSE_SEC        = 0.2    # 送給手臂的脈衝寬度 (秒)
MOVE_TIMEOUT_SEC = 30     # 等手臂 DONE 的逾時 (秒)


# =====================================================================
#  數手指 (幾何規則,不需訓練模型)
# =====================================================================
#
#  MediaPipe 回傳 21 個關節點,每個有 (x, y, z) 座標 (正規化 0~1)。
#
#  判斷規則:
#    食指~小指:指尖 (tip) 的 y 比第二關節 (PIP) 小 → 在上方 → 伸直
#    拇指:橫向開合,用 x 判斷,左右手方向相反
#    ★ 常差 1 的話把拇指的 < 和 > 對調
#
def count_fingers(landmarks, handedness_label):
    """數出伸直的手指數 (0~5)。"""
    fingers = 0
    # 食指(8,6)、中指(12,10)、無名指(16,14)、小指(20,18)
    for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)):
        if landmarks[tip].y < landmarks[pip].y:   # 指尖比關節高 → 伸直
            fingers += 1
    # 拇指:右手 tip.x < ip.x → 伸直;左手相反
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
    """封裝 Picamera2 + MediaPipe。.read() → (RGB影像, 手指數或None)。"""

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
        time.sleep(1.0)          # 等相機暖機
        self.frame_idx = 0

    def read(self):
        """回傳 (RGB frame, 手指數或 None)。"""
        frame = self.cam.capture_array()
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        ts_ms = self.frame_idx * 33      # VIDEO 模式時間戳要嚴格遞增
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
    """在影像頂部畫彩色橫幅+文字。OpenCV 不支援中文,所以用 Pillow 繪製。"""

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
#  與手臂的數位握手 (GPIO)
# =====================================================================
class ArmIO:
    """
    GPIO 數位訊號與手臂通訊。握手:Pi 送脈衝 → 等手臂 DONE 拉高 → 下一步。
    """

    def __init__(self, pick_pins, place_pin, done_pin):
        self.pick = {n: OutputDevice(p, active_high=True, initial_value=False)
                     for n, p in pick_pins.items()}
        self.place = OutputDevice(place_pin, active_high=True, initial_value=False)
        self.done = InputDevice(done_pin, pull_up=False)

    def _pulse_then_wait(self, dev):
        """送脈衝 → 等 DONE。"""
        dev.on(); time.sleep(PULSE_SEC); dev.off()
        return self._wait_done()

    def _wait_done(self):
        """等手臂 DONE 拉高,逾時回 False。"""
        start = time.time()
        while not self.done.value:
            if time.time() - start > MOVE_TIMEOUT_SEC:
                return False
            time.sleep(0.01)
        return True

    def goto_pick(self, count):
        """通知手臂去第 count 個取料點。"""
        return self._pulse_then_wait(self.pick[count])

    def goto_place(self):
        """通知手臂去放置點。"""
        return self._pulse_then_wait(self.place)


# =====================================================================
#  氣動夾爪
# =====================================================================
class Gripper:
    """
    GPIO → 繼電器 → 氣泵/電磁閥,操作氣動軟夾爪。
    充氣=閥開+泵開→等→泵關(保壓);洩氣=泵關+閥關(排氣)→等。
    """

    def __init__(self, pump_pin, valve_pin):
        self.pump = OutputDevice(pump_pin, active_high=True, initial_value=False)
        self.valve = OutputDevice(valve_pin, active_high=True, initial_value=False)

    def inflate(self):
        """充氣夾取。"""
        self.valve.on(); self.pump.on()
        time.sleep(INFLATE_SEC)
        self.pump.off()

    def deflate(self):
        """洩氣放開。"""
        self.pump.off(); self.valve.off()
        time.sleep(DEFLATE_SEC)

    def off(self):
        """安全關閉。"""
        self.pump.off(); self.valve.off()


# =====================================================================
#  一次完整取放循環 (背景執行緒,視窗不會卡)
# =====================================================================
def run_cycle(arm, gripper, count):
    """取→放一次:去取料點→充氣→去放置點→洩氣。"""
    print(f"[流程] 鎖定 {count} 指 → 送出取料點 {count} 指令")
    if not arm.goto_pick(count):
        print("[警告] 等手臂取料移動逾時,取消本次循環"); return
    print("[流程] 手臂到取料點,開始充氣夾取")
    gripper.inflate()
    print("[流程] 夾取完成,通知手臂前往放置點")
    if not arm.goto_place():
        print("[警告] 等手臂放置移動逾時,洩氣後回待機"); gripper.deflate(); return
    print("[流程] 手臂到放置點,開始洩氣放開")
    gripper.deflate()
    print("[流程] 完成一個循環,回到待機\n")


# =====================================================================
#  主程式 (狀態機 + 倒數鎖定 + 顯示視窗)
# =====================================================================
#  狀態機:DETECTING (偵測/倒數) → RUNNING (手臂作動) → DETECTING
def main():
    cam = FingerCam(MODEL_PATH)
    gripper = Gripper(PUMP_PIN, VALVE_PIN)
    arm = ArmIO(PICK_PINS, PLACE_PIN, DONE_PIN)
    banner = Banner(FONT_PATH, size=40)

    state = "DETECTING"
    candidate = None             # 目前正在倒數的手指數
    countdown_start = None       # 倒數開始時間
    worker = None                # 背景執行緒

    print("待機中,伸出手指 (1~5)。視窗:綠=待機 黃=倒數 紅=作動中。按 q 結束。")
    try:
        while True:
            frame_rgb, count = cam.read()
            now = time.time()

            if state == "DETECTING":
                if count in PICK_PINS:
                    if count == candidate:
                        if now - countdown_start >= COUNTDOWN_SEC:
                            # 倒數結束、手勢沒變 → 鎖定,開背景執行緒跑手臂
                            worker = threading.Thread(
                                target=run_cycle, args=(arm, gripper, count), daemon=True)
                            worker.start()
                            state = "RUNNING"
                    else:
                        # 手勢改變 → 用新手指數重新倒數
                        candidate = count
                        countdown_start = now
                else:
                    # 手不見了 → 取消倒數
                    candidate = None
                    countdown_start = None

            elif state == "RUNNING":
                if worker is None or not worker.is_alive():
                    # 手臂做完 → 回偵測
                    state = "DETECTING"
                    candidate = None
                    countdown_start = None

            # --- 畫面 ---
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
