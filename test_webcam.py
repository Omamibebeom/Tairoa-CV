"""
手指數量 + 倒數鎖定 + 紅綠視窗  測試 (筆電 / USB webcam 用)
==========================================================
【這支在課程裡的定位】
  屬於「上樹莓派之前的筆電預演」,不是正式的第一階段。
  正式第一階段是在 Pi 上接螢幕+鍵盤+CSI 相機,跑 gesture_pick_place.py 並把
  FAKE_ARM 設成 True (同樣只測辨識、不接手臂)。
  想先在自己筆電上熟悉流程,或手邊只有 USB webcam,就先跑這支。
  (這支也能在 Pi 上用 USB webcam 跑;若用的是 CSI 相機,請改用主程式 FAKE_ARM=True。)

跟主程式 gesture_pick_place.py 同樣的流程,但:
  * 用筆電 webcam (cv2.VideoCapture) 取代 Picamera2
  * 手臂動作用「假的 sleep」取代,不需 GPIO / 手臂 / 氣動
讓你先在筆電上把「數手指 + 倒數鎖定 + 紅綠切換」整個流程測到順。

【流程】
  綠色 可偵測 → 偵測到手勢進入 黃色倒數(手勢一變就重來)
  → 倒數歸零鎖定 → 紅色 作動中(背景假動作)→ 完成回綠色

【執行】
  python test_webcam.py   (按 q 或 ESC 結束)

【中文字型】
  筆電上可把 FONT_PATH 指到本機中文字型,否則自動退回英文:
    Windows: C:/Windows/Fonts/msjh.ttc
    macOS  : /System/Library/Fonts/PingFang.ttc

【要測的情境】
  1. 倒數中途改手指數 (例如 3 → 2) → 應以新數字重新倒數
  2. 倒數中途把手收掉 → 應立即變回綠色
  3. 紅色作動期間比手勢 → 應被忽略,且畫面不凍住
"""

# =====================================================================
#  匯入套件
# =====================================================================
import time
import os
import threading          # 假手臂動作也放背景,模擬真正的非同步行為

import cv2                # OpenCV:抓 webcam + 顯示視窗
import numpy as np
import mediapipe as mp    # MediaPipe:手部關節點偵測
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

# Pillow:中文橫幅用 (沒裝就退回英文)
try:
    from PIL import Image, ImageDraw, ImageFont
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

# =====================================================================
#  設定區
# =====================================================================

# 模型路徑 (以程式檔所在資料夾為基準,從哪裡執行都找得到)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "hand_landmarker.task")

CAM_INDEX = 0              # 攝影機編號,多顆時改 1, 2…
COUNTDOWN_SEC = 5.0        # 鎖定前倒數秒數
FAKE_RUN_SEC = 4.0         # 模擬手臂作動的時間 (紅色維持多久)
VALID_COUNTS = {1, 2, 3, 4, 5}   # 哪些手指數算有效 (對應主程式的 PICK_PINS)

# 中文字型路徑 (找不到自動退回英文)
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


# =====================================================================
#  數手指 (與主程式 gesture_pick_place.py 完全相同的邏輯)
# =====================================================================
#
#  食指~小指:指尖(tip)比第二關節(PIP)高(y較小)→ 伸直
#  拇指:左右手 x 方向相反;常差 1 就把 < > 對調
#
def count_fingers(landmarks, handedness_label):
    """數出伸直的手指數 (0~5)。"""
    fingers = 0
    for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)):
        if landmarks[tip].y < landmarks[pip].y:
            fingers += 1
    if handedness_label == "Right":
        if landmarks[4].x < landmarks[3].x:
            fingers += 1
    else:
        if landmarks[4].x > landmarks[3].x:
            fingers += 1
    return fingers


# =====================================================================
#  畫面橫幅 (中文優先,無字型退回英文)
# =====================================================================
class Banner:
    """在影像頂部畫彩色橫幅。OpenCV 不支援中文 → 用 Pillow 畫。"""

    def __init__(self, font_path, size=40):
        self.font = None
        if _HAS_PIL:
            try:
                self.font = ImageFont.truetype(font_path, size)
            except Exception:
                print("找不到中文字型,改用英文標示 (可把 FONT_PATH 指到本機中文字型)")

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
#  模擬手臂作動 (取代真正的 run_cycle,只是 sleep)
# =====================================================================
def fake_cycle(count):
    """假裝手臂在動作 (sleep),紅色橫幅會維持 FAKE_RUN_SEC 秒。"""
    print(f"[模擬] 鎖定 {count} 指 → 假裝手臂移動 / 充氣 / 放置 / 洩氣…")
    time.sleep(FAKE_RUN_SEC)
    print("[模擬] 假裝手臂完成,回到待機\n")


# =====================================================================
#  主程式 (狀態機 + 倒數鎖定 + 紅綠視窗)
# =====================================================================
#  狀態機:DETECTING (偵測/倒數) → RUNNING (假手臂作動) → DETECTING
def main():
    # --- 初始化 MediaPipe Hand Landmarker ---
    landmarker = mp_vision.HandLandmarker.create_from_options(
        mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
        )
    )

    # --- 開啟 webcam ---
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"打不開攝影機 (index={CAM_INDEX}),試試把 CAM_INDEX 改成 1")
        return
    banner = Banner(FONT_PATH, size=40)

    # --- 狀態機變數 ---
    state = "DETECTING"          # "DETECTING" 或 "RUNNING"
    candidate = None             # 目前正在倒數的手指數
    countdown_start = None       # 倒數開始時間
    worker = None                # 背景執行緒
    frame_idx = 0                # 給 MediaPipe 的遞增時間戳

    print("對著鏡頭伸出手指 (1~5)。視窗:綠=待機 黃=倒數 紅=作動中。按 q 結束。")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)   # 鏡像,看起來比較自然

            # --- 辨識手指 ---
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = frame_idx * 33       # 模擬 ~30fps 的遞增時間戳
            frame_idx += 1
            result = landmarker.detect_for_video(mp_image, ts_ms)

            count = None
            if result.hand_landmarks:
                label = result.handedness[0][0].category_name
                count = count_fingers(result.hand_landmarks[0], label)

            now = time.time()

            # --- 狀態機 ---
            if state == "DETECTING":
                if count in VALID_COUNTS:
                    if count == candidate:
                        # 手勢沒變 → 檢查倒數是否歸零
                        if now - countdown_start >= COUNTDOWN_SEC:
                            # 鎖定 → 開背景執行緒跑假動作
                            worker = threading.Thread(
                                target=fake_cycle, args=(count,), daemon=True)
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
                    # 假動作跑完 → 回偵測
                    state = "DETECTING"
                    candidate = None
                    countdown_start = None

            # --- 畫面 ---
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

            label_txt = f"fingers: {count}" if count is not None else "no hand"
            cv2.putText(frame, label_txt, (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            cv2.imshow("Finger -> (fake) Arm  test", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("結束程式。")


if __name__ == "__main__":
    main()
