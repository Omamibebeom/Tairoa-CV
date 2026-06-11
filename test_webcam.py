"""
手指數量 + 倒數鎖定 + 紅綠視窗  測試 (筆電 / USB webcam 用)
==========================================================
跟主程式 gesture_pick_place.py 同樣的流程,但:
  * 用筆電 webcam (cv2) 取代 Picamera2
  * 手臂動作用一個「假的 sleep」取代,不需 GPIO / 手臂 / 氣動
讓你先在筆電上把「倒數鎖定 + 紅綠切換」整個流程測到順。

流程:綠色 可偵測 → 偵測到手勢進入 黃色倒數(手勢一變就重來)
      → 倒數歸零鎖定 → 紅色 作動中(背景假動作)→ 完成回綠色

執行:python test_webcam.py   (按 q 或 ESC 結束)
中文標示需要字型;筆電上可把 FONT_PATH 指到本機中文字型,否則自動退回英文:
  Windows: C:/Windows/Fonts/msjh.ttc
  macOS  : /System/Library/Fonts/PingFang.ttc
"""

import time
import os
import threading

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

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
CAM_INDEX = 0
COUNTDOWN_SEC = 5.0        # 鎖定前倒數秒數
FAKE_RUN_SEC = 4.0         # 模擬手臂作動的時間 (紅色維持多久)
VALID_COUNTS = {1, 2, 3, 4, 5}   # 哪些手指數算有效 (對應主程式的 PICK_PINS)
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


# =====================================================================
#  數手指 (與主程式相同)
# =====================================================================
def count_fingers(landmarks, handedness_label):
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
    print(f"[模擬] 鎖定 {count} 指 → 假裝手臂移動 / 充氣 / 放置 / 洩氣…")
    time.sleep(FAKE_RUN_SEC)
    print("[模擬] 假裝手臂完成,回到待機\n")


# =====================================================================
#  主程式 (狀態機 + 倒數鎖定 + 紅綠視窗)
# =====================================================================
def main():
    landmarker = mp_vision.HandLandmarker.create_from_options(
        mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
        )
    )
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"打不開攝影機 (index={CAM_INDEX}),試試把 CAM_INDEX 改成 1")
        return
    banner = Banner(FONT_PATH, size=40)

    state = "DETECTING"          # DETECTING / RUNNING
    candidate = None             # 目前正在倒數的手指數
    countdown_start = None
    worker = None
    frame_idx = 0

    print("對著鏡頭伸出手指 (1~5)。視窗:綠=待機 黃=倒數 紅=作動中。按 q 結束。")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)   # 鏡像,看起來自然

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = frame_idx * 33
            frame_idx += 1
            result = landmarker.detect_for_video(mp_image, ts_ms)

            count = None
            if result.hand_landmarks:
                label = result.handedness[0][0].category_name
                count = count_fingers(result.hand_landmarks[0], label)

            now = time.time()

            if state == "DETECTING":
                if count in VALID_COUNTS:
                    if count == candidate:
                        # 手勢沒變,倒數歸零就鎖定並啟動 (背景執行緒)
                        if now - countdown_start >= COUNTDOWN_SEC:
                            worker = threading.Thread(
                                target=fake_cycle, args=(count,), daemon=True)
                            worker.start()
                            state = "RUNNING"
                    else:
                        # 剛偵測到或手勢改變 → 以新手勢重新開始倒數
                        candidate = count
                        countdown_start = now
                else:
                    candidate = None
                    countdown_start = None

            elif state == "RUNNING":
                if worker is None or not worker.is_alive():
                    state = "DETECTING"
                    candidate = None
                    countdown_start = None

            # ---------- 畫面 ----------
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
