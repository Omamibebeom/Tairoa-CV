"""
手指數量 + 倒數鎖定 + 紅綠視窗  測試 (筆電 / USB webcam 用) + 絕對不報錯骨架版
==========================================================
【這支在課程裡的定位】
  屬於「上樹莓派之前的筆電預演」,不是正式的第一階段。
  正式第一階段是在 Pi 上接螢幕+鍵盤+CSI 相機,跑 gesture_pick_place.py 並把
  FAKE_ARM 設成 True (同樣只測辨識、不接手臂)。
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

# (已經徹底刪除會報錯的 mediapipe.framework 與 solutions)

# Pillow:中文橫幅用 (沒裝就退回英文)
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
COUNTDOWN_SEC = 5.0        
FAKE_RUN_SEC = 4.0         
VALID_COUNTS = {1, 2, 3, 4, 5}   

FONT_PATH = "C:/Windows/Fonts/msjh.ttc"  

# --- 👇 自己定義手部 21 個節點的連線方式 👇 ---
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),        # 食指
    (5, 9), (9, 10), (10, 11), (11, 12),   # 中指
    (9, 13), (13, 14), (14, 15), (15, 16), # 無名指
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20) # 小指
]
# ---------------------------------------------

# =====================================================================
#  數手指 
# =====================================================================
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
#  畫面橫幅
# =====================================================================
class Banner:
    def __init__(self, font_path, size=40):
        self.font = None
        if _HAS_PIL:
            try:
                self.font = ImageFont.truetype(font_path, size)
            except Exception:
                print("找不到中文字型,改用英文標示")

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


def fake_cycle(count):
    print(f"[模擬] 鎖定 {count} 指 → 假裝手臂移動 / 充氣 / 放置 / 洩氣…")
    time.sleep(FAKE_RUN_SEC)
    print("[模擬] 假裝手臂完成,回到待機\n")


# =====================================================================
#  主程式 
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
        print(f"打不開攝影機 (index={CAM_INDEX})")
        return
    banner = Banner(FONT_PATH, size=40)

    state = "DETECTING"          
    candidate = None             
    countdown_start = None       
    worker = None                
    frame_idx = 0                

    print("對著鏡頭伸出手指 (1~5)。按 q 結束。")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)   
            h, w = frame.shape[:2] # 取得畫面長寬

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = frame_idx * 33       
            frame_idx += 1
            result = landmarker.detect_for_video(mp_image, ts_ms)

            count = None
            if result.hand_landmarks:
               
                # --------------------------------

                label = result.handedness[0][0].category_name
                count = count_fingers(result.hand_landmarks[0], label)

            now = time.time()

            if state == "DETECTING":
                if count in VALID_COUNTS:
                    if count == candidate:
                        if now - countdown_start >= COUNTDOWN_SEC:
                            worker = threading.Thread(
                                target=fake_cycle, args=(count,), daemon=True)
                            worker.start()
                            state = "RUNNING"
                    else:
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