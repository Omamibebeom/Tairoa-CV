"""
vision_tuner.py —— 互動調參工具: ROI 裁切 + HSV 過濾 + 物件中心偵測

用途
  1) 框出你只想看的桌面區域 (ROI), 把畫面縮到要夾取的範圍, 減少雜訊與
     邊緣畸變 / 傾斜誤差。
  2) 用滑桿即時調 HSV / 形態學 / 最小面積, 直到畫面只剩你的紅 / 藍物件。
  3) 看到偵測出的中心點與座標, 滿意後按 s 存成 vision_config.json。

之後在 arm_server 之類的程式可直接用:
    from color_detect import DetectConfig, detect_objects
    cfg = DetectConfig.from_json("vision_config.json")
    objs, mask, roi = detect_objects(frame, cfg)
    if objs:
        cx, cy = objs[0]["cx"], objs[0]["cy"]   # 全畫面像素座標 → 餵給定位/標定

操作鍵
  滑桿       即時調參數
  r          重新框選 ROI (滑鼠拖曳框出, Enter 確定 / c 取消)
  f          ROI 還原成整張畫面
  s          存檔 vision_config.json (單一物件用)
  a          把這組 HSV 以命名 profile 加入 vision_profiles.json (多物件多色用)
  p          把目前參數印到終端機
  q 或 ESC   離開

提示: 想抓「紅色」時, 把 H min 設大 (約170)、H max 設小 (約10), 程式會自動
      用跨界兩段聯集去抓 (紅色 hue 落在 0 與 179 兩端)。
"""
import os
import cv2

import board_config as cam          # 沿用專案開相機的慣例 (C270 / DSHOW / 1280x720)
from color_detect import DetectConfig, detect_objects

CONFIG_FILE = "vision_config.json"
PROFILES_FILE = "vision_profiles.json"   # 多物件多色: 各色 append 進這裡
WIN = "tuner (ROI view)"            # 主視窗: 裁切後畫面 + 滑桿 + 偵測疊圖
WIN_MASK = "mask"                  # 遮罩視窗
WIN_FULL = "full (press r to set ROI)"  # 全畫面縮圖 + ROI 框

# (滑桿名稱, DetectConfig 欄位, 上限)。MinArea 以 /100 顯示, 實際值 = 滑桿 * 100。
TRACKBARS = [
    ("H min",        "h_min",    179),
    ("H max",        "h_max",    179),
    ("S min",        "s_min",    255),
    ("S max",        "s_max",    255),
    ("V min",        "v_min",    255),
    ("V max",        "v_max",    255),
    ("Blur",         "blur",      15),
    ("Open",         "open_k",    31),
    ("Close",        "close_k",   31),
    ("MinArea/100",  "min_area", 300),
]


def _noop(_):
    pass


def init_trackbars(cfg):
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 900, 700)
    for name, _field, maxv in TRACKBARS:
        cv2.createTrackbar(name, WIN, 0, maxv, _noop)
    push_cfg_to_trackbars(cfg)


def push_cfg_to_trackbars(cfg):
    """把 cfg 的值寫進滑桿 (啟動載入既有設定時用)。"""
    for name, field, maxv in TRACKBARS:
        val = getattr(cfg, field)
        if field == "min_area":
            val = val // 100
        cv2.setTrackbarPos(name, WIN, int(max(0, min(val, maxv))))


def read_trackbars(cfg):
    """把滑桿目前值讀回 cfg (ROI 不在滑桿上, 由 r/f 鍵設定)。"""
    for name, field, _maxv in TRACKBARS:
        v = cv2.getTrackbarPos(name, WIN)
        if field == "min_area":
            v *= 100
        setattr(cfg, field, v)
    return cfg


def select_roi(cap, cfg):
    """抓一張當前畫面讓使用者拖曳框選 ROI。取消(全0)則不更動。"""
    ok, frame = cap.read()
    if not ok:
        print("[tuner] 取得畫面失敗, 無法框選")
        return
    r = cv2.selectROI("select ROI (drag, Enter=OK / c=cancel)", frame,
                      showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("select ROI (drag, Enter=OK / c=cancel)")
    x, y, w, h = (int(v) for v in r)
    if w > 0 and h > 0:
        cfg.roi_x, cfg.roi_y, cfg.roi_w, cfg.roi_h = x, y, w, h
        print(f"[tuner] ROI = ({x},{y},{w},{h})")
    else:
        print("[tuner] 取消框選, ROI 不變")


def draw_overlay(view, objs, cfg):
    """在裁切後畫面 view 上畫輪廓 + 中心 + 座標文字。"""
    for i, o in enumerate(objs):
        color = (0, 255, 0) if i == 0 else (0, 220, 220)  # 最大綠, 其餘黃
        cv2.drawContours(view, [o["contour"]], -1, color, 2)
        cx, cy = int(round(o["cx_roi"])), int(round(o["cy_roi"]))
        cv2.drawMarker(view, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)
        cv2.circle(view, (cx, cy), 3, color, -1)
        txt = f"#{i} ({o['cx']:.0f},{o['cy']:.0f}) a={o['area']:.0f}"
        cv2.putText(view, txt, (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    wrap = "RED-WRAP ON" if cfg.h_min > cfg.h_max else ""
    head = f"objs={len(objs)}  {wrap}   [r]ROI [f]ull [s]ave [a]dd-profile [q]uit"
    cv2.putText(view, head, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2, cv2.LINE_AA)
    return view


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            c = DetectConfig.from_json(CONFIG_FILE)
            print(f"[tuner] 已載入既有設定 {CONFIG_FILE}")
            return c
        except Exception as e:
            print(f"[tuner] 讀 {CONFIG_FILE} 失敗 ({e}); 改用預設值")
    return DetectConfig()


def append_profile(cfg):
    """把目前參數以一個『命名 profile』加入 vision_profiles.json。
    到終端機輸入名稱 (例 red/blue); 同名則覆蓋。供多物件多色偵測使用。"""
    from color_detect import load_profiles, save_profiles
    name = input("  這組 HSV 的物件名稱 (例 red/blue; Enter 取消): ").strip()
    if not name:
        print("  取消")
        return
    import copy
    cfg = copy.copy(cfg)
    cfg.name = name
    # 保留目前框好的 ROI(A 区), 各颜色 profile 共用同一个 A 区范围。
    # 建议流程: 先用 r 框好 A 区 → 再逐色按 a 加入 profile。
    try:
        profiles = load_profiles(PROFILES_FILE) if os.path.exists(PROFILES_FILE) else []
    except Exception:
        profiles = []
    profiles = [p for p in profiles if p.name != name]  # 同名覆蓋
    profiles.append(cfg)
    save_profiles(profiles, PROFILES_FILE)
    print(f"  已寫入 profile '{name}' → {os.path.abspath(PROFILES_FILE)} "
          f"(共 {len(profiles)} 個: {', '.join(p.name for p in profiles)})")


def main():
    cfg = load_config()
    cap = cam.open_camera()
    if not cap.isOpened():
        print("[tuner] 開不了相機。檢查 C270 是否插好、是否被其他程式佔用。")
        return
    init_trackbars(cfg)
    cv2.namedWindow(WIN_MASK, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_FULL, cv2.WINDOW_NORMAL)
    print(__doc__)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[tuner] 讀取畫面失敗, 重試...")
            cv2.waitKey(50)
            continue

        read_trackbars(cfg)
        objs, mask, (rx, ry, rw, rh) = detect_objects(frame, cfg)

        view = frame[ry:ry + rh, rx:rx + rw].copy()
        view = draw_overlay(view, objs, cfg)

        full = frame.copy()
        cv2.rectangle(full, (rx, ry), (rx + rw, ry + rh), (0, 165, 255), 2)
        full_small = cv2.resize(full, (640, 360))

        cv2.imshow(WIN, view)
        cv2.imshow(WIN_MASK, mask)
        cv2.imshow(WIN_FULL, full_small)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):           # q 或 ESC
            break
        elif key == ord('r'):
            select_roi(cap, cfg)
        elif key == ord('f'):
            cfg.roi_x = cfg.roi_y = cfg.roi_w = cfg.roi_h = 0
            print("[tuner] ROI 還原為整張畫面")
        elif key == ord('s'):
            cfg.to_json(CONFIG_FILE)
            print(f"[tuner] 已存檔 → {os.path.abspath(CONFIG_FILE)}")
        elif key == ord('a'):
            append_profile(cfg)
        elif key == ord('p'):
            print("[tuner] 目前參數:", cfg)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
