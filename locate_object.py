"""
locate_object.py —— 即時回傳「放在板子上的物件」相對原點的座標(免貼標籤)

【整體邏輯 / 偵測原理】
相機與板子固定不動的俯拍情境:
  1. 先按 b 鎖定板子姿態(知道板面在 3D 哪裡, 並標出原點)。
  2. 用 HSV/ROI 偵測物件 (color_detect, 讀 vision_config.json) — 耐光線,
     適合紅/藍等與環境差異大的物件; 用 vision_tuner.py 先把 HSV/ROI 調好。
  3. 取物件代表點(質心或接觸點)→ 反投影到板面 →
     得到它相對「板子原點」的 (X, Y) 座標(cm)→ 換算成第幾欄第幾列。

座標原點 = 板子畫了座標軸的那一角(畫面上以黃圈 "O(0,0)" 標示):
    +X (紅軸) 沿 5 格寬邊 (150 mm)，欄 A..E
    +Y (綠軸) 沿 7 格長邊 (210 mm)，列 1..7

【質心 vs 接觸點】(按 m 即時切換,兩點都會畫出來)
    質心(紅點)  : 對扁平物件最準;垂直俯拍預設用這個
    接觸點(紫點): 輪廓最低點,某些傾斜視角對高物件較好

操作:
    b = 鎖定板子姿態與原點
    m = 切換用 質心 / 接觸點 來判定座標
    g = 顯示/隱藏格線
    q / ESC = 離開
"""
import os
import time
import cv2
import numpy as np
import board_config as cfg
from color_detect import DetectConfig, detect_objects

DIFF_THRESH = 40        # 差異門檻:像素亮度差超過這個才算「有東西」(越大越不敏感)
MIN_AREA = 800          # 最小面積(像素):比這小的差異當雜訊忽略
COL_LABELS = "ABCDE"    # 欄的標籤(X 方向 5 欄)
SQUARE_M = cfg.SQUARE_LENGTH_M  # 每格邊長(公尺),用來換算格子
PRINT_PERIOD = 0.3      # 終端機即時輸出的最短間隔(秒)


def board_plane_pose(detector, board, gray, K, dist):
    """偵測板子並回傳姿態 (rvec, tvec);看不到足夠角點則回傳 None。"""
    cc, ci, mc, mi = detector.detectBoard(gray)   # 找板子
    if ci is None or len(ci) < 6:                  # 角點太少,姿態不可靠
        return None
    obj_pts, img_pts = board.matchImagePoints(cc, ci)  # 3D-2D 配對
    if obj_pts is None or len(obj_pts) < 6:
        return None
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)  # 解姿態
    return (rvec, tvec) if ok else None


def pixel_to_board_xy(u, v, R, t, K, dist):
    """把畫面像素 (u,v) 反投影到板面(板子座標系的 Z=0 平面)。
    回傳板子座標 (X, Y),單位公尺。
    原理:從相機射出一條穿過該像素的光線,求它跟板面平面的交點。"""
    # 1) 去畸變,把像素轉成「正規化相機光線方向」(z=1 的射線)
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    norm = cv2.undistortPoints(pt, K, dist)[0, 0]   # 得到 (x', y')
    d = np.array([norm[0], norm[1], 1.0])           # 光線方向向量 d
    # 2) 板面在「相機座標」裡:法向量 n = 旋轉矩陣第3欄,平面通過點 = tvec
    n = R[:, 2]
    p0 = t.ravel()
    denom = n @ d                                    # 光線與平面的夾角因子
    if abs(denom) < 1e-9:                            # 幾乎平行 → 無交點
        return None
    # 3) 解出光線要走多遠 s 才碰到平面,得到交點(相機座標)
    s = (n @ p0) / denom
    P_cam = s * d
    # 4) 把交點從「相機座標」轉回「板子座標」(R 的反向 = R.T)
    P_board = R.T @ (P_cam - p0)                     # 結果 Z≈0
    return P_board[0], P_board[1]                    # 只要 X, Y


def xy_to_cell(x_m, y_m):
    """板子座標(公尺)→ (格子標籤, 欄索引, 列索引);超出板子回傳 None。"""
    col = int(np.floor(x_m / SQUARE_M))   # X 除以每格 → 第幾欄(0起算)
    row = int(np.floor(y_m / SQUARE_M))   # Y 除以每格 → 第幾列
    if 0 <= col < cfg.SQUARES_X and 0 <= row < cfg.SQUARES_Y:  # 確認在板內
        return f"{COL_LABELS[col]}{row + 1}", col, row         # 例 "C4"
    return None


def draw_origin(view, rvec, tvec, K, dist):
    """在畫面上標出座標原點(0,0,0):畫 XYZ 軸 + 黃圈 + "O(0,0)" 文字。"""
    axis_len = cfg.SQUARE_LENGTH_M * 2
    cv2.drawFrameAxes(view, K, dist, rvec, tvec, axis_len, 3)  # 紅X綠Y藍Z
    org, _ = cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, K, dist)  # 投影原點
    px = tuple(org.ravel().astype(int))
    cv2.circle(view, px, 9, (0, 255, 255), 2)                 # 黃色空心圈
    cv2.putText(view, "O(0,0)", (px[0] + 10, px[1] + 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


def main():
    # 一樣需要標定檔(反投影要用到內參)
    if not os.path.exists(cfg.CALIB_FILE):
        print(f"ERROR: {cfg.CALIB_FILE} not found. Run calibrate.py first.")
        return
    data = np.load(cfg.CALIB_FILE)
    K, dist = data["camera_matrix"], data["dist_coeffs"]

    detector, board = cfg.get_detector()
    cap = cfg.open_camera()
    if not cap.isOpened():
        print("ERROR: cannot open the C270 camera")
        return

    # HSV/ROI 偵測設定 (vision_tuner.py 調出來的); 沒有就用預設
    try:
        det_cfg = DetectConfig.from_json("vision_config.json")
        print("[locate] 已載入 vision_config.json")
    except Exception:
        det_cfg = DetectConfig()
        print("[locate] 找不到 vision_config.json, 用預設 HSV; 建議先跑 vision_tuner.py")

    pose = None             # 鎖定的板子姿態,存 (rvec, R, tvec)
    show_grid = True        # 是否畫格線
    use_contact = False     # False=用質心判定, True=用接觸點判定
    last_print = 0.0        # 上次印到終端機的時間
    print(__doc__)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        view = frame.copy()

        # 若已鎖定姿態:畫原點與(可選)格線
        if pose is not None:
            rvec, R, t = pose
            if show_grid:
                for c in range(cfg.SQUARES_X + 1):
                    for r in range(cfg.SQUARES_Y + 1):
                        p = np.array([[c * SQUARE_M, r * SQUARE_M, 0.0]])  # 板上格點
                        px, _ = cv2.projectPoints(p, rvec, t, K, dist)     # → 畫面像素
                        px = tuple(px.ravel().astype(int))
                        cv2.circle(view, px, 2, (180, 180, 0), -1)
            draw_origin(view, rvec, t, K, dist)   # 標出座標原點

        status = "press 'b' to lock board pose (origin)"
        detected_coord = None   # 這一幀偵測到的座標,供終端機輸出用

        # 已鎖姿態就用 HSV/ROI 偵測物件
        if pose is not None:
            rvec, R, t = pose
            objs, _mask, (rx, ry, rw, rh) = detect_objects(frame, det_cfg)
            # 畫 ROI 框 (橘)
            if rw > 0 and rh > 0:
                cv2.rectangle(view, (rx, ry), (rx + rw, ry + rh), (0, 140, 255), 1)
            if objs:
                obj = objs[0]                           # 面積最大者
                cx, cy = int(round(obj["cx"])), int(round(obj["cy"]))
                # --- 接觸點(輪廓 y 最大者; contour 為 ROI 座標, 平移回整張畫面)---
                cnt = obj["contour"]
                by = tuple(cnt[cnt[:, :, 1].argmax()][0])
                bottom = (int(by[0] + rx), int(by[1] + ry))
                # 依模式選一個點來判定座標
                ux, uy = (bottom if use_contact else (cx, cy))

                # --- 視覺標示 ---
                cv2.drawContours(view, [cnt + np.array([[[rx, ry]]])], -1,
                                 (0, 165, 255), 2)        # 橘色輪廓 (平移回全畫面)
                cv2.circle(view, (cx, cy), 7 if not use_contact else 4, (0, 0, 255), -1)   # 紅=質心
                cv2.circle(view, bottom, 7 if use_contact else 4, (255, 0, 255), -1)       # 紫=接觸點

                # 反投影 → 板上座標(相對原點)
                xy = pixel_to_board_xy(ux, uy, R, t, K, dist)
                if xy is not None:
                    x_cm, y_cm = xy[0] * 100, xy[1] * 100
                    r_cm = float(np.hypot(x_cm, y_cm))   # 離原點直線距離
                    cell = xy_to_cell(*xy)
                    cell_name = cell[0] if cell else "off-board"
                    detected_coord = (x_cm, y_cm, r_cm, cell_name)
                    status = (f"X={x_cm:6.1f}  Y={y_cm:6.1f} cm   "
                              f"r={r_cm:5.1f} cm   cell={cell_name}")
                    cv2.putText(view, f"{cell_name} ({x_cm:.1f},{y_cm:.1f})",
                                (ux + 10, uy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)
                else:
                    status = "object detected (plane projection failed)"
            else:
                status = "no object detected (check HSV/ROI in vision_tuner)"

        # --- 終端機即時輸出(throttle)---
        now = time.time()
        if pose is not None and now - last_print >= PRINT_PERIOD:
            if detected_coord:
                x_cm, y_cm, r_cm, cell_name = detected_coord
                print(f"[obj] X={x_cm:7.1f}  Y={y_cm:7.1f} cm  r={r_cm:6.1f} cm  cell={cell_name}")
            else:
                print("[obj] (none)")
            last_print = now

        # --- 畫面文字:狀態列 + 模式 + 操作提示 ---
        cv2.putText(view, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        mode_txt = "mode: CONTACT(purple)" if use_contact else "mode: CENTROID(red)"
        cv2.putText(view, mode_txt, (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(view, "b=lock pose  m=point  g=grid  q=quit",
                    (10, view.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow("locate object", view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('g'):              # 切換格線顯示
            show_grid = not show_grid
        elif key == ord('m'):              # 切換 質心/接觸點
            use_contact = not use_contact
            print("  判定點 ->", "接觸點(紫)" if use_contact else "質心(紅)")
        elif key == ord('b'):              # 鎖定板子姿態 (原點)
            p = board_plane_pose(detector, board, gray, K, dist)
            if p is None:
                print("  cannot see enough of the board to lock pose")
            else:
                rvec, tvec = p
                R, _ = cv2.Rodrigues(rvec)  # 旋轉向量 → 旋轉矩陣(反投影要用矩陣)
                pose = (rvec, R, tvec)
                print("  pose locked (原點已鎖定)")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
