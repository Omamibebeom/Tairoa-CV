"""
arm_server.py — 笔电端 TCP Server:夹取定位 + 放置点查询 + 即时画面

==============================================================================
这支程式做什么
==============================================================================
相机固定俯拍桌面, 桌面有两个题目区:
  A 区(取物区): 放了多种颜色的物件, 用 HSV/ROI 框出、侦测中心点。
  B 区(放置区): 一些固定的指定放置点(每个颜色对应一个)。
手臂(达明 TM)透过 socket 向本程式要「下一个要夹的物件在哪、要放到哪」,
夹起来搬到 B 区对应点, 一个一个清空 A 区。

像素→手臂坐标用 affine(affine_calib.py 标定, 整个桌面平面共用一组)。
颜色→放置点用 place_points.json(teach_place.py 教出来)。

==============================================================================
通讯协定(对应 TM socket: 送字串、收字串, 前置 "$", 后置换行)
==============================================================================
一轮的流程(手臂先停在 A 区外, 画面干净时):
  手臂送 SCAN\r\n   → 本程式对「当下这一帧」侦测 A 区所有物件、排序、建佇列。
                      回 $COUNT,<n>\r\n   例 $COUNT,3
  手臂送 GET\r\n    → 吐出佇列中「下一个」, 回:
                      $<pick_x>,<pick_y>,<place_x>,<place_y>,<color>\r\n
                      例 $712.3,55.8,840.0,120.0,red
                      佇列空了 → $NONE\r\n
  手臂送 RESET\r\n  → 清空佇列, 回 $OK\r\n

为什么用「先 SCAN 再一直 GET」而不是「每次 GET 都重新侦测」:
  手臂夹取时会伸进 A 区, 若每次 GET 都即时侦测, 会侦测到手臂或被手臂挡住。
  改成一轮开始拍一张干净的快照、排好序冻结成佇列, 之后 GET 只是从佇列取下一个,
  手臂在画面里也不影响。画面上的即时侦测照常显示(只供你看), 与佇列无关。
  (相容:若手臂没先 SCAN 就直接 GET, 本程式会自动先 SCAN 一次。)

TM Flow 端大致写法(伪码):
  socket 连上 → 送 "SCAN\r\n" → 读一行取得数量
  Loop:
    送 "GET\r\n" → 读一行
    若收到 "$NONE" → 跳出 Loop(这一区清完)
    否则用 "," 拆成 pick_x,pick_y,place_x,place_y,color
        → 移到 (pick_x,pick_y) 夹起 → 移到 (place_x,place_y) 放下 → 回 Loop

==============================================================================
两种定位方式(METHOD 切换), 偵測都用 HSV/ROI
==============================================================================
  METHOD = "affine"   老师的最小平方(affine_calib.json), 不需要棋盘板。
  METHOD = "charuco"  旧方法(board_locator), 需要棋盘板 + 开机 lock。

需要的档案(放同一资料夹):
  vision_profiles.json  多颜色 HSV/ROI(vision_tuner.py 按 a 逐色加入)
                        没有就退回 vision_config.json 单色。
  affine_calib.json     像素→手臂(affine_calib.py)。
  place_points.json     颜色→B 区放置手臂坐标(teach_place.py)。没有则放置回 NA。

按 q / ESC 关闭画面。
"""
import os
import socket
import threading
import json
import cv2
import numpy as np
import board_config as cfg

# ========================= 设定 =========================
METHOD = "affine"            # "affine" 或 "charuco"
VISION_CONFIG = "vision_config.json"
PROFILES_CONFIG = "vision_profiles.json"
AFFINE_CONFIG = "affine_calib.json"
PLACE_CONFIG = "place_points.json"

# 夹取顺序:先照这个颜色优先序, 同色再由左到右(影像 x 小的先)。
# 不在这表里的颜色排最后。可自由调整。
PICK_ORDER = ["red", "blue", "green", "yellow"]

HOST = "0.0.0.0"
PORT = 5000
RECV_TIMEOUT = 30.0          # 秒; 一轮内手臂会多次 GET, 给宽一点

# charuco 模式才用到的手臂三参考点(mm), 教于 2025-06-22
ARM_ORIGIN = (679.392, 18.194)
ARM_XEND   = (829.392, 18.194)
ARM_YEND   = (679.392, 228.194)

# ---- 共享状态 ----
_latest = {"frame": None}
_frame_lock = threading.Lock()
_plan = {"queue": [], "i": 0, "scanned": False}   # 夹取佇列(快照排队)
_plan_lock = threading.Lock()


# ===================== 放置点表 =====================
def load_place_points(path=PLACE_CONFIG):
    """读 place_points.json: {"red":[x,y], "blue":[x,y], ...} → dict[str, (x,y)]。"""
    if not os.path.exists(path):
        print(f"[server] 找不到 {path}; 放置坐标一律回 NA(请先跑 teach_place.py)")
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        pts = {k: (float(v[0]), float(v[1])) for k, v in d.items()}
        print(f"[server] 放置点: {', '.join(f'{k}{tuple(v)}' for k, v in pts.items())}")
        return pts
    except Exception as e:
        print(f"[server] 读 {path} 失败 ({e}); 放置坐标回 NA")
        return {}


PLACE_POINTS = {}    # 在 main() 载入


# ===================== affine 模式定位器 =====================
class AffineLocator:
    """偵測(HSV/ROI) + 映射(AffineMapper)。介面对齐 BoardLocator。
    有 vision_profiles.json → 多颜色模式, 每个物件带 name(颜色)。
    像素→手臂的 affine 对所有颜色共用(几何与颜色无关), 只标定一次。"""
    def __init__(self, vision_cfg_path=VISION_CONFIG,
                 affine_cfg_path=AFFINE_CONFIG,
                 profiles_path=PROFILES_CONFIG):
        from color_detect import DetectConfig, load_profiles, clamp_roi
        from affine_calib import AffineMapper
        self._clamp_roi = clamp_roi
        self.profiles = None
        self.det_cfg = None
        if os.path.exists(profiles_path):
            try:
                self.profiles = load_profiles(profiles_path)
                names = ", ".join(p.name for p in self.profiles)
                print(f"[server] 多颜色模式: {len(self.profiles)} 组 profile [{names}]")
            except Exception as e:
                print(f"[server] 读 {profiles_path} 失败 ({e}); 退回单色")
        if self.profiles is None:
            try:
                self.det_cfg = DetectConfig.from_json(vision_cfg_path)
                print(f"[server] 单色 HSV: 已载入 {vision_cfg_path}")
            except Exception:
                self.det_cfg = DetectConfig()
                print(f"[server] 找不到 {vision_cfg_path}, 用预设 HSV")
        self.mapper = AffineMapper.from_json(affine_cfg_path)
        print(f"[server] affine: kind={self.mapper.kind}, n={self.mapper.n}, "
              f"RMS={self.mapper.rms}")
        self.cap = None
        self.last_roi = (0, 0, 0, 0)

    def open_camera(self):
        self.cap = cfg.open_camera()
        return self.cap.isOpened()

    def read(self):
        return self.cap.read()

    def close(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

    def locate_all(self, frame):
        from color_detect import detect_objects, detect_with_profiles
        out = []
        if self.profiles is not None:
            # 多 profile 共用同一个 A 区 ROI(取第一组的 ROI 当显示框)
            h, w = frame.shape[:2]
            self.last_roi = self._clamp_roi(self.profiles[0], w, h)
            objs, _masks = detect_with_profiles(frame, self.profiles)
            for o in objs:
                ax, ay = self.mapper.pixel_to_arm(o["cx"], o["cy"])
                out.append({
                    "cx": int(round(o["cx"])), "cy": int(round(o["cy"])),
                    "area": int(o["area"]),
                    "arm_x": round(ax, 1), "arm_y": round(ay, 1),
                    "name": o.get("name", ""),
                })
            return out
        # 单色
        objs, _mask, roi = detect_objects(frame, self.det_cfg)
        self.last_roi = roi
        rx, ry, _, _ = roi
        for o in objs:
            ax, ay = self.mapper.pixel_to_arm(o["cx"], o["cy"])
            cnt = o["contour"].copy()
            cnt[:, :, 0] += rx
            cnt[:, :, 1] += ry
            out.append({
                "cx": int(round(o["cx"])), "cy": int(round(o["cy"])),
                "area": int(o["area"]),
                "arm_x": round(ax, 1), "arm_y": round(ay, 1),
                "name": "", "contour": cnt,
            })
        return out


# ===================== 排序 / 佇列 / 协定 =====================
def _color_rank(name):
    try:
        return PICK_ORDER.index(name)
    except ValueError:
        return len(PICK_ORDER)        # 不在优先序里 → 排最后


def plan_picks(objects, place_points):
    """把侦测到的物件排序并配上放置点, 产生「夹取计划」清单。
    排序: 先颜色优先序(PICK_ORDER), 同色再由左到右(影像 cx 小的先)。
    每项: pick_x,pick_y(手臂夹取坐标), place_x,place_y(放置坐标或 None),
          color, cx,cy(供显示)。"""
    plan = []
    for o in objects:
        if o.get("arm_x") is None:
            continue
        color = o.get("name") or ""
        place = place_points.get(color)
        plan.append({
            "pick_x": o["arm_x"], "pick_y": o["arm_y"],
            "place_x": None if place is None else round(place[0], 1),
            "place_y": None if place is None else round(place[1], 1),
            "color": color, "cx": o["cx"], "cy": o["cy"],
        })
    plan.sort(key=lambda p: (_color_rank(p["color"]), p["cx"]))
    return plan


def build_message(item):
    """夹取计划项 → 回给手臂的字串。"""
    if item is None or item.get("pick_x") is None:
        return "$NONE\r\n"
    qx = "NA" if item["place_x"] is None else item["place_x"]
    qy = "NA" if item["place_y"] is None else item["place_y"]
    color = item["color"] or "NA"
    return f"${item['pick_x']},{item['pick_y']},{qx},{qy},{color}\r\n"


def get_latest_frame():
    with _frame_lock:
        if _latest["frame"] is None:
            return None
        return _latest["frame"].copy()


def do_scan(loc):
    """对当下这一帧侦测 A 区所有物件, 排序建佇列。回传数量。"""
    frame = get_latest_frame()
    objs = loc.locate_all(frame) if frame is not None else []
    plan = plan_picks(objs, PLACE_POINTS)
    with _plan_lock:
        _plan["queue"] = plan
        _plan["i"] = 0
        _plan["scanned"] = True
    print(f"  SCAN -> {len(plan)} objects queued: "
          + ", ".join(f"{p['color']}" for p in plan))
    return len(plan)


def get_next(loc):
    """吐佇列中下一个(若从未 SCAN 过, 自动先 SCAN 一次)。回传要送出的字串。"""
    with _plan_lock:
        scanned = _plan["scanned"]
    if not scanned:
        do_scan(loc)
    with _plan_lock:
        if _plan["i"] < len(_plan["queue"]):
            item = _plan["queue"][_plan["i"]]
            _plan["i"] += 1
        else:
            item = None
    return build_message(item)


def reset_plan():
    with _plan_lock:
        _plan["queue"] = []
        _plan["i"] = 0
        _plan["scanned"] = False


# ===================== TCP server =====================
def handle_client(conn, addr, loc):
    print(f"Arm connected: {addr}")
    conn.settimeout(RECV_TIMEOUT)
    try:
        buf = b""
        while True:
            try:
                data = conn.recv(1024)
            except socket.timeout:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode(errors="ignore").strip().upper()
                if not cmd:
                    continue
                if cmd == "SCAN":
                    n = do_scan(loc)
                    conn.sendall(f"$COUNT,{n}\r\n".encode())
                elif cmd == "GET":
                    msg = get_next(loc)
                    conn.sendall(msg.encode())
                    print(f"  GET -> {msg.strip()}")
                elif cmd == "RESET":
                    reset_plan()
                    conn.sendall(b"$OK\r\n")
                    print("  RESET")
                else:
                    conn.sendall(b"$ERR\r\n")
    except Exception as e:
        print(f"  [{addr}] error: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print(f"Arm disconnected: {addr}")


def serve_forever(loc):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    print(f"Waiting for arm on {HOST}:{PORT} ...")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client, args=(conn, addr, loc),
                         daemon=True).start()


# ===================== 画面绘制 =====================
def _largest(objects):
    best = None
    for o in objects:
        if best is None or o["area"] > best["area"]:
            best = o
    return best


def _draw_objects(view, objects, show_board_mm):
    big = _largest(objects)
    for obj in objects:
        cx, cy = obj["cx"], obj["cy"]
        is_big = obj is big
        color = (0, 0, 255) if is_big else (0, 200, 255)
        if "contour" in obj:
            cv2.drawContours(view, [obj["contour"]], -1, (0, 165, 255), 2)
        cv2.circle(view, (cx, cy), 7 if is_big else 5, color, -1)
        if obj.get("name"):
            cv2.putText(view, obj["name"], (cx + 10, cy - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if show_board_mm and obj.get("bx_mm") is not None:
            cell = obj.get("cell") or "off-board"
            cv2.putText(view, f"{cell} b({obj['bx_mm']:.0f},{obj['by_mm']:.0f})",
                        (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2)
        if obj.get("arm_x") is not None:
            cv2.putText(view, f"arm({obj['arm_x']:.0f},{obj['arm_y']:.0f})",
                        (cx + 10, cy + 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2)


def draw_charuco(view, loc, objects):
    if loc._pose is not None:
        rvec, R, t = loc._pose
        sq = cfg.SQUARE_LENGTH_M
        for c in range(loc.cols + 1):
            for r in range(loc.rows + 1):
                p = np.array([[c * sq, r * sq, 0.0]])
                px, _ = cv2.projectPoints(p, rvec, t, loc.K, loc.dist)
                cv2.circle(view, tuple(px.ravel().astype(int)), 2, (180, 180, 0), -1)
        cv2.drawFrameAxes(view, loc.K, loc.dist, rvec, t, sq * 2, 3)
    _draw_objects(view, objects, show_board_mm=True)


def draw_affine(view, loc, objects):
    rx, ry, rw, rh = loc.last_roi
    if rw > 0 and rh > 0:
        cv2.rectangle(view, (rx, ry), (rx + rw, ry + rh), (0, 165, 255), 2)
        cv2.putText(view, "A area (ROI)", (rx + 4, ry + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
    _draw_objects(view, objects, show_board_mm=False)


def display_loop(loc, draw_fn):
    win = "arm_server live"
    while True:
        ok, frame = loc.read()
        if not ok:
            continue
        with _frame_lock:
            _latest["frame"] = frame
        view = frame.copy()
        objects = loc.locate_all(frame)
        draw_fn(view, loc, objects)
        with _plan_lock:
            done, total = _plan["i"], len(_plan["queue"])
        cv2.putText(view, f"[{METHOD}] live objs:{len(objects)}  "
                    f"queue:{done}/{total}  (:{PORT})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(view, "q/ESC=quit", (10, view.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(win, view)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break
    loc.close()


# ===================== 主程式 =====================
def build_affine():
    loc = AffineLocator(VISION_CONFIG, AFFINE_CONFIG, PROFILES_CONFIG)
    if not loc.open_camera():
        print("Camera open failed")
        return None, None
    return loc, draw_affine


def build_charuco():
    import board_locator as bl
    loc = bl.BoardLocator()
    loc.load_detect_config(VISION_CONFIG)
    if not loc.open_camera():
        print("Camera open failed")
        return None, None
    print("Make sure the BOARD is visible, then press Enter...")
    input()
    ok, frame = loc.read()
    if not ok or not loc.lock(frame):
        print("Board lock failed")
        return None, None
    print("Board locked.")
    loc.set_arm_reference(ARM_ORIGIN, ARM_XEND, ARM_YEND)
    return loc, draw_charuco


def main():
    global PLACE_POINTS
    PLACE_POINTS = load_place_points(PLACE_CONFIG)
    if METHOD == "affine":
        loc, draw_fn = build_affine()
    elif METHOD == "charuco":
        loc, draw_fn = build_charuco()
    else:
        print(f"Unknown METHOD={METHOD!r}")
        return
    if loc is None:
        return
    threading.Thread(target=serve_forever, args=(loc,), daemon=True).start()
    display_loop(loc, draw_fn)


if __name__ == "__main__":
    main()
