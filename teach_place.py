"""
teach_place.py — 教 B 区(放置区)的指定点, 存成 place_points.json

place_points.json 格式: {"red":[x,y], "blue":[x,y], ...}  (手臂坐标 mm)
arm_server.py 会读它, 让每个颜色的物件被放到对应的手臂坐标。

==============================================================================
两种教点方式都支持(你说两个都想试)
==============================================================================
方式 J(直接较点 / jog-teach)—— 推荐, 最准
    指定点是固定已知的, 直接把手臂尖端 Jog 到放置点, 读达明显示板的 X,Y,
    输入颜色 + 坐标即可。不经过相机, 没有视觉误差。
    因为手臂固定、点固定, 教一次重复用。

方式 D(相机侦测 / detect)—— 用同一个 affine
    在放置点摆一个该颜色的物件 / 标记, 相机侦测它的像素中心,
    用「同一个」affine(affine_calib.json)换算成手臂坐标。
    不需要另做标定。但前提: affine 取样时要涵盖到 B 区, 否则是外推会不准;
    且同平面同高度。教出来的坐标会和 J 方式一样落在同一个手臂坐标系。

两种存进同一个 place_points.json, 你可以混用、互相比对。

==============================================================================
操作
==============================================================================
  画面会用 HSV/ROI 即时侦测(读 vision_profiles.json 或 vision_config.json),
  侦测到的最大物件会显示像素与(经 affine 的)手臂坐标。

  j   方式 J: 终端机输入「颜色 X Y」(例: red 840 120) → 写入该颜色放置点
  d   方式 D: 把当前侦测到的物件当放置点 → 终端机输入颜色(多 profile 会自动带颜色)
              → 用 affine 换算手臂坐标并写入
  l   列出目前 place_points 表
  s   存档 place_points.json
  x   删除某颜色(终端机输入颜色名)
  q / ESC  离开(离开前会提醒尚未存档)
"""
import os
import json
import cv2

import board_config as cam
from color_detect import (DetectConfig, detect_objects, detect_with_profiles,
                          load_profiles)
from affine_calib import AffineMapper

PLACE_FILE = "place_points.json"
PROFILES_FILE = "vision_profiles.json"
VISION_FILE = "vision_config.json"
AFFINE_FILE = "affine_calib.json"


def load_existing():
    if os.path.exists(PLACE_FILE):
        try:
            with open(PLACE_FILE, "r", encoding="utf-8") as f:
                d = json.load(f)
            return {k: [float(v[0]), float(v[1])] for k, v in d.items()}
        except Exception as e:
            print(f"[teach] 读 {PLACE_FILE} 失败 ({e}); 从空表开始")
    return {}


def save_table(table):
    with open(PLACE_FILE, "w", encoding="utf-8") as f:
        json.dump(table, f, ensure_ascii=False, indent=2)
    print(f"[teach] 已存 {PLACE_FILE}: "
          + ", ".join(f"{k}{tuple(v)}" for k, v in table.items()))


def detect_current(frame, profiles, det_cfg):
    """回传最大物件 dict(含 cx,cy,name) 或 None。"""
    if profiles is not None:
        objs, _ = detect_with_profiles(frame, profiles)
        objs = sorted(objs, key=lambda o: o["area"], reverse=True)
        return objs[0] if objs else None
    objs, _, _ = detect_objects(frame, det_cfg)
    return objs[0] if objs else None


def main():
    # 偵測设定: 优先多 profile
    profiles = None
    det_cfg = None
    if os.path.exists(PROFILES_FILE):
        try:
            profiles = load_profiles(PROFILES_FILE)
            print(f"[teach] 多颜色侦测: {', '.join(p.name for p in profiles)}")
        except Exception as e:
            print(f"[teach] 读 {PROFILES_FILE} 失败 ({e})")
    if profiles is None:
        try:
            det_cfg = DetectConfig.from_json(VISION_FILE)
        except Exception:
            det_cfg = DetectConfig()
        print("[teach] 单色侦测")

    # affine(方式 D 才需要)
    mapper = None
    try:
        mapper = AffineMapper.from_json(AFFINE_FILE)
        print(f"[teach] 已载入 affine({mapper.kind}); 方式 D 可用")
    except Exception:
        print("[teach] 找不到 affine_calib.json; 只能用方式 J(直接较点)")

    cap = cam.open_camera()
    if not cap.isOpened():
        print("[teach] 开不了相机")
        return
    table = load_existing()
    dirty = False
    print(__doc__)
    win = "teach_place"

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        view = frame.copy()
        cur = detect_current(frame, profiles, det_cfg)
        if cur is not None:
            cx, cy = int(round(cur["cx"])), int(round(cur["cy"]))
            cv2.drawMarker(view, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            label = cur.get("name", "") or "?"
            txt = f"{label} px=({cx},{cy})"
            if mapper is not None:
                ax, ay = mapper.pixel_to_arm(cx, cy)
                txt += f" arm=({ax:.0f},{ay:.0f})"
            cv2.putText(view, txt, (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # 已教的点列表
        y0 = 30
        cv2.putText(view, f"placed: {len(table)}   j=jog d=detect l=list s=save x=del q=quit",
                    (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        for i, (k, v) in enumerate(table.items()):
            cv2.putText(view, f"{k}: ({v[0]:.0f},{v[1]:.0f})",
                        (10, y0 + 24 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2)
        cv2.imshow(win, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            if dirty:
                print("[teach] 尚未存档! 按 s 存档, 或再按一次 q 强制离开")
                dirty = False        # 下一次 q 才离开
                continue
            break
        elif key == ord('j'):        # 方式 J: 直接较点
            s = input("  方式J 输入「颜色 X Y」(例 red 840 120; Enter 取消): ").strip()
            if not s:
                continue
            s = s.replace(",", " ")
            parts = s.split()
            if len(parts) < 3:
                print("  格式不对")
                continue
            color = parts[0]
            try:
                x, y = float(parts[1]), float(parts[2])
            except ValueError:
                print("  坐标不是数字")
                continue
            table[color] = [round(x, 1), round(y, 1)]
            dirty = True
            print(f"  [J] {color} -> ({x:.1f},{y:.1f})")
        elif key == ord('d'):        # 方式 D: 相机侦测 + affine
            if mapper is None:
                print("  没有 affine, 无法用方式 D")
                continue
            if cur is None:
                print("  画面没侦测到物件")
                continue
            color = cur.get("name", "")
            if not color:
                color = input("  这个放置点的颜色名 (Enter 取消): ").strip()
                if not color:
                    continue
            ax, ay = mapper.pixel_to_arm(cur["cx"], cur["cy"])
            table[color] = [round(ax, 1), round(ay, 1)]
            dirty = True
            print(f"  [D] {color} -> arm({ax:.1f},{ay:.1f})  (相机+affine)")
        elif key == ord('l'):
            print("  目前 place_points:",
                  ", ".join(f"{k}{tuple(v)}" for k, v in table.items()) or "(空)")
        elif key == ord('x'):
            color = input("  要删除的颜色名: ").strip()
            if color in table:
                table.pop(color)
                dirty = True
                print(f"  已删除 {color}")
            else:
                print("  没有这个颜色")
        elif key == ord('s'):
            save_table(table)
            dirty = False

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
