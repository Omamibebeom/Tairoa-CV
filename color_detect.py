"""
color_detect.py —— HSV 顏色過濾 + ROI 裁切 + 物件中心偵測 (純邏輯, 無 GUI)

設計給「紅 / 藍等與環境差異大的物件」, 取代原本灰階背景相減
(對光線敏感、深色物件輪廓易缺角) 的方式。HSV 把「色相(H)」和
「亮度(V)」分開, 只要 H、S 卡在物件顏色、V 範圍放寬, 就能容忍
桌面整體變亮/變暗, 比灰階差值穩很多。

本檔不含任何視窗 / trackbar, 可被 import 或在無顯示環境下做單元測試。
互動式調參請用 vision_tuner.py。

座標說明:
  偵測到的中心 (cx, cy) 一律回傳「整張原始畫面」的像素座標
  (已加回 ROI 位移)。這樣即使日後改 ROI, 下游『像素→手臂』標定的
  座標系也不會跑掉。另附 ROI 內相對座標 (cx_roi, cy_roi) 供顯示。
"""
from __future__ import annotations
import json
from dataclasses import dataclass, asdict
import cv2
import numpy as np


@dataclass
class DetectConfig:
    """所有可調參數集中在這。可存成 / 讀回 JSON。"""
    # --- ROI 裁切 (像素)。w 或 h <= 0 代表用整張畫面 ---
    roi_x: int = 0
    roi_y: int = 0
    roi_w: int = 0
    roi_h: int = 0
    # --- HSV 門檻 (OpenCV 範圍: H 0..179, S 0..255, V 0..255) ---
    h_min: int = 0
    h_max: int = 179
    s_min: int = 80     # 飽和度下限拉高 → 濾掉灰白/反光等低彩度雜訊
    s_max: int = 255
    v_min: int = 60     # 亮度下限 → 濾掉陰影; 上限放滿讓它耐亮度變化
    v_max: int = 255
    # --- 前處理 / 形態學 ---
    blur: int = 3       # 高斯模糊核 (0=不模糊; 偶數會自動 +1 變奇數)
    open_k: int = 3     # 開運算核 (0=不做): 先腐蝕後膨脹, 去掉小白點雜訊
    close_k: int = 5    # 閉運算核 (0=不做): 先膨脹後腐蝕, 補物件內部小破洞
    min_area: int = 800 # 最小輪廓面積 (像素), 比這小視為雜訊丟掉
    name: str = ""      # 這組設定的標籤 (多物件多色時用, 例: "red"/"blue")

    def to_json(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, ensure_ascii=False, indent=2)

    @classmethod
    def from_json(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        # 只取本類別有的欄位, 容忍 JSON 多/少欄位不致整個崩潰
        valid = {k: v for k, v in data.items()
                 if k in cls.__dataclass_fields__}
        return cls(**valid)


def _odd(k):
    """把核大小轉成 >=1 的奇數 (OpenCV 形態學/模糊核必須是奇數)。"""
    k = int(k)
    if k < 1:
        return 1
    return k if k % 2 == 1 else k + 1


def clamp_roi(cfg, frame_w, frame_h):
    """把 ROI 限制在畫面範圍內; w 或 h <= 0 時回傳整張畫面。
    回傳 (x, y, w, h)。"""
    if cfg.roi_w <= 0 or cfg.roi_h <= 0:
        return 0, 0, int(frame_w), int(frame_h)
    x = max(0, min(int(cfg.roi_x), frame_w - 1))
    y = max(0, min(int(cfg.roi_y), frame_h - 1))
    w = max(1, min(int(cfg.roi_w), frame_w - x))
    h = max(1, min(int(cfg.roi_h), frame_h - y))
    return x, y, w, h


def build_mask(roi_bgr, cfg):
    """對 ROI 影像做 HSV 過濾 + 形態學, 回傳二值遮罩 (uint8, 0/255)。

    自動處理「紅色跨界」: 紅色的 H 落在環的兩端 (接近 0 與接近 179)。
    當 h_min > h_max 時, 視為 [0, h_max] ∪ [h_min, 179] 兩段聯集,
    所以想抓紅色時把 H min 設大(如 170)、H max 設小(如 10) 即可。
    """
    img = roi_bgr
    if cfg.blur and cfg.blur > 0:
        k = _odd(cfg.blur)
        img = cv2.GaussianBlur(img, (k, k), 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if cfg.h_min <= cfg.h_max:
        lower = np.array([cfg.h_min, cfg.s_min, cfg.v_min], np.uint8)
        upper = np.array([cfg.h_max, cfg.s_max, cfg.v_max], np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
    else:
        lo1 = np.array([0,         cfg.s_min, cfg.v_min], np.uint8)
        up1 = np.array([cfg.h_max, cfg.s_max, cfg.v_max], np.uint8)
        lo2 = np.array([cfg.h_min, cfg.s_min, cfg.v_min], np.uint8)
        up2 = np.array([179,       cfg.s_max, cfg.v_max], np.uint8)
        mask = cv2.inRange(hsv, lo1, up1) | cv2.inRange(hsv, lo2, up2)

    if cfg.open_k and cfg.open_k > 0:
        k = _odd(cfg.open_k)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if cfg.close_k and cfg.close_k > 0:
        k = _odd(cfg.close_k)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_objects(frame_bgr, cfg):
    """主功能: 在 frame 的 ROI 內找出所有符合顏色的物件中心。

    回傳 (objects, mask, roi_box):
      objects : list[dict], 依面積由大到小排序, 每個含
          cx, cy         整張畫面的像素座標 (已加回 ROI 位移) ← 下游用這個
          cx_roi, cy_roi ROI 內相對座標 (畫在裁切畫面上用)
          area           輪廓面積 (像素)
          contour        輪廓點 (ROI 座標系, 顯示用)
      mask    : ROI 大小的二值遮罩 (顯示 / 除錯用)
      roi_box : (x, y, w, h) 實際使用的 ROI
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return [], np.zeros((1, 1), np.uint8), (0, 0, 0, 0)
    h, w = frame_bgr.shape[:2]
    rx, ry, rw, rh = clamp_roi(cfg, w, h)
    roi = frame_bgr[ry:ry + rh, rx:rx + rw]
    mask = build_mask(roi, cfg)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    objects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < cfg.min_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx_roi = M["m10"] / M["m00"]
        cy_roi = M["m01"] / M["m00"]
        objects.append({
            "cx": rx + cx_roi,
            "cy": ry + cy_roi,
            "cx_roi": cx_roi,
            "cy_roi": cy_roi,
            "area": float(area),
            "contour": cnt,
        })
    objects.sort(key=lambda o: o["area"], reverse=True)
    return objects, mask, (rx, ry, rw, rh)


# ===================== 多物件 / 多色 (profiles) =====================
# 一個「profile」就是一組 DetectConfig (含 name)。不同顏色的物件各調一組
# HSV, 但『像素→手臂』的 affine 幾何對所有顏色都一樣, 所以只要標定一次。

def load_profiles(path):
    """讀多組設定。相容兩種 JSON:
      (A) 單組 (vision_config.json 原格式) → 回傳含 1 個 DetectConfig 的 list
      (B) 多組  {"profiles": [ {..,"name":"red"}, {..,"name":"blue"} ]}
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict) and "profiles" in data:
        items = data["profiles"]
    else:
        items = [data]                       # 舊的單組格式
    profiles = []
    for i, d in enumerate(items):
        valid = {k: v for k, v in d.items()
                 if k in DetectConfig.__dataclass_fields__}
        cfg = DetectConfig(**valid)
        if not cfg.name:
            cfg.name = f"obj{i}"
        profiles.append(cfg)
    return profiles


def save_profiles(profiles, path):
    """把多組 DetectConfig 存成 {"profiles":[...]} 格式。"""
    out = {"profiles": [asdict(p) for p in profiles]}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)


def detect_with_profiles(frame_bgr, profiles, dedup_px=20):
    """對每個 profile 各跑一次偵測, 每個結果標上 profile 的 name。
    回傳 (objects, masks):
      objects : list[dict], 同 detect_objects 的欄位再加 "name" (來自哪個 profile),
                依面積由大到小排序。
      masks   : dict name -> 該 profile 的遮罩 (顯示/除錯用)
    去重: 若兩個偵測中心相距 < dedup_px (常因 HSV 範圍重疊), 只留面積較大者,
          避免同一個物體被兩組 profile 重複數到。
    """
    tagged, masks = [], {}
    for p in profiles:
        objs, mask, _roi = detect_objects(frame_bgr, p)
        masks[p.name] = mask
        for o in objs:
            o = dict(o)
            o["name"] = p.name
            tagged.append(o)
    tagged.sort(key=lambda o: o["area"], reverse=True)   # 大的優先保留
    kept = []
    for o in tagged:
        if any((o["cx"] - k["cx"]) ** 2 + (o["cy"] - k["cy"]) ** 2
               < dedup_px ** 2 for k in kept):
            continue
        kept.append(o)
    return kept, masks
