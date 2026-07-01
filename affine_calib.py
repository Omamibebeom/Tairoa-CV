"""
affine_calib.py —— 老師的「最小平方標定」: 像素 (cx,cy) → 手臂 (X,Y)

概念 (對應白板)
--------------
用手臂自己當尺。在桌面取 m 個點, 每個點同時得到:
  - 相機量到的像素中心 (cx, cy)   ← 由 color_detect.detect_objects() 給
  - 手臂尖端去碰那個點, 從達明顯示板讀到的座標 (X, Y)
把這些對應點丟進最小平方, 解出一個線性映射, 之後相機看到任何像素就能
直接換成手臂座標, 執行時不再需要棋盤板。

  仿射(affine, 白板版, 6 參數, 底列固定 0 0 1):
      X = R11*cx + R12*cy + R13
      Y = R21*cx + R22*cy + R23
  每個座標各是一條 3 參數線性迴歸。疊成 m 點的超定方程組 X = A·R,
  最小平方解就是白板上的 R = (AᵀA)⁻¹ Aᵀ X。
  (本檔用 numpy.lstsq 求解, 數學上等價於上式, 但數值更穩, 不必真的去求逆。)

  單應性(homography, 8 參數): 相機若有傾斜, 影像→平面其實是投影關係,
  affine 在邊緣會有系統性誤差。本檔會「順便」也擬合一個 homography 並印出
  它的殘差, 讓你直接比較: 若 homography 明顯比 affine 準, 代表相機歪了,
  那就把 SAVE_KIND 改成 "homography"。

座標系一致性 (重要)
------------------
取樣時相機量的 (cx,cy) 一律是「整張畫面」的像素 (color_detect 已加回 ROI
位移)。所以日後即使改 ROI, 這組標定也不會跑掉; 但相機本身一旦被移動,
整組標定就失效, 必須重新取樣。

操作 (互動取樣) —— 為避免「手臂進場被當成物件偵測到」, 採兩段式:
--------------
  1) 把要當基準的單一物件 / 校正針放到一個點, 畫面顯示偵測到的 (cx,cy)。
  2) 按 SPACE 鎖定該像素 → 偵測「凍結」, 此後畫面不再偵測 (手臂進來也不會被抓)。
  3) 把手臂尖端 Jog 到畫面上那個鎖定十字所在的同一物理點。
  4) 按 c, 到終端機輸入達明顯示板的 X,Y (例 712.3,55.8) → 完成這一點並自動解鎖。

  SPACE    鎖定目前物件像素 (凍結偵測)
  x        解除鎖定 (放棄這一點)
  c        輸入手臂 X,Y, 完成這一點
  u        刪掉上一點
  f        立即擬合, 印參數 + 殘差 RMS + 留一驗證 RMS
  s        存檔 affine_calib.json (+ affine_samples.json 原始點)
  q / ESC  離開

之後 arm_server 會用:
    from affine_calib import AffineMapper
    mapper = AffineMapper.from_json("affine_calib.json")
    arm_x, arm_y = mapper.pixel_to_arm(cx, cy)
"""
from __future__ import annotations
import json
from dataclasses import dataclass
import numpy as np

CONFIG_FILE = "affine_calib.json"
SAMPLES_FILE = "affine_samples.json"
SAVE_KIND = "affine"          # 要存哪種: "affine" (白板版) 或 "homography"
MIN_AFFINE = 3                # 仿射至少 3 個不共線點
MIN_HOMOG = 4                 # 單應性至少 4 點


# ========================= 純邏輯: 映射器 =========================
@dataclass
class AffineMapper:
    """像素 (cx,cy) → 手臂 (X,Y) 的線性映射。可存 / 讀 JSON。
    kind="affine"     params 形狀 (2,3)
    kind="homography" params 形狀 (3,3)
    """
    params: np.ndarray
    kind: str = "affine"
    rms: float = None          # 訓練殘差 RMS (mm)
    rms_loo: float = None      # 留一交叉驗證 RMS (mm), 估「對新點」的實際精度
    n: int = 0                 # 取樣點數

    # ---- 套用 ----
    def pixel_to_arm(self, cx, cy):
        p = self.params
        if self.kind == "affine":
            x = p[0, 0] * cx + p[0, 1] * cy + p[0, 2]
            y = p[1, 0] * cx + p[1, 1] * cy + p[1, 2]
            return float(x), float(y)
        # homography
        den = p[2, 0] * cx + p[2, 1] * cy + p[2, 2]
        if abs(den) < 1e-12:
            den = 1e-12
        x = (p[0, 0] * cx + p[0, 1] * cy + p[0, 2]) / den
        y = (p[1, 0] * cx + p[1, 1] * cy + p[1, 2]) / den
        return float(x), float(y)

    # ---- 擬合 ----
    @staticmethod
    def _fit_affine(pixels, arms):
        """解 X = A·R 的最小平方 (numpy.lstsq, 等價於 (AᵀA)⁻¹AᵀX)。"""
        m = len(pixels)
        A = np.column_stack([pixels[:, 0], pixels[:, 1], np.ones(m)])  # (m,3)
        rx, *_ = np.linalg.lstsq(A, arms[:, 0], rcond=None)   # [R11,R12,R13]
        ry, *_ = np.linalg.lstsq(A, arms[:, 1], rcond=None)   # [R21,R22,R23]
        return np.vstack([rx, ry])                            # (2,3)

    @staticmethod
    def _fit_homography(pixels, arms):
        import cv2
        H, _ = cv2.findHomography(pixels.astype(np.float64),
                                  arms.astype(np.float64), method=0)  # 0 = 全點最小平方
        if H is None:
            raise ValueError("findHomography 失敗 (點太少或共線?)")
        return H

    @classmethod
    def fit(cls, pixels, arms, kind="affine", compute_loo=True):
        pixels = np.asarray(pixels, float).reshape(-1, 2)
        arms = np.asarray(arms, float).reshape(-1, 2)
        n = len(pixels)
        need = MIN_HOMOG if kind == "homography" else MIN_AFFINE
        if n < need:
            raise ValueError(f"{kind} 至少需要 {need} 點, 目前只有 {n} 點")
        params = (cls._fit_homography(pixels, arms) if kind == "homography"
                  else cls._fit_affine(pixels, arms))
        m = cls(params=params, kind=kind, n=n)
        m.rms = m._rms(pixels, arms)
        m.rms_loo = cls._loo_rms(pixels, arms, kind) if compute_loo else None
        return m

    # ---- 殘差 ----
    def residuals(self, pixels, arms):
        """每個點的歐氏誤差 (mm)。"""
        pixels = np.asarray(pixels, float).reshape(-1, 2)
        arms = np.asarray(arms, float).reshape(-1, 2)
        errs = []
        for (cx, cy), (X, Y) in zip(pixels, arms):
            ax, ay = self.pixel_to_arm(cx, cy)
            errs.append(np.hypot(ax - X, ay - Y))
        return np.asarray(errs)

    def _rms(self, pixels, arms):
        e = self.residuals(pixels, arms)
        return float(np.sqrt(np.mean(e ** 2))) if len(e) else None

    @staticmethod
    def _loo_rms(pixels, arms, kind):
        """留一交叉驗證: 每次留 1 點不參與擬合, 拿它測誤差。
        反映『對沒看過的新點』的實際精度, 也能凸顯壞點。點不夠則回傳 None。"""
        need = MIN_HOMOG if kind == "homography" else MIN_AFFINE
        n = len(pixels)
        if n < need + 1:
            return None
        errs = []
        idx = np.arange(n)
        for i in range(n):
            tr = idx != i
            try:
                mi = AffineMapper.fit(pixels[tr], arms[tr], kind,
                                      compute_loo=False)
            except Exception:
                return None
            ax, ay = mi.pixel_to_arm(*pixels[i])
            errs.append(np.hypot(ax - arms[i, 0], ay - arms[i, 1]))
        return float(np.sqrt(np.mean(np.square(errs))))

    # ---- JSON ----
    def to_json(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump({
                "kind": self.kind,
                "params": self.params.tolist(),
                "rms": self.rms,
                "rms_loo": self.rms_loo,
                "n": self.n,
            }, f, ensure_ascii=False, indent=2)

    @classmethod
    def from_json(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        return cls(params=np.asarray(d["params"], float),
                   kind=d.get("kind", "affine"),
                   rms=d.get("rms"), rms_loo=d.get("rms_loo"),
                   n=d.get("n", 0))


# ========================= 互動取樣工具 =========================
def _report(pixels, arms):
    """擬合 affine 與 homography, 印出參數與殘差比較。回傳 (affine, homog 或 None)。"""
    n = len(pixels)
    print(f"\n===== 擬合結果 (n={n} 點) =====")
    aff = AffineMapper.fit(pixels, arms, "affine")
    print("[affine]  X = {:.5f}*cx + {:.5f}*cy + {:.3f}".format(*aff.params[0]))
    print("          Y = {:.5f}*cx + {:.5f}*cy + {:.3f}".format(*aff.params[1]))
    loo = "n/a" if aff.rms_loo is None else f"{aff.rms_loo:.2f}"
    print(f"          train RMS = {aff.rms:.2f} mm   留一驗證 RMS = {loo} mm")
    e = aff.residuals(pixels, arms)
    print("          每點誤差(mm): "
          + ", ".join(f"{v:.1f}" for v in e)
          + f"   (max {e.max():.1f})")

    hom = None
    if n >= MIN_HOMOG:
        try:
            hom = AffineMapper.fit(pixels, arms, "homography")
            hloo = "n/a" if hom.rms_loo is None else f"{hom.rms_loo:.2f}"
            print(f"[homog ]  train RMS = {hom.rms:.2f} mm   留一驗證 RMS = {hloo} mm")
            if hom.rms < aff.rms * 0.6:
                print("  → homography 明顯較準, 相機可能有傾斜; 考慮把 SAVE_KIND 設 'homography'。")
            else:
                print("  → affine 已足夠, 相機大致垂直。")
        except Exception as ex:
            print(f"[homog ]  擬合失敗: {ex}")
    else:
        print(f"[homog ]  點數 < {MIN_HOMOG}, 暫不比較。")
    return aff, hom


def _ask_arm_xy():
    """到終端機輸入手臂 X,Y。格式 'X,Y' 或 'X Y'。空白則取消。"""
    s = input("  輸入手臂座標 X,Y (例 712.3,55.8 ; 直接 Enter 取消): ").strip()
    if not s:
        return None
    s = s.replace(",", " ")
    try:
        x, y = (float(t) for t in s.split()[:2])
        return x, y
    except Exception:
        print("  格式不對, 略過這點")
        return None


def main():
    # 延遲 import: 讓 AffineMapper 能在無相機 / 無 GUI 環境被單獨 import 測試
    import cv2
    import board_config as cam
    from color_detect import DetectConfig, detect_objects

    # 載入 HSV/ROI 設定 (vision_tuner 存的)
    try:
        det_cfg = DetectConfig.from_json("vision_config.json")
        print("[calib] 已載入 vision_config.json")
    except Exception:
        det_cfg = DetectConfig()
        print("[calib] 找不到 vision_config.json, 用預設 HSV; 建議先跑 vision_tuner.py")

    cap = cam.open_camera()
    if not cap.isOpened():
        print("[calib] 開不了相機")
        return
    print(__doc__)

    pixels, arms = [], []      # 取樣點對
    locked_px = None           # 鎖定的物件像素 (cx,cy); 非 None 時「凍結偵測」
    last_mask = None
    win = "affine_calib (SPACE=lock, then bring arm in, c=enter X,Y)"

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        view = frame.copy()
        objs = []

        if locked_px is None:
            # --- 未鎖定: 正常偵測物件 (此時手臂還沒進場) ---
            objs, mask, (rx, ry, rw, rh) = detect_objects(frame, det_cfg)
            last_mask = mask
            cv2.rectangle(view, (rx, ry), (rx + rw, ry + rh), (0, 165, 255), 2)
            cur = objs[0] if objs else None
            if cur is not None:
                cx, cy = int(round(cur["cx"])), int(round(cur["cy"]))
                cv2.drawMarker(view, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
                cv2.putText(view, f"px=({cx},{cy})  area={cur['area']:.0f}",
                            (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)
            hint = "SPACE=lock object"
        else:
            # --- 已鎖定: 凍結偵測, 不再找物件 (手臂進場也不會被偵測到) ---
            cur = None
            cx, cy = locked_px
            cv2.drawMarker(view, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 26, 2)
            cv2.circle(view, (cx, cy), 10, (255, 255, 0), 2)
            cv2.putText(view, f"LOCKED px=({cx},{cy})  bring ARM here",
                        (cx + 12, cy - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 0), 2)
            hint = "c=enter arm X,Y   x=unlock"

        for (pcx, pcy) in pixels:                       # 已擷取的點畫綠圈
            cv2.circle(view, (int(pcx), int(pcy)), 6, (0, 255, 0), 2)

        head = (f"samples={len(pixels)}  "
                f"{'FROZEN' if locked_px is not None else f'objs={len(objs)}'}   "
                f"{hint}  u=undo f=fit s=save q=quit")
        cv2.putText(view, head, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        cv2.imshow(win, view)
        if last_mask is not None:
            cv2.imshow("mask", last_mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord(' '):                # SPACE: 鎖定目前物件像素
            if locked_px is not None:
                print("  已是鎖定狀態; 按 x 解除或 c 輸入手臂座標")
            elif cur is None:
                print("  畫面沒偵測到物件, 無法鎖定 (先把物件放好/調好 HSV)")
            else:
                locked_px = (int(round(cur["cx"])), int(round(cur["cy"])))
                print(f"  已鎖定像素 {locked_px} — 現在把手臂 Jog 到這一點 (偵測已凍結)")
        elif key == ord('x'):                # 解除鎖定 (取消這一點)
            if locked_px is not None:
                print("  已解除鎖定")
                locked_px = None
        elif key == ord('c'):                # 輸入手臂座標, 完成這一點
            if locked_px is None:
                print("  請先按 SPACE 鎖定物件, 再讓手臂進場, 然後按 c")
                continue
            cx, cy = float(locked_px[0]), float(locked_px[1])
            print(f"  鎖定像素 ({cx:.1f},{cy:.1f}) — 讀達明顯示板的 X,Y")
            axy = _ask_arm_xy()
            if axy is None:
                continue
            pixels.append((cx, cy))
            arms.append(list(axy))
            locked_px = None                 # 完成後自動解除, 進入下一點
            print(f"  已存第 {len(pixels)} 點: px=({cx:.1f},{cy:.1f}) arm={axy}")
        elif key == ord('u'):
            if pixels:
                pixels.pop(); arms.pop()
                print(f"  刪掉上一點, 剩 {len(pixels)} 點")
        elif key == ord('f'):
            if len(pixels) < MIN_AFFINE:
                print(f"  至少要 {MIN_AFFINE} 點才能擬合 (目前 {len(pixels)})")
            else:
                _report(np.array(pixels), np.array(arms))
        elif key == ord('s'):
            if len(pixels) < MIN_AFFINE:
                print(f"  至少要 {MIN_AFFINE} 點才能存 (目前 {len(pixels)})")
                continue
            aff, hom = _report(np.array(pixels), np.array(arms))
            chosen = hom if (SAVE_KIND == "homography" and hom is not None) else aff
            chosen.to_json(CONFIG_FILE)
            with open(SAMPLES_FILE, "w", encoding="utf-8") as f:
                json.dump({"pixels": pixels, "arms": arms}, f,
                          ensure_ascii=False, indent=2)
            print(f"  已存 {CONFIG_FILE} (kind={chosen.kind}) 與 {SAMPLES_FILE}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
