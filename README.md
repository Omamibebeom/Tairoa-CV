# 手指數量偵測 → 機械手臂取放 (TAIROA)

用攝影機數出手指數 (1~5),樹莓派 Pi 5 透過 GPIO/繼電器數位訊號指揮機械手臂
到對應取料點取放物件,並控制氣動軟夾爪充氣/洩氣。偵測到手勢後會**倒數 5 秒
鎖定**,作動期間視窗顯示**紅色警示**,完成後轉**綠色**才接受下一個手勢。

辨識:MediaPipe **Hand Landmarker**(21 關節點)+ 幾何規則數手指,不需訓練模型。

## 檔案結構

```
TAIROA/
├── README.md                ← 本說明
├── requirements.txt         ← 套件清單
├── .gitignore
├── hand_landmarker.task     ← 手部關節點模型 (已包含在 repo,不必另外下載)
├── gesture_pick_place.py    ← 主程式,在 Pi 上接硬體執行 (有紅綠視窗)
├── test_webcam.py           ← 筆電 webcam 測試,不需任何硬體 (手臂用假動作)
└── relay_control.py         ← 繼電器控制 + 互動測試 (單獨測繼電器/接線)
```

---

## A. 上傳到 GitHub(在筆電的 TAIROA 資料夾裡)

```bash
git init
git add .
git commit -m "init: gesture pick and place"
git branch -M main
git remote add origin https://github.com/<你的帳號>/<repo名>.git
git push -u origin main
```

> 模型檔 `hand_landmarker.task` 約 7.8MB,在 GitHub 100MB 上限內,可以直接推上去,
> 這樣 Pi 端 clone 完就能跑、不必再下載模型。

---

## B. 在筆電上先測(建議先做,不需任何硬體)

`test_webcam.py` 用筆電 webcam,手臂部分用假的 sleep 代替,可以把
「數手指 + 倒數鎖定 + 紅綠切換」整個流程先測順。

```bash
python -m venv venv
# Windows:  venv\Scripts\activate
# Mac/Linux: source venv/bin/activate
pip install -r requirements.txt
python test_webcam.py
```

中文橫幅需要中文字型;筆電上把 `test_webcam.py` 的 `FONT_PATH` 改成本機字型
(Windows: `C:/Windows/Fonts/msjh.ttc`,macOS: `/System/Library/Fonts/PingFang.ttc`),
找不到會自動退回英文。

要測的情境:倒數中途改手指數 → 應以新數字重新倒數;倒數中途把手收掉 → 應變回綠色;
紅色作動期間比手勢 → 應被忽略且畫面不凍住。

---

## C. 在樹莓派 Pi 5 上執行

### 1) 燒系統
用 **Raspberry Pi Imager** 燒 **Raspberry Pi OS (64-bit) Bookworm**(不要 Trixie)。
燒錄前在進階設定開好 **SSH**、設定 **帳密** 與 **WiFi**。

### 2) 連進 Pi 並更新
```bash
ssh pi@pi5.local
sudo apt update && sudo apt full-upgrade -y
rpicam-hello --list-cameras          # CSI 相機有列出就 OK (USB webcam 則改用 cv2)
sudo apt install -y fonts-noto-cjk   # 中文橫幅字型 (不裝會退回英文)
```

### 3) 從 GitHub 下載並安裝
`picamera2` 與 `gpiozero` 在 Bookworm 已內建,建立 venv 要加
`--system-site-packages` 讓虛擬環境看得到它們:

```bash
git clone https://github.com/<你的帳號>/<repo名>.git
cd <repo名>/TAIROA            # 視你的 repo 結構,進到有 .py 的那層
python -m venv --system-site-packages venv
source venv/bin/activate
pip install mediapipe opencv-python numpy pillow
```

### 4) 執行
```bash
python gesture_pick_place.py
```

**重要:主程式會開一個視窗顯示紅綠狀態,所以要有畫面才看得到。**
請在 **接 HDMI 螢幕的 Pi 桌面** 或 **VNC**(`sudo raspi-config` → Interface → VNC 開啟)
下執行。**純 SSH 沒有畫面,`cv2.imshow` 會報錯。**

> 若想先用 SSH 無畫面測辨識邏輯,可把主程式 `cv2.imshow(...)` 那行先註解掉,
> 改看終端機的 `[流程]` 文字訊息。

執行前先改好 `gesture_pick_place.py` 設定區的腳位 (`PICK_PINS`、`PLACE_PIN`、
`DONE_PIN`、`PUMP_PIN`、`VALVE_PIN`)、倒數秒數 `COUNTDOWN_SEC`、充洩氣時間。

---

## D. 繼電器測試 (relay_control.py)

接到手臂機櫃前,先單獨測繼電器:
```bash
python relay_control.py        # 打 on ch1 / off ch1 / pulse ch1 / status / quit
```
打 `on` 沒反應、`off` 反而吸合,代表是低電位觸發模組,把 `ACTIVE_HIGH` 改成 `False`。
繼電器是乾接點 (COM/NO) 接進手臂機櫃數位輸入,電氣隔離,不必擔心 3.3V 對 24V。

---

## 建議測試順序(由軟到硬)

1. **筆電** 跑 `test_webcam.py`,確認辨識與紅綠流程。
2. **Pi(接螢幕/VNC)** 跑主程式,先註解 `imshow` 或先不接手臂,確認辨識。
3. **繼電器** 用 `relay_control.py` 確認吸合方向與接線。
4. **LED + 按鈕** 模擬手臂/夾爪,驗證整個握手邏輯,完全不會弄壞手臂。
5. **接真夾爪**,單獨測充洩氣時間。
6. **接真手臂**,Pi 與手臂之間經繼電器/光耦合器隔離並共地。

## 設定速查

| 設定 | 位置 | 說明 |
|---|---|---|
| `COUNTDOWN_SEC` | gesture_pick_place.py | 鎖定前倒數秒數 |
| `PICK_PINS` | gesture_pick_place.py | 手指數→GPIO腳位,要幾個點留幾個 |
| `ACTIVE_HIGH` | relay_control.py | 繼電器高/低電位觸發 |
| `FONT_PATH` | 兩支程式 | 中文字型路徑,找不到退回英文 |
