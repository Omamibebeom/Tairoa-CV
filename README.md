# 手指數量偵測 → 機械手臂取放 (TAIROA)

用攝影機數出手指數 (1~5),樹莓派 Pi 5 透過 GPIO/繼電器數位訊號指揮機械手臂
到對應取料點取放物件,並控制氣動軟夾爪充氣/洩氣。偵測到手勢後會**倒數 5 秒
鎖定**,作動期間視窗顯示**紅色警示**,完成後轉**綠色**才接受下一個手勢。

辨識:MediaPipe **Hand Landmarker**(21 關節點)+ 幾何規則數手指,不需訓練模型。

## 兩階段流程 (本課程的規劃)

- **階段一(座機):** 在接螢幕的工作站把 Pi 的程式與環境裝好,確認相機、數手指、
  倒數鎖定、紅綠視窗都正常。此時**還沒接繼電器/手臂**,所以把主程式設定區的
  `DRY_RUN` 設為 `True`(手臂動作用假的,不碰 GPIO)。
- **階段二(接硬體):** 把 Pi 拆下來接繼電器與手臂,先用 `relay_control.py` 確認
  繼電器,再把主程式的 `DRY_RUN` 改成 `False`,切換成真實控制。

## 檔案結構

```
Tairoa-CV/
├── README.md                ← 本說明
├── requirements.txt         ← 套件清單
├── .gitignore
├── hand_landmarker.task     ← 手部關節點模型 (已包含在 repo,不必另外下載)
├── gesture_pick_place.py    ← 主程式 (有 DRY_RUN 開關 + 紅綠視窗)
├── test_webcam.py           ← 筆電 webcam 測試,不需任何硬體 (手臂用假動作)
└── relay_control.py         ← 繼電器控制 + 互動測試 (單獨測繼電器/接線)
```

---

## A. 上傳到 GitHub

### 方式一:用網頁上傳 (最簡單)
1. 到 github.com 建立新 repository (不要勾 Add README)
2. 進入 repo → **Add file → Upload files**
3. 把資料夾裡的檔案全選拖進去 → **Commit changes**

### 方式二:用命令列
```bash
cd Tairoa-CV
git init
git add .
git commit -m "init: gesture pick and place"
git branch -M main
git remote add origin https://github.com/Omamibebeom/Tairoa-CV.git
git push -u origin main
```
> 模型檔約 7.8MB,在 GitHub 100MB 上限內,可直接推上去,Pi 端 clone 完就能跑。

---

## B. 在筆電上先測 (最快,不需任何硬體,連 Pi 都不用)

`test_webcam.py` 用筆電 webcam,手臂用假 sleep,可先把整個辨識+紅綠流程測順。
```bash
python -m venv venv
# Windows:  venv\Scripts\activate
# Mac/Linux: source venv/bin/activate
pip install -r requirements.txt
python test_webcam.py
```
中文橫幅需要中文字型;筆電上把 `FONT_PATH` 改成本機字型 (Windows
`C:/Windows/Fonts/msjh.ttc`,macOS `/System/Library/Fonts/PingFang.ttc`),找不到退回英文。

---

## C. 階段一:座機把 Pi 的程式與環境設好 (尚未接繼電器/手臂)

### 1) 燒系統
用 **Raspberry Pi Imager** 燒 **Raspberry Pi OS (64-bit) Bookworm**(不要 Trixie)。
燒錄前在進階設定開好 **SSH**、設定 **帳密** 與 **WiFi**。

### 2) 連進 Pi 並更新
```bash
ssh pi@pi5.local                      # 帳號換成你設的;連不上就改用 IP
sudo apt update && sudo apt full-upgrade -y
rpicam-hello --list-cameras           # CSI 相機有列出就 OK
sudo apt install -y fonts-noto-cjk    # 中文橫幅字型 (不裝會退回英文)
```

### 3) 下載並安裝
`picamera2` 與 `gpiozero` 在 Bookworm 已內建,建立 venv 要加 `--system-site-packages`:
```bash
git clone https://github.com/Omamibebeom/Tairoa-CV.git
cd Tairoa-CV
python -m venv --system-site-packages venv
source venv/bin/activate
pip install mediapipe opencv-python numpy pillow
```

### 4) 用 DRY_RUN 模式執行 (驗證辨識,不需硬體)
確認 `gesture_pick_place.py` 設定區是 `DRY_RUN = True`(預設就是),然後:
```bash
python gesture_pick_place.py
```
此時手臂/夾爪是假動作,畫面左上會標 `[DRY RUN - no hardware]`。

**重要:主程式會開視窗顯示紅綠狀態,要有畫面才看得到。** 請在 **接 HDMI 的 Pi 桌面**
或 **VNC**(`sudo raspi-config` → Interface → VNC 開啟)下執行。**純 SSH 沒畫面會報錯。**
這一階段要確認:相機畫面正常、手指數正確、倒數→紅色→綠色切換順暢。

---

## D. 階段二:接繼電器與手臂

### 1) 接線 (4-relay module → Pi 5)
```
繼電器模組        樹莓派 Pi 5
──────────        ──────────────────────
VCC           →   5V      (實體腳位 2)
GND           →   GND     (實體腳位 34)
IN1           →   GPIO5   (實體腳位 29)
IN2           →   GPIO6   (實體腳位 31)
IN3           →   GPIO13  (實體腳位 33)
IN4           →   GPIO16  (實體腳位 36)

繼電器輸出端 COM/NO → 接到手臂機櫃的數位輸入端子
(繼電器吸合時 COM-NO 導通 = 送訊號給手臂)
```
> 在 Pi 終端機打 `pinout` 可看完整腳位圖,從上面數到對應編號再插。

### 2) 先單獨測繼電器
```bash
python relay_control.py        # 進入互動模式
> on ch1                        # 應該聽到「喀」→ 代表接線正確
> off ch1
> pulse ch1                     # 喀一下就放開 (送脈衝)
> quit                          # ★ 一定要打 quit 結束,不要 Ctrl-Z
```
- 打 `on` 沒反應、`off` 反而吸合 → 低電位觸發模組,把 `relay_control.py` 的
  `ACTIVE_HIGH` 改成 `False`(主程式裡 OutputDevice 的 active_high 也要一致)
- 出現 `lgpio.error: GPIO busy` → GPIO 被別的程式佔住 (常因上次 Ctrl-Z 沒真的結束),
  `sudo reboot` 後重試;測繼電器時不要同時開著主程式
- 繼電器沒聲音先檢查:模組電源燈亮不亮、VCC 接 5V 不是 3.3V、JD-VCC 跳帽有沒有插

### 3) 切換成真實控制
把 `gesture_pick_place.py` 設定區的 `DRY_RUN` 改成 `False`,再執行:
```bash
python gesture_pick_place.py
```
這時就會用真實 GPIO 送訊號給繼電器/手臂。

> 安全建議:接真手臂前,先在 `PICK/PLACE/PUMP/VALVE` 腳位接 LED、在 `DONE` 腳位接
> 按鈕,跑一次完整流程驗證握手邏輯(看 LED 順序、按鈕當「手臂完成」),確認沒問題
> 再接真夾爪、真手臂。Pi 與手臂之間務必經繼電器/光耦合器隔離並共地。

---

## 設定速查

| 設定 | 位置 | 說明 |
|---|---|---|
| `DRY_RUN` | gesture_pick_place.py | True=階段一假動作 / False=階段二真實硬體 |
| `COUNTDOWN_SEC` | gesture_pick_place.py | 鎖定前倒數秒數 |
| `PICK_PINS` | gesture_pick_place.py | 手指數→GPIO腳位,要幾個點留幾個 |
| `ACTIVE_HIGH` | relay_control.py | 繼電器高/低電位觸發 |
| `FONT_PATH` | 兩支程式 | 中文字型路徑,找不到退回英文 |
