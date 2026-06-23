# 手指數量偵測 → 機械手臂取放 (TAIROA)

用攝影機數出手指數 (1~5),樹莓派 Pi 5 透過 GPIO/繼電器數位訊號指揮機械手臂
到對應取料點取放物件,並控制氣動軟夾爪充氣/洩氣。偵測到手勢後會**倒數 5 秒
鎖定**,作動期間視窗顯示**紅色警示**,完成後轉**綠色**才接受下一個手勢。

辨識:MediaPipe **Hand Landmarker**(21 關節點)+ 幾何規則數手指,不需訓練模型。

## 課程兩階段

| 階段 | 硬體 | 跑什麼 | 學生練到什麼 |
|---|---|---|---|
| **第一階段** | Pi + 螢幕 + 鍵盤 + 相機 | `gesture_pick_place.py`,設定 `FAKE_ARM = True` | 數手指、倒數鎖定、紅綠視窗切換(手臂用假動作,不接任何硬體) |
| **第二階段** | 第一階段 + 繼電器 + 手臂 + 夾爪 | 先 `relay_control.py` 確認接線,再 `gesture_pick_place.py` 設 `FAKE_ARM = False` | 繼電器隔離、GPIO 握手、真實取放 |

> **同一支主程式 `gesture_pick_place.py` 用 `FAKE_ARM` 開關切換兩階段**:
> 第一階段相機與辨識邏輯,跟第二階段完全一樣,只差手臂是真是假,所以第一階段
> 測順的東西到第二階段不會變。

## 檔案結構

```
Tairoa-CV/
├── README.md                ← 本說明
├── requirements.txt         ← pip 套件清單 (picamera2/gpiozero 是 Pi 系統套件,不在內)
├── .gitignore
├── hand_landmarker.task     ← 手部關節點模型 (已包含在 repo,不必另外下載)
├── gesture_pick_place.py    ← 主程式。FAKE_ARM=True 跑第一階段;=False 跑第二階段
├── test_webcam.py           ← (選做) 上 Pi 前的筆電預演,USB webcam + 假手臂
└── relay_control.py         ← (第二階段用) 繼電器控制 + 互動測試,單獨確認接線
```

---

## 預備(選做):先在筆電上預演

`test_webcam.py` 用筆電 webcam,手臂用假的 sleep 代替,可以**不必動 Pi**就先把
「數手指 + 倒數鎖定 + 紅綠切換」整個流程摸熟。手邊只有筆電、想先預習的人適合。
(正式第一階段是在 Pi 上跑,見下節。)

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

## 第一階段:樹莓派 + 相機,測手勢辨識

**目標:Pi 接好螢幕、鍵盤、相機,跑出辨識 + 倒數 + 紅綠視窗,完全不接手臂。**

### 1) 燒系統
用 **Raspberry Pi Imager** 燒 **Raspberry Pi OS (64-bit) Bookworm**(不要 Trixie)。
燒錄前在進階設定開好 **SSH**、設定 **帳密** 與 **WiFi**。

### 2) 從筆電連進 Pi 並更新
兩台接同一個 WiFi,在筆電終端機:
```bash
ssh pi@pi5.local                      # 帳號換成你設的;連不上就改用 IP
sudo apt update && sudo apt full-upgrade -y
rpicam-hello --list-cameras           # CSI 相機有列出就 OK
sudo apt install -y fonts-noto-cjk    # 中文橫幅字型 (不裝會退回英文)
```

### 3) 從 GitHub 下載並安裝
`picamera2` 與 `gpiozero` 在 Bookworm 已內建,建立 venv 要加
`--system-site-packages` 讓虛擬環境看得到它們:

```bash
git clone https://github.com/Omamibebeom/Tairoa-CV.git
cd Tairoa-CV
python -m venv --system-site-packages venv
source venv/bin/activate
pip install mediapipe opencv-python numpy pillow
```

### 4) 設成第一階段並執行
打開 `gesture_pick_place.py`,把設定區最上面的開關改成:
```python
FAKE_ARM = True        # 第一階段:只測辨識,不碰 GPIO/手臂
```
然後執行:
```bash
python gesture_pick_place.py
```
畫面上方應該出現 綠色(待機)→ 比手勢進入 黃色(倒數)→ 歸零後 紅色(假動作幾秒)
→ 回綠色。終端機會印 `[模擬] …` 訊息。**這時完全沒接繼電器或手臂也能跑。**

**重要:程式會開一個視窗顯示紅綠狀態,所以 Pi 要有畫面。**
請在 **接 HDMI 螢幕的 Pi 桌面** 或 **VNC**(`sudo raspi-config` → Interface → VNC 開啟)
下執行。**純 SSH 沒有畫面,`cv2.imshow` 會報錯。**

> - 用 **CSI 相機模組**:照上面直接跑(主程式用 Picamera2)。
> - 手邊只有 **USB webcam**:改跑 `test_webcam.py`(它用 `cv2.VideoCapture`)。
> - 想先用 SSH 無畫面測辨識邏輯:把主程式 `cv2.imshow(...)` 那行先註解掉,
>   改看終端機 `[模擬]` / `fingers:` 文字訊息。

要測的情境(同筆電預演):倒數中途改手指數 → 重新倒數;倒數中途把手收掉 → 變回綠色;
紅色期間比手勢 → 被忽略且畫面不凍住。

---

## 第二階段:接繼電器 → 控制手臂

第一階段辨識與紅綠流程都順了,再進這一段。Pi GPIO 是 3.3V,手臂機櫃可能 24V,
**中間一定要經繼電器/光耦合器隔離並共地**,切勿把手臂高壓直接接到 Pi。

### 2-1) 繼電器接線與測試 (relay_control.py)

先用這支單獨確認「觸發方向」與「每一路接線」,確認好再跑主程式。

接線圖 (4-relay module → Pi 5):
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
> 在 Pi 終端機打 `pinout` 可看完整腳位圖。

```bash
python relay_control.py        # 進入互動模式
> on ch1                        # 應該聽到「喀」→ 代表接線正確
> off ch1
> pulse ch1                     # 喀一下就放開 (送脈衝)
> status                        # 看所有通道狀態
> quit                          # ★ 一定要打 quit 結束,不要 Ctrl-Z
```

- 打 `on` 沒反應、`off` 反而吸合 → 低電位觸發模組,把 `ACTIVE_HIGH` 改成 `False`
- 出現 `GPIO busy` → GPIO 被別的程式佔住,`sudo reboot` 後重試
- 繼電器是乾接點 (COM/NO) 接進手臂機櫃數位輸入,電氣隔離,不必擔心 3.3V 對 24V

> `relay_control.py` 預設只列 4 路做接線練習。完整系統用到的路數更多
> (各取料點 `PICK_PINS`、放置 `PLACE_PIN`、氣泵 `PUMP_PIN`、電磁閥 `VALVE_PIN`),
> 全都定義在 `gesture_pick_place.py` 設定區,實際接線要照那邊的腳位。

### 2-2) 設定主程式腳位,改成第二階段並執行

打開 `gesture_pick_place.py`,改設定區:
```python
FAKE_ARM = False       # 第二階段:送真實 GPIO 訊號給手臂與夾爪
```
並確認這些腳位和你實際接線一致:`PICK_PINS`(手指數→取料點)、`PLACE_PIN`、
`DONE_PIN`(手臂回傳完成的輸入腳)、`PUMP_PIN`、`VALVE_PIN`,以及倒數秒數
`COUNTDOWN_SEC` 與充洩氣時間 `INFLATE_SEC` / `DEFLATE_SEC`。

```bash
python gesture_pick_place.py
```
鎖定手勢後,終端機會印 `[流程] …`,Pi 送脈衝給手臂、等 `DONE` 拉高,再控制夾爪充洩氣。

### 2-3) 建議的硬體接入順序(由軟到硬,保護手臂)

1. **繼電器** 用 `relay_control.py` 確認吸合方向與接線。
2. **LED + 按鈕** 接在繼電器輸出/輸入端,模擬手臂與 DONE 訊號,驗證整個握手邏輯,
   完全不會弄壞手臂。
3. **接真夾爪**,單獨測充洩氣時間 (`INFLATE_SEC` / `DEFLATE_SEC`)。
4. **接真手臂**,Pi 與手臂之間經繼電器/光耦合器隔離並共地,再跑 `FAKE_ARM=False`。

---

## 設定速查

| 設定 | 位置 | 說明 |
|---|---|---|
| `FAKE_ARM` | gesture_pick_place.py | **階段開關**。True=第一階段(只測辨識);False=第二階段(接手臂) |
| `COUNTDOWN_SEC` | gesture_pick_place.py | 鎖定前倒數秒數 |
| `FAKE_RUN_SEC` | gesture_pick_place.py | 第一階段假動作維持秒數(紅色橫幅多久) |
| `PICK_PINS` | gesture_pick_place.py | 手指數→GPIO腳位,要幾個點留幾個 |
| `PLACE_PIN` / `DONE_PIN` | gesture_pick_place.py | 放置點輸出 / 手臂完成輸入 |
| `PUMP_PIN` / `VALVE_PIN` | gesture_pick_place.py | 氣泵 / 電磁閥 |
| `INFLATE_SEC` / `DEFLATE_SEC` | gesture_pick_place.py | 充氣 / 洩氣時間 |
| `ACTIVE_HIGH` | relay_control.py | 繼電器高/低電位觸發 |
| `FONT_PATH` | 兩支程式 | 中文字型路徑,找不到退回英文 |

---

## 附錄:上傳到 GitHub

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

> 模型檔 `hand_landmarker.task` 約 7.8MB,在 GitHub 100MB 上限內,可以直接推上去,
> 這樣 Pi 端 clone 完就能跑、不必再下載模型。
