# TM 手臂視覺定位 + 氣動夾爪(筆電 + Arduino + L298N)

相機俯拍桌面,筆電偵測物件位置並換算成手臂座標;達明(TM)手臂透過網路向筆電
要「夾哪裡、放哪裡」,夾爪由筆電經 USB 序列驅動 Arduino + L298N 的充/洩氣馬達控制。

```
達明手臂 ── 網路線(TCP :5000)── 筆電 arm_server.py ── USB ── Arduino ── L298N ─┬─ 充氣馬達(夾)
                                        │                                        └─ 洩氣馬達(放)
                                     USB 相機(俯拍)
```

## 檔案地圖

| 檔案 | 角色 | 何時執行 |
|---|---|---|
| `arm_server.py` | 主程式:TCP server + 偵測 + 排序 + 夾爪指令 | 比賽/展示時常駐 |
| `gripper.py` | 夾爪控制模組(USB 序列 → Arduino) | 被 arm_server import;可單獨執行測試 |
| `gripper_arduino/gripper_arduino.ino` | Arduino 韌體(收指令、開關 L298N) | 燒錄一次 |
| `board_config.py` | 相機/板子共用設定(自動適配 Windows/Linux) | 被 import |
| `color_detect.py` | HSV/ROI 偵測核心 | 被 import |
| `affine_calib.py` | 像素→手臂座標標定工具 + AffineMapper | 賽前跑一次 |
| `vision_tuner.py` | 逐色調 HSV/ROI → vision_profiles.json | 賽前跑一次 |
| `teach_place.py` | 教 B 區放置點 → place_points.json | 賽前跑一次 |
| `tm_scripts/tm-flow-single.txt` | **單物件穩定版**(基線,已驗證) | 載入 TMflow |
| `tm_scripts/tm-flow-v3-multi.txt` | 多物件版(單次執行清空全部) | 載入 TMflow |
| `tm_scripts/tm-grip-test.txt` | 通訊隔離測試(不動手臂,只測 GRIP/RELEASE) | 除錯用 |

校正產物(`*.json`、`*.npz`)不在 repo 裡(.gitignore 排除),換環境必須重新產生。

## 安裝

```
git clone <你的 repo 網址>
cd tm-air-gripper
python -m venv .venv
.venv\Scripts\activate          # Windows(Linux: source .venv/bin/activate)
pip install -r requirements.txt
```

Arduino 端:用 Arduino IDE 燒錄 `gripper_arduino/gripper_arduino.ino`,
腳位依實際接線修改。**每個指令必須回一行 OK**(詳見 .ino 內註解)。

## 設定與校正(依序,換環境都要重做)

1. `gripper.py` 開頭:改 `PORT`(Windows 如 "COM6";裝置管理員可查)。
2. 單獨測夾爪:`python gripper.py` → 應充氣一次、洩氣一次,
   終端機顯示 GRIP/RELEASE 總耗時(應約 1.5~2.5 秒)。
3. `python vision_tuner.py` → 逐色調 HSV/ROI,存出 vision_profiles.json。
4. `python affine_calib.py` → 像素→手臂標定,存出 affine_calib.json。
5. `python teach_place.py` → 教每個顏色的放置點,存出 place_points.json。
6. TM 腳本裡的 Socket IP 改成筆電 IP;確認筆電與手臂同網段可互 ping。

## 執行

```
python arm_server.py        # 不要用 IDE 除錯器跑,直接用終端機
```

看到 `Waiting for arm on 0.0.0.0:5000` 後,在 TMflow 執行
`tm-flow-single`(每次夾一個;要清多個就讓專案重複執行此腳本)。

## 通訊協定(TCP :5000,訊息前置 `$`、後置 `\r\n`)

| 手臂送 | 筆電回 | 說明 |
|---|---|---|
| `SCAN` | `$COUNT,<n>` | 對當下畫面拍快照、排序、建佇列(手臂須在 A 區外) |
| `GET` | `$px,py,qx,qy,color` 或 `$NONE` | 佇列取下一個(夾取點+放置點) |
| `GRIP` | `$GRIPOK` | 充氣夾取;**完成後才回覆**(阻塞) |
| `RELEASE` | `$RELEOK` | 洩氣鬆開;完成後才回覆 |
| `RESET` | `$OK` | 清空佇列 |

夾取順序:依 `arm_server.py` 的 `PICK_ORDER`(預設 red→blue→green→yellow),
同色由畫面左至右。

## TM 腳本三大鐵律(除錯前先背)

1. **每個 sendline 都要配一個 socket_read_string 把回覆讀走**,
   否則回覆殘留在接收緩衝區,污染之後所有讀取(錯位)。
2. **運動指令不等手臂到位**(只是預約進佇列),
   送 GRIP/RELEASE 前必須 `WaitQueueTag()`。
3. **讀取逾時 > 伺服器處理耗時**。GRIP/RELEASE 用 8000ms;
   伺服器終端機會印「總耗時」,若把充氣秒數(gripper.py 的
   INFLATE_SEC)調大,TM 端逾時要跟著加大。

## 疑難排解

- 夾爪不動/方向相反:先 `python gripper.py` 單測;方向反了就把該馬達
  OUT 兩線對調。GRIP/RELEASE 總耗時若接近 5 秒以上,代表 Arduino
  沒回 OK(檢查 .ino)。
- 手臂顯示 no ack:跑 `tm_scripts/tm-grip-test.txt`(手臂不動,只測通訊),
  對照教導器的 ack=[...] 與伺服器時間戳輸出定位問題。
- 座標不準:重跑 affine_calib.py(相機動過必失準)。
- 相機打不開:board_config.py 的 CAMERA_NAME / CAMERA_INDEX_FALLBACK。
