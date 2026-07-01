"""
board_config.py —— 共用設定中心(所有程式都 import 這個檔)

把「板子規格、相機設定、檔名」集中在這一個檔,其他程式都呼叫這裡的
函式,好處是:之後要改參數(例如換相機、換板子尺寸),只改這一個檔就好,
不用每個程式都去改。

【跨平台】本檔會依作業系統自動切換相機設定:
  - Windows：用 DirectShow(CAP_DSHOW)後端,並用「相機名稱」找編號(較穩)。
  - 樹莓派 / Linux：用 V4L2(CAP_V4L2)後端,直接用「相機編號」開啟
    (Linux 沒有 pygrabber,也沒有 DirectShow;USB 相機通常是 /dev/video0)。
  程式碼不用改,同一份在筆電與樹莓派都能跑。

你的板子規格:
  - 排列            : 5 x 7  (橫向5格 x 縱向7格)
  - 方格邊長        : 30 mm
  - marker 邊長     : 22 mm
  - 字典            : DICT_4X4_50
  - 列印解析度      : 300 dpi
"""
import sys
import cv2  # OpenCV:電腦視覺函式庫,所有影像/相機/ArUco 功能都來自它

# 是否為 Windows。其餘(樹莓派 Raspberry Pi OS、一般 Linux)都當 Linux 處理。
IS_WINDOWS = sys.platform == "win32"

# === 板子幾何參數 ===================================================
SQUARES_X = 5             # 板子橫向有幾格(寬)
SQUARES_Y = 7             # 板子縱向有幾格(高)
SQUARE_LENGTH_M = 0.030   # 每個方格邊長,單位「公尺」= 30mm。OpenCV 算姿態用公尺
MARKER_LENGTH_M = 0.022   # 每個 ArUco marker 邊長 = 22mm(印在白格裡的黑白圖案)
DICTIONARY_ID = cv2.aruco.DICT_4X4_50  # 用哪一本「marker 字典」,要跟你印的一致

# === 相機設定 =======================================================
# Windows 的相機「編號(index)」會跑掉:插個手機/平板當相機、開 OBS 虛擬鏡頭,
# 編號就會被擠動。所以 Windows 上我們不靠編號,改用「名稱」去找相機,比較穩。
# 樹莓派上沒有這個問題(pygrabber 也裝不起來),就直接用編號。
CAMERA_NAME = "C270"             # (僅 Windows 用)名稱的部分字串比對(你的是 C270 HD WEBCAM)
CAMERA_INDEX_FALLBACK = 0        # 相機編號:樹莓派通常是 0(/dev/video0);Windows 找不到名稱時的備援

# 相機後端:依平台自動選。
#   Windows → CAP_DSHOW(DirectShow,穩定,不會去碰會卡死的平板)
#   Linux/樹莓派 → CAP_V4L2(Video4Linux2,Linux 的標準相機介面)
CAMERA_BACKEND = cv2.CAP_DSHOW if IS_WINDOWS else cv2.CAP_V4L2

FRAME_WIDTH = 1280               # 要求相機輸出的畫面寬(C270 最高 1280)
FRAME_HEIGHT = 720               # 畫面高(C270 最高 720)

# === 檔名 ===========================================================
CALIB_FILE = "charuco_calib.npz"  # 標定結果(相機內參)存檔/讀檔的名稱


def find_camera_index(name=CAMERA_NAME):
    """(僅 Windows 用)依「名稱」找出相機的編號。
    回傳第一個名稱含有 name 的相機編號;找不到或沒裝 pygrabber 就回傳備用編號。
    樹莓派 / Linux 不會走到這裡(pygrabber 是 Windows 專用)。"""
    try:
        # pygrabber 能列出系統所有攝影機的「名稱」與順序(編號),只有 Windows 有
        from pygrabber.dshow_graph import FilterGraph
        devices = FilterGraph().get_input_devices()  # 例:['USB2.0...','C270...','Tab S9...']
        for i, dev in enumerate(devices):            # i = 編號, dev = 該相機名稱
            if name.lower() in dev.lower():          # 名稱裡有 "c270" 就是它
                return i                             # 找到 → 回傳這個編號
        # 跑到這裡代表清單裡沒有 C270
        print(f"[camera] '{name}' not found in {devices}; "
              f"using fallback index {CAMERA_INDEX_FALLBACK}")
    except Exception as e:
        # pygrabber 沒裝或出錯就走這裡,不讓程式整個掛掉
        print(f"[camera] name lookup failed ({e}); "
              f"using fallback index {CAMERA_INDEX_FALLBACK}")
    return CAMERA_INDEX_FALLBACK


def open_camera():
    """打開相機,設定好後端與解析度,回傳 cap(影像擷取物件)。
    Windows:先用名稱找編號;樹莓派 / Linux:直接用 CAMERA_INDEX_FALLBACK(通常 0)。"""
    if IS_WINDOWS:
        idx = find_camera_index()                    # Windows 才用名稱找 C270
    else:
        idx = CAMERA_INDEX_FALLBACK                  # 樹莓派 / Linux:直接用編號(0=/dev/video0)

    backend_name = "CAP_DSHOW" if IS_WINDOWS else "CAP_V4L2"
    print(f"[camera] opening index {idx} via {backend_name} "
          f"({'Windows' if IS_WINDOWS else 'Linux/RaspberryPi'})")

    cap = cv2.VideoCapture(idx, CAMERA_BACKEND)       # 用指定後端打開該編號的相機
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)    # 要求寬 1280
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)  # 要求高 720
    return cap                                        # 回傳給呼叫者去 read() 抓畫面


def get_dictionary():
    """回傳 ArUco 字典物件(把字典編號轉成實際可用的字典)。"""
    return cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)


def get_board():
    """依你的規格建立一個 ChArUco「板子模型」。
    這個物件知道每個方格/角點/marker 在板子上的真實 3D 座標(公尺),
    是後面算姿態、反投影的「標準答案」。"""
    return cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),  # (橫格數, 縱格數) = (5, 7)
        SQUARE_LENGTH_M,         # 方格邊長 0.03m
        MARKER_LENGTH_M,         # marker 邊長 0.022m
        get_dictionary(),        # 用哪本字典
    )


def get_detector():
    """建立「ChArUco 偵測器」,回傳 (detector, board)。
    detector 負責在影像裡找出 marker 與棋盤角點。"""
    board = get_board()                                  # 先有板子模型
    det_params = cv2.aruco.DetectorParameters()          # marker 偵測的參數設定
    # 開啟「次像素角點精修」:把角點位置算到小數點,姿態會更穩更準
    det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    charuco_params = cv2.aruco.CharucoParameters()        # ChArUco 專屬參數(用預設)
    # 把板子模型 + 兩組參數組成偵測器
    detector = cv2.aruco.CharucoDetector(board, charuco_params, det_params)
    return detector, board
