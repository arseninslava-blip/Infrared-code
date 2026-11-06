import pyb
import struct
import sensor
import image
import time
import math

# ═══════════════════════════════════════════════════════════════════════════════
#  ИНИЦИАЛИЗАЦИЯ КАМЕРЫ — OpenMV H7 R2
# ═══════════════════════════════════════════════════════════════════════════════
def __init__():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HVGA)       # 480×320 — аппаратный буфер
    sensor.set_windowing(170, 75, 190, 190)   # Полное окно, без потери разрешения

    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.skip_frames(time=2000)
__init__()

ball_detect = 0

# ═══════════════════════════════════════════════════════════════════════════════
#  КОНСТАНТЫ
# ═══════════════════════════════════════════════════════════════════════════════
CENTER_X = 93
CENTER_Y = 95

INNER_R  = 30
OUTER_R  = 90
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

_roi_x   = CENTER_X - OUTER_R
_roi_y   = CENTER_Y - OUTER_R
_roi_w   = OUTER_R * 2
_roi_h   = OUTER_R * 2
BLOB_ROI = (_roi_x, _roi_y, _roi_w, _roi_h)

RAD_TO_DEG = 57.2957795

# Предрасчёт констант для reloc — убираем повторные вычисления из цикла
_RELOC_Y0   =  7        # y-координата жёлтой точки в реальных координатах
_RELOC_B0   =  212      # y-координата синей точки
_RELOC_DY   =  _RELOC_Y0 - _RELOC_B0   # = -205, используется в числителе y
_RELOC_OUT_Y_OFFSET = 79  # смещение выходной y

# Предрасчёт π для нормализации в calcDirect
_PI     = math.pi
_TWO_PI = 2.0 * math.pi
_PI_NEG = -math.pi

# Цветовые пороги LAB
YELLOW_THRESHOLD = (0, 100, -32, 6, 22, 127)
ORANGE_THRESHOLD = (0, 100, 28, 91, 8, 116)
BLUE_THRESHOLD   = (0, 100, -128, 127, -128, -19)
THRESHOLDS       = [YELLOW_THRESHOLD, ORANGE_THRESHOLD, BLUE_THRESHOLD]

_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (  0, 128, 255)
_COL_W = (255, 255, 255)

# ═══════════════════════════════════════════════════════════════════════════════
#  I2C — SLAVE 0x12
#  Пакет (8 байт, little-endian):
#    [0:2] angle_o  uint16
#    [2:4] angle_y  uint16
#    [4:6] angle_b  uint16
#    [6]   flags    uint8   бит0=orange бит1=yellow бит2=blue
#    [7]   XOR-checksum
# ═══════════════════════════════════════════════════════════════════════════════
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit()
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)

packet = bytearray(12)

# ═══════════════════════════════════════════════════════════════════════════════
#  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ═══════════════════════════════════════════════════════════════════════════════

def calculate_angle(cx, cy):
    """Угол 0–359° от правой оси, CCW."""
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    a = math.atan2(dy, dx) * RAD_TO_DEG
    if a < 0:
        a += 360.0
    return int(a)

def blob_in_ring(cx, cy):
    """Проверка попадания точки в кольцо."""
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    r2 = dx * dx + dy * dy
    return INNER_R2 < r2 < OUTER_R2

# ═══════════════════════════════════════════════════════════════════════════════
#  СОСТОЯНИЕ
# ═══════════════════════════════════════════════════════════════════════════════
angle_o = angle_y = angle_b = 0
seen_o  = seen_y  = seen_b  = False

LINE_ROI = [61, 60, 62, 62]

# ⚠ ПОДСТАВЬТЕ реальные координаты окружности-исключения: центр (x, y) и радиус
EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R = 93, 94, 28
EXCLUDE_R2 = EXCLUDE_R * EXCLUDE_R

LINE_MERGE_DISTANCE = 5    # сегменты ближе этого расстояния объединяются
LINE_MAX_THETA_DIFF = 15   # макс. разница углов (град) для объединения сегментов
LINE_MIN_LENGTH     = 10    # отбрасываем совсем короткие обрывки (шум)

clock = time.clock()
print("OpenMV H7 R2 | %dx%d | ROI %s" % (sensor.width(), sensor.height(), BLOB_ROI))

# ═══════════════════════════════════════════════════════════════════════════════
#  ОСНОВНОЙ ЦИКЛ
# ═══════════════════════════════════════════════════════════════════════════════

def point_in_circle(x, y, cx, cy, r2):
    dx = x - cx
    dy = y - cy
    return (dx * dx + dy * dy) <= r2

def segment_fully_in_circle(x1, y1, x2, y2, cx, cy, r2):
    return (point_in_circle(x1, y1, cx, cy, r2)
            and point_in_circle(x2, y2, cx, cy, r2))

def sysAngle(a):
    x = (a - 180) % 360
    return x - 360 if x > 180 else x

def find_all_lines(img):
    segments = img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    )
    result = []
    for l in segments:
        if l.length() < LINE_MIN_LENGTH:
            continue
        x1, y1, x2, y2 = l.line()
        if segment_fully_in_circle(x1, y1, x2, y2, EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R2):
            continue
        result.append(l)
        img.draw_line(l.line(), color=(0, 255, 0), thickness=2)

    img.draw_circle(EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R, color=(255, 255, 255), thickness=1)

    sum_x = sum_y = 0
    for l in result:
        x1, y1, x2, y2 = l.line()
        sum_x += (x1 + x2) / 2
        sum_y += (y1 + y2) / 2
    k =  max(1,  len(result))
    avg_x = int(sum_x / k)
    avg_y = int(sum_y / k)
    img.draw_cross(avg_x, avg_y, color=(255, 255, 255), size=8, thickness=2)

    if (avg_x > 0) and (avg_y > 0):
        return math.atan2(avg_y - CENTER_Y, avg_x - CENTER_X)
    return -1

def pointInRect(rect=[83, 91, 81, 82], px, py):
    return (rect[0] <= px <= p[1]) and (rect[2] <= py <= rect[3])

while True:
    clock.tick()

    # ── Захват кадра с защитой от таймаута ────────────────────────────────────
    try:
        img = sensor.snapshot()
    except Exception as e:
        __init__()
        continue

    angle_l = find_all_lines(img)
    # ── Поиск блобов ──────────────────────────────────────────────────────────
    blobs = img.find_blobs(
        THRESHOLDS,
        roi=BLOB_ROI,
        pixels_threshold=5,
        area_threshold=5,
        merge=True,
        marin=5,
        x_stride=2,
        y_stride=2,
    )

    # ── Один проход: выбираем лучший блоб каждого цвета внутри кольца ─────────
    best_o_blob = best_y_blob = best_b_blob = None
    max_o = max_y = max_b = 0

    for b in blobs:
        bcx = b.cx()
        bcy = b.cy()
        if not blob_in_ring(bcx, bcy):
            continue
        code = b.code()
        area = b.area()
        if (code & 1) and area > max_y:
            max_y = area; best_y_blob = b
        if (code & 2) and area > max_o:
            max_o = area; best_o_blob = b
        if (code & 4) and area > max_b:
            max_b = area; best_b_blob = b

    # ── Обновление углов + визуализация ───────────────────────────────────────
    seen_o = best_o_blob is not None
    seen_y = best_y_blob is not None
    seen_b = best_b_blob is not None

    if seen_o:
        cx, cy = best_o_blob.cx(), best_o_blob.cy()
        ball_detect = pointInRect(cx, cy)
        angle_o = sysAngle(calculate_angle(cy, cx))
        img.draw_cross(cx, cy, color=_COL_O, size=8)

    if seen_y:
        cx, cy = best_y_blob.cx(), best_y_blob.cy()
        angle_y = sysAngle(calculate_angle(cy, cx))
        img.draw_cross(cx, cy, color=_COL_Y, size=8)
        pointY = (cx, cy)

    if seen_b:
        cx, cy = best_b_blob.cx(), best_b_blob.cy()
        angle_b = sysAngle(calculate_angle(cy, cx))
        img.draw_cross(cx, cy, color=_COL_B, size=8)

    # ── Упаковка и отправка пакета ────────────────────────────────────────────

    struct.pack_into('<HHHH', packet, 0, angle_o, angle_y, int(angle_l),  ball_detect)

    try:
        bus.send(packet, timeout=1)
    except OSError:
        pass