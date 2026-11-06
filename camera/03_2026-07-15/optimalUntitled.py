import pyb
import struct
import sensor
import image
import time
import math

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

# Constants
YELLOW_THRESHOLD = (0, 100, -32, 6, 22, 127)#(44, 83, -17, 4, 40, 68)
ORANGE_THRESHOLD = (0, 100, 23, 85, 4, 113)
BLUE_THRESHOLD   = (13, 37, -21, 5, -41, -21)#(20, 35, -13, 9, -30, -5)
THRESHOLDS       = [YELLOW_THRESHOLD, ORANGE_THRESHOLD, BLUE_THRESHOLD]

_PACKET_LEN = 12

CENTER_X = 93
CENTER_Y = 95

INNER_R  = 15
OUTER_R  = 95
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

_roi_x   = CENTER_X - OUTER_R
_roi_y   = CENTER_Y - OUTER_R
_roi_w   = OUTER_R * 2
_roi_h   = OUTER_R * 2
BLOB_ROI = (_roi_x, _roi_y, _roi_w, _roi_h)

RAD_TO_DEG = 57.2957795

# Предрасчёт π для нормализации в calcDirect
_PI     = math.pi
_TWO_PI = 2.0 * math.pi
_PI_NEG = -math.pi

# Info boxes

packet = bytearray(_PACKET_LEN)

_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (  0, 128, 255)
_COL_W = (255, 255, 255)

orangeDirect = blueDirect = yellowDirect = outDirect = 0
orangeDist = blueDust = yellowDist = outDist = 0

ball_detect = 0

# System functions

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

def __i2c__():
    bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
    bus.deinit()
    bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
__i2c__()

# Fast functions

def normalization(x):
    x = (x - 180) % 360
    if (x > 180):
        return x - 360
    return x

def getDirect(cx, cy):
    """Угол 0–359° от правой оси, CCW."""
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    a = math.atan2(dy, dx) * RAD_TO_DEG
    return int(a)

def pointInRing(x, y):
    dx = x - CENTER_X
    dy = y - CENTER_Y
    r2 = dx * dx + dy * dy
    return INNER_R2 < r2 < OUTER_R2

def point_in_circle(x, y):
    dx = x - CENTER_X
    dy = y - CENTER_Y
    return (dx * dx + dy * dy) <= EXCLUDE_R2

def segment_fully_in_circle(x1, y1, x2, y2):
    return (point_in_circle(x1, y1) and point_in_circle(x2, y2))

def getDistance(x, y):
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    r = dx ** dx + dy ** dy
    return max(math.sqrt(r) - INNER_R, 0)

def getCollide(x, y, _ROI=[0, 0, 0, 0]):
    if(_ROI[0] <= x <= _ROI[2] and _ROI[1] <= y <= _ROI[3]):
        return 1
    else:
        return 0

# Logik functions

def find_all_lines(img):
    #block 1
    segments = img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    )
    
    # block 2
    counter = 0
    sum_x = sum_y = 0
    for l in segments:
        if l.length() < LINE_MIN_LENGTH:
            continue
        x1, y1, x2, y2 = l.line()
        if segment_fully_in_circle(x1, y1, x2, y2):
            continue
        sum_x += (x1 + x2) // 2
        sum_y += (y1 + y2) // 2
        counter += 1

    # block 3
    #img.draw_cross(avg_x, avg_y, color=(255, 255, 255), size=8, thickness=2)
    if counter:
        return math.atan2(sum_y // counter - CENTER_Y, sum_x // counter - CENTER_X)
    return -1

def main():
    while True:
        # do img with error protect
        try:
            img = sensor.snapshot()
        except:
            __init__()
            continue

        # logic
        outDirect = find_all_lines(img)

        blobs = img.find_blobs(
            THRESHOLDS,
            roi=BLOB_ROI,
            pixels_threshold=5,
            area_threshold=5,
            merge=True,
            margin=5,
            x_stride=1,
            y_stride=1,
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
            ball_detect = getCollide(cx, cy, _ROI=[87, 68, 98, 70])
            orangeDirect = normalization(calculate_angle(cy, cx))
            img.draw_cross(cx, cy, color=_COL_O, size=8)
            orangeDist = getDistance(cx, cy)

        if seen_y:
            cx, cy = best_y_blob.cx(), best_y_blob.cy()
            yellowDirect = normalization(calculate_angle(cy, cx))
            img.draw_cross(cx, cy, color=_COL_Y, size=8)

        if seen_b:
            cx, cy = best_b_blob.cx(), best_b_blob.cy()
            blueDirect = normalization(calculate_angle(cy, cx))
            img.draw_cross(cx, cy, color=_COL_B, size=8)

        if(ball_detect):
            trig.high()
        else:
            trig.low()
        
        try:
            bus.send(packet, timeout=1)
        except OSError:
            pass


if __name__ == "__main__":
    main()