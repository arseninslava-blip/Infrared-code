import pyb
import struct
import sensor
import time
import math

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

# Constants
YELLOW_THRESHOLD = (0, 100, -128, 127, 28, 127)
ORANGE_THRESHOLD = (53, 100, 7, 127, 10, 127)
BLUE_THRESHOLD = (0, 45, -128, 4, -128, -14)
THRESHOLDS = [YELLOW_THRESHOLD, ORANGE_THRESHOLD, BLUE_THRESHOLD]

_PACKET_LEN = 12

CENTER_X = 95
CENTER_Y = 95

# Независимые центры внутреннего и внешнего колец (по умолчанию совпадают с CENTER_X/Y)
INNER_CX, INNER_CY = 92, 86
OUTER_CX, OUTER_CY = CENTER_X, CENTER_Y

INNER_R = 30
OUTER_R = 95
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

_roi_x = OUTER_CX - OUTER_R
_roi_y = OUTER_CY - OUTER_R
_roi_w = OUTER_R * 2
_roi_h = OUTER_R * 2
BLOB_ROI = (_roi_x, _roi_y, _roi_w, _roi_h)

RAD_TO_DEG = 57.2957795

# Предрасчёт π для нормализации в calcDirect
_PI = math.pi
_TWO_PI = 2.0 * math.pi
_PI_NEG = -math.pi

# Line / exclusion-zone constants (были потеряны в исходном "оптимизированном" варианте)
EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R = INNER_CX, INNER_CY, 30
EXCLUDE_R2 = EXCLUDE_R * EXCLUDE_R

LINE_ROI_R = 38
LINE_ROI = (INNER_CX - LINE_ROI_R, INNER_CY - LINE_ROI_R, LINE_ROI_R * 2, LINE_ROI_R * 2)

LINE_MERGE_DISTANCE = 10
LINE_MAX_THETA_DIFF = 20
LINE_MIN_LENGTH = 30

# Зона столкновения с мячом (для ball_detect)
COLLIDE_ROI = (94, 62, 110, 80)

packet = bytearray(_PACKET_LEN)

_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (0, 128, 255)
_COL_W = (255, 255, 255)

# Цвета отладочной визуализации
_COL_RING = (0, 255, 0)
_COL_LINE_ROI = (255, 0, 255)
_COL_EXCLUDE = (255, 0, 0)
_COL_COLLIDE = (0, 255, 255)
_COL_SEG = (0, 255, 0)

# Показывать отладочную визуализацию колец/линий/зон
SHOW_DEBUG = True

orangeDirect = blueDirect = yellowDirect = outDirect = 0
orangeDist = blueDist = yellowDist = outDist = 0

ball_detect = 0

bus = None  # заполняется в __i2c__()

# System functions


def __init__():
    sensor.reset()
    sensor.set_pixformat(sensor.YUV422)
    sensor.set_framesize(sensor.HVGA)       # 480×320 — аппаратный буфер
    sensor.set_windowing(158, 17, 190, 185)   # Полное окно, без потери разрешения

    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.skip_frames(time=2000)


__init__()


def __i2c__():
    global bus
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
    dxi = x - INNER_CX
    dyi = y - INNER_CY
    if dxi * dxi + dyi * dyi <= INNER_R2:
        return False
    dxo = x - OUTER_CX
    dyo = y - OUTER_CY
    return dxo * dxo + dyo * dyo < OUTER_R2


def point_in_circle(x, y):
    dx = x - EXCLUDE_CX
    dy = y - EXCLUDE_CY
    return (dx * dx + dy * dy) <= EXCLUDE_R2


def segment_fully_in_circle(x1, y1, x2, y2):
    return (point_in_circle(x1, y1) and point_in_circle(x2, y2))


def getDistance(x, y):
    dx = x - CENTER_X
    dy = y - CENTER_Y
    r = dx * dx + dy * dy
    return max(math.sqrt(r) - INNER_R, 0)


def getCollide(x, y, _ROI=(0, 0, 0, 0)):
    if (_ROI[0] <= x <= _ROI[2] and _ROI[1] <= y <= _ROI[3]):
        return 1
    else:
        return 0

# Logik functions


def draw_debug(img, o_blob, y_blob, b_blob, line_point, line_segments):
    """Вся отладочная визуализация рисуется одним проходом в конце кадра."""
    # Кольцо поиска блобов (независимые центры внутреннего/внешнего радиуса)
    img.draw_circle(INNER_CX, INNER_CY, INNER_R, color=_COL_RING, thickness=1)
    img.draw_circle(OUTER_CX, OUTER_CY, OUTER_R, color=_COL_RING, thickness=1)

    # ROI поиска линий
    img.draw_rectangle(LINE_ROI, color=_COL_LINE_ROI, thickness=1)

    # Зона исключения сегментов линий (обычно совпадает с зоной мяча)
    img.draw_circle(EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R, color=_COL_EXCLUDE, thickness=1)

    # Зона столкновения с мячом (ball_detect)
    img.draw_rectangle(
        (COLLIDE_ROI[0], COLLIDE_ROI[1],
         COLLIDE_ROI[2] - COLLIDE_ROI[0], COLLIDE_ROI[3] - COLLIDE_ROI[1]),
        color=_COL_COLLIDE, thickness=1,
    )

    # Найденные сегменты линий
    for l in line_segments:
        img.draw_line(l.line(), color=_COL_SEG, thickness=2)

    # Блобы: крест по центру + рамка того же цвета
    if o_blob is not None:
        img.draw_cross(o_blob.cx(), o_blob.cy(), color=_COL_O, size=4)
        img.draw_rectangle(o_blob.rect(), color=_COL_O, thickness=1)

    if y_blob is not None:
        img.draw_cross(y_blob.cx(), y_blob.cy(), color=_COL_Y, size=4)
        img.draw_rectangle(y_blob.rect(), color=_COL_Y, thickness=1)

    if b_blob is not None:
        img.draw_cross(b_blob.cx(), b_blob.cy(), color=_COL_B, size=4)
        img.draw_rectangle(b_blob.rect(), color=_COL_B, thickness=1)

    if line_point is not None:
        img.draw_cross(line_point[0], line_point[1], color=_COL_W, size=4)


def find_all_lines(img):
    segments = img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    )

    counter = 0
    sum_x = sum_y = 0
    valid_segments = []
    for l in segments:
        if l.length() < LINE_MIN_LENGTH:
            continue
        x1, y1, x2, y2 = l.line()
        if segment_fully_in_circle(x1, y1, x2, y2):
            continue
        sum_x += (x1 + x2) // 2
        sum_y += (y1 + y2) // 2
        counter += 1
        valid_segments.append(l)

    point = (sum_x // counter, sum_y // counter) if counter else None
    return point, valid_segments


def main():
    global orangeDirect, blueDirect, yellowDirect, outDirect
    global orangeDist, blueDist, yellowDist, outDist
    global ball_detect

    clock = time.clock()

    while True:
        clock.tick()

        # do img with error protect
        try:
            img = sensor.snapshot()
            img.to_rgb565()
        except Exception as e:
            print("Snapshot error:", e)
            __init__()
            continue

        # logic
        line_point, line_segments = find_all_lines(img)
        seen_l = line_point is not None

        blobs = img.find_blobs(
            THRESHOLDS,
            pixels_threshold=1,
            area_threshold=1,
            x_stride=1,
            y_stride=1,
        )

        # ── Один проход: выбираем лучший блоб каждого цвета внутри кольца ─────────
        best_o_blob = best_y_blob = best_b_blob = None
        max_o = max_y = max_b = 0

        for b in blobs:
            bcx = b.cx()
            bcy = b.cy()
            if not pointInRing(bcx, bcy):
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
            ball_detect = getCollide(cx, cy, _ROI=COLLIDE_ROI)
            orangeDirect = normalization(getDirect(cy, cx))
            orangeDist = getDistance(cx, cy)

        if seen_y:
            cx, cy = best_y_blob.cx(), best_y_blob.cy()
            yellowDirect = normalization(getDirect(cy, cx))
            yellowDist = getDistance(cx, cy)

        if seen_b:
            cx, cy = best_b_blob.cx(), best_b_blob.cy()
            blueDirect = normalization(getDirect(cy, cx))
            blueDist = getDistance(cx, cy)

        if seen_l:
            lx, ly = line_point
            a = getDirect(ly, lx)
            outDirect = a + 360 if a < 0 else a
            outDist = getDistance(lx, ly)
        else:
            outDirect = -1

        # ── Вся отладочная отрисовка — одним проходом в конце кадра ───────────────
        if SHOW_DEBUG:
            draw_debug(img, best_o_blob, best_y_blob, best_b_blob, line_point, line_segments)

        struct.pack_into(
            '<hhhhhh', packet, 0,
            int(orangeDirect), int(blueDirect), int(yellowDirect),
            int(outDirect), int(orangeDist), int(ball_detect),
        )

        if ball_detect:
            trig.high()
        else:
            trig.low()

        try:
            bus.send(
                packet, timeout=10
            )  # Send the len first (16-bits).
        except OSError as err:
            pass  # Don't care about errors - so pass.

        print(orangeDirect, ball_detect, clock.fps())


if __name__ == "__main__":
    main()
