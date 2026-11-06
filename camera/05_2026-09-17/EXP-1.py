import pyb
import struct
import sensor
import time
import math

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

# Constants
YELLOW_THRESHOLD = (74, 100, -128, 127, 38, 127)
ORANGE_THRESHOLD = (59, 100, -5, 127, 15, 127)
BLUE_THRESHOLD = (0, 100, -128, 127, -128, -59)
THRESHOLDS = [YELLOW_THRESHOLD, ORANGE_THRESHOLD, BLUE_THRESHOLD]

_PACKET_LEN = 12

CENTER_X = 96
CENTER_Y = 93

INNER_R = 30
OUTER_R = 95
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

_roi_x = CENTER_X - OUTER_R
_roi_y = CENTER_Y - OUTER_R
_roi_w = OUTER_R * 2
_roi_h = OUTER_R * 2
BLOB_ROI = (_roi_x, _roi_y, _roi_w, _roi_h)

RAD_TO_DEG = 57.2957795

# Предрасчёт π для нормализации в calcDirect
_PI = math.pi
_TWO_PI = 2.0 * math.pi
_PI_NEG = -math.pi

# Line / exclusion-zone constants (были потеряны в исходном "оптимизированном" варианте)
LINE_ROI = [62, 61, 70, 75]

EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R = 95, 98, 35
EXCLUDE_R2 = EXCLUDE_R * EXCLUDE_R

LINE_MERGE_DISTANCE = 10
LINE_MAX_THETA_DIFF = 10
LINE_MIN_LENGTH = 30

# ── Треугольник pointY-pointMe-pointB (чисто в пиксельных координатах) ─────
# pointMe — фиксированная точка, где на кадре всегда находится сам робот
# (камера жёстко закреплена на шасси). По умолчанию = оптический центр
# кольца; если по факту робот на кадре не совпадает с CENTER_X/CENTER_Y —
# поправь эту константу отдельно, как в эталонном скрипте.
POINT_ME = (CENTER_X, CENTER_Y)

_RELOC_Y0 = 7.0      # y жёлтой точки
_RELOC_B0 = 212.0    # y синей точки
_RELOC_DY = _RELOC_Y0 - _RELOC_B0
_RELOC_OUT_Y_OFFSET = 79.0

packet = bytearray(_PACKET_LEN)

_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (0, 128, 255)
_COL_W = (255, 255, 255)

orangeDirect = blueDirect = yellowDirect = outDirect = 0
orangeDist = blueDist = yellowDist = outDist = 0

# Позиция, посчитанная по треугольнику pointY-pointMe-pointB. Компас не
# нужен: calcDirect берёт угол между двумя лучами из одной и той же
# точки в ОДНОМ кадре, курс робота в это не входит и сокращается сам.
pointY = (0, 0)
pointB = (0, 0)
posX = posY = 0.0
posValid = False

ball_detect = 0

bus = None  # заполняется в __i2c__()

# System functions


def __init__():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HVGA)       # 480×320 — аппаратный буфер
    sensor.set_windowing(160, 70, 200, 200)   # Полное окно, без потери разрешения

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
    dx = x - CENTER_X
    dy = y - CENTER_Y
    r2 = dx * dx + dy * dy
    return INNER_R2 < r2 < OUTER_R2


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


def calcDirect(A, B, C):
    """
    Знаковый угол в радианах [-π, π] от луча A→B до луча A→C.
    Положительный — CCW, отрицательный — CW.
    Возвращает радианы: именно их ожидает math.tan() в reloc().
    """
    ax, ay = A
    diff = math.atan2(C[1] - ay, C[0] - ax) - math.atan2(B[1] - ay, B[0] - ax)
    if diff > _PI:
        diff -= _TWO_PI
    elif diff < _PI_NEG:
        diff += _TWO_PI
    return diff


def reloc(YgoalD, BgoalD):
    """
    Вычисляет позицию (x, y) по двум угловым пеленгам (в радианах).
    YgoalD — угол от жёлтой точки (y=_RELOC_Y0)
    BgoalD — угол от синей точки (y=_RELOC_B0)
    Пересечение двух прямых: y = tan(a)*(x - x0) + y0
    """
    tanY = math.tan(YgoalD)
    tanB = math.tan(BgoalD)
    deltaTg = tanY - tanB
    if abs(deltaTg) < 1e-9:
        return (0, 0)
    x = (tanY * _RELOC_Y0 - tanB * _RELOC_B0) / deltaTg
    y = (tanY * tanB * _RELOC_DY) / deltaTg
    return (x, _RELOC_OUT_Y_OFFSET - y)

# Logik functions


def find_all_lines(img):
    segments = img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    )

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
        #img.draw_line(l.line(), color=(0, 255, 0), thickness=2)
    #img.draw_circle(EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R, color=(0, 255, 0), thickness=1)
    if counter:
        return (sum_x // counter, sum_y // counter)
    return None


def main():
    global orangeDirect, blueDirect, yellowDirect, outDirect
    global orangeDist, blueDist, yellowDist, outDist
    global ball_detect
    global pointY, pointB, posX, posY, posValid

    clock = time.clock()

    while True:
        clock.tick()

        # do img with error protect
        try:
            img = sensor.snapshot()
        except Exception as e:
            print("Snapshot error:", e)
            __init__()
            continue

        # logic
        line_point = find_all_lines(img)
        seen_l = line_point is not None

        blobs = img.find_blobs(
            THRESHOLDS,
            roi=BLOB_ROI,
            pixels_threshold=1,
            area_threshold=1,
            merge=False,
            #margin=5,
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
            ball_detect = getCollide(cx, cy, _ROI=(94, 62, 110, 80))
            orangeDirect = normalization(getDirect(cy, cx))
            img.draw_cross(cx, cy, color=_COL_O, size=8)
            orangeDist = getDistance(cx, cy)

        if seen_y:
            cx, cy = best_y_blob.cx(), best_y_blob.cy()
            yellowDirect = normalization(getDirect(cy, cx))
            img.draw_cross(cx, cy, color=_COL_Y, size=8)
            yellowDist = getDistance(cx, cy)
            pointY = (cx, cy)

        if seen_b:
            cx, cy = best_b_blob.cx(), best_b_blob.cy()
            blueDirect = normalization(getDirect(cy, cx))
            img.draw_cross(cx, cy, color=_COL_B, size=8)
            blueDist = getDistance(cx, cy)
            pointB = (cx, cy)

        if seen_l:
            lx, ly = line_point
            a = getDirect(ly, lx)
            outDirect = a + 360 if a < 0 else a
            img.draw_cross(lx, ly, color=_COL_W, size=8)
            outDist = getDistance(lx, ly)
        else:
            outDirect = -1

        # Позиция по треугольнику pointY-pointMe-pointB - только когда видны
        # обе метки сразу (иначе один из углов не посчитать).
        if seen_y and seen_b:
            dYMB = calcDirect(pointY, POINT_ME, pointB)  # угол Y->Me->B, если смотреть от Y
            dBMY = calcDirect(pointB, POINT_ME, pointY)  # угол B->Me->Y, если смотреть от B
            if abs(dYMB) < 1e-9 or abs(dBMY) < 1e-9:
                posValid = False
            else:
                # (PI/2 + |dBMY|) - в исходнике-образце тут стояло "90"
                # (градусы) вместо math.pi/2, хотя calcDirect отдаёт радианы -
                # та же ошибка единиц измерения, что уже ловили в reloc()
                # на плате. Поставил math.pi/2, чтобы не мешать градусы и
                # радианы в одной формуле.
                BgoalD = (_PI / 2.0 + abs(dBMY)) * (dYMB / abs(dYMB))
                posX, posY = reloc(dYMB, BgoalD)
                posValid = True
        else:
            posValid = False

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
                packet, timeout=15
            )  # Send the len first (16-bits).
        except OSError as err:
            pass  # Don't care about errors - so pass.

        if posValid:
            print("X=%.1f Y=%.1f" % (posX, posY))


if __name__ == "__main__":
    main()
