import pyb
import struct
import sensor
import time
import math

# EXP-4-V: вектор (x, y) в системе робота, длина = дистанция (px).
# Пакет I2C 18 байт LE: 8×int16 (ball, yellow, blue, line) + uint8 flags + uint8 checksum.

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

YELLOW_THRESHOLD = (0, 100, -128, 127, 42, 127)
ORANGE_THRESHOLD = (0, 100, 17, 127, 25, 127)
BLUE_THRESHOLD   = (0, 100, -128, 127, -128, -16)
THRESHOLDS = [YELLOW_THRESHOLD, ORANGE_THRESHOLD, BLUE_THRESHOLD]

_PACKET_LEN = 18
VEC_SCALE = 100

FLAG_BALL        = 1
FLAG_YELLOW      = 2
FLAG_BLUE        = 4
FLAG_LINE        = 8
FLAG_BALL_DETECT = 16

CENTER_X = 100
CENTER_Y = 98

INNER_CX, INNER_CY = 98, 97
OUTER_CX, OUTER_CY = CENTER_X, CENTER_Y

INNER_R = 32
OUTER_R = 98
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

_roi_x = OUTER_CX - OUTER_R
_roi_y = OUTER_CY - OUTER_R
_roi_w = OUTER_R * 2
_roi_h = OUTER_R * 2
BLOB_ROI = (_roi_x, _roi_y, _roi_w, _roi_h)

RAD_TO_DEG = 57.2957795
_PI = math.pi
_TWO_PI = 2.0 * math.pi
_PI_NEG = -math.pi

EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R = INNER_CX, INNER_CY, 33
EXCLUDE_R2 = EXCLUDE_R * EXCLUDE_R

LINE_ROI_R = 42
LINE_ROI = (INNER_CX - LINE_ROI_R, INNER_CY - LINE_ROI_R, LINE_ROI_R * 2, LINE_ROI_R * 2)

LINE_MERGE_DISTANCE = 10
LINE_MAX_THETA_DIFF = 35
LINE_MIN_LENGTH = 15
LINE_MAX_INSIDE = 1

COLLIDE_ROI = (-1, -1, -1, -1)

packet = bytearray(_PACKET_LEN)

_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (0, 128, 255)
_COL_W = (255, 255, 255)
_COL_RING      = (0, 255, 0)
_COL_LINE_ROI  = (255, 0, 255)
_COL_EXCLUDE   = (255, 0, 0)
_COL_COLLIDE   = (0, 255, 255)
_COL_SEG       = (0, 255, 0)
_COL_LINK      = (255, 255, 255)
_COL_OLD       = (64, 224, 208)

SHOW_DEBUG = True
LINE_EVERY = 2
PRINT_INTERVAL_MS = 500
PROFILE = True

X_RANGE = 110
Y_RANGE = 80
USE_BLOB_EDGE = True
SHOW_OLD_CENTERS = False

ballVec = yellowVec = blueVec = lineVec = (0.0, 0.0)
ball_detect = 0
bus = None

# ------------------------------------------------------------------ system ---

def __init__():
    sensor.reset()
    sensor.set_pixformat(sensor.YUV422)
    sensor.set_framesize(sensor.HVGA)
    sensor.set_windowing(158, 80, 200, 200)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=10000)
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

# ------------------------------------------------------------------- fast ----

def toVector(x, y, length, flip=True):
    """
    Вектор из точки (x, y) кадра в систему робота, длина = length.
    Направление совпадает с прежним getDirect(y, x)+normalization().
    Вместо atan2/cos/sin — один sqrt (в 2-3 раза быстрее).
    """
    ux = y - CENTER_X
    uy = x - CENTER_Y
    if flip:
        ux = -ux
        uy = -uy
    r = math.sqrt(ux * ux + uy * uy)
    if r == 0.0:
        return (0.0, 0.0)
    s = length / r
    return (ux * s, uy * s)


def vecFromBlobPoint(x, y):
    """
    Вектор мяча/ворот: length = getDistance(x, y), flip=True.
    Совмещает getDistance + toVector, экономя один вызов sqrt для каждого объекта.
    """
    ddx = x - CENTER_X
    ddy = y - CENTER_Y
    dist = math.sqrt(ddx * ddx + ddy * ddy) - INNER_R
    if dist < 0.0:
        dist = 0.0

    ux = -(y - CENTER_X)      # flip: ux = -(y - CENTER_X)
    uy = -(x - CENTER_Y)      # flip: uy = -(x - CENTER_Y)
    r = math.sqrt(ux * ux + uy * uy)
    if r == 0.0:
        return (0.0, 0.0)
    s = dist / r
    return (ux * s, uy * s)


def _i16(v):
    v = int(round(v * VEC_SCALE))
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return v


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


def segment_invalid(x1, y1, x2, y2, mx, my):
    inside = 0
    if point_in_circle(x1, y1):
        inside += 1
    if point_in_circle(x2, y2):
        inside += 1
    if point_in_circle(mx, my):
        inside += 1
    return inside > LINE_MAX_INSIDE


def segmentRadius(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    dd = dx * dx + dy * dy
    t = 0.0
    if dd:
        t = ((CENTER_X - x1) * dx + (CENTER_Y - y1) * dy) / dd
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
    px = x1 + t * dx - CENTER_X
    py = y1 + t * dy - CENTER_Y
    return math.sqrt(px * px + py * py)


def getCollide(x, y, _ROI=(0, 0, 0, 0)):
    return 1 if (_ROI[0] <= x <= _ROI[2] and _ROI[1] <= y <= _ROI[3]) else 0

# ------------------------------------------------------------------ логика ---

def draw_debug(img, o_blob, y_blob, b_blob, line_point, line_segments, y_pt, b_pt):
    img.draw_circle(INNER_CX, INNER_CY, INNER_R, color=_COL_RING, thickness=1)
    img.draw_circle(OUTER_CX, OUTER_CY, OUTER_R, color=_COL_RING, thickness=1)
    img.draw_rectangle(LINE_ROI, color=_COL_LINE_ROI, thickness=1)
    img.draw_circle(EXCLUDE_CX, EXCLUDE_CY, EXCLUDE_R, color=_COL_EXCLUDE, thickness=1)

    if COLLIDE_ROI[2] > COLLIDE_ROI[0] and COLLIDE_ROI[3] > COLLIDE_ROI[1]:
        img.draw_rectangle(
            (COLLIDE_ROI[0], COLLIDE_ROI[1],
             COLLIDE_ROI[2] - COLLIDE_ROI[0], COLLIDE_ROI[3] - COLLIDE_ROI[1]),
            color=_COL_COLLIDE, thickness=1,
        )

    for l in line_segments:
        img.draw_line(l.line(), color=_COL_SEG, thickness=2)

    if o_blob is not None:
        img.draw_cross(o_blob.cx(), o_blob.cy(), color=_COL_O, size=4)
        img.draw_rectangle(o_blob.rect(), color=_COL_O, thickness=1)
    if y_blob is not None:
        img.draw_rectangle(y_blob.rect(), color=_COL_Y, thickness=1)
    if b_blob is not None:
        img.draw_rectangle(b_blob.rect(), color=_COL_B, thickness=1)
    if line_point is not None:
        img.draw_cross(line_point[0], line_point[1], color=_COL_W, size=4)

    if SHOW_OLD_CENTERS:
        if y_blob is not None:
            img.draw_cross(y_blob.cx(), y_blob.cy(), color=_COL_OLD, size=4)
            img.draw_line(INNER_CX, INNER_CY, y_blob.cx(), y_blob.cy(), color=_COL_OLD, thickness=1)
        if b_blob is not None:
            img.draw_cross(b_blob.cx(), b_blob.cy(), color=_COL_OLD, size=4)
            img.draw_line(INNER_CX, INNER_CY, b_blob.cx(), b_blob.cy(), color=_COL_OLD, thickness=1)
        if y_blob is not None and b_blob is not None:
            img.draw_line(y_blob.cx(), y_blob.cy(), b_blob.cx(), b_blob.cy(), color=_COL_OLD, thickness=1)

    if y_pt is not None:
        img.draw_cross(int(y_pt[0]), int(y_pt[1]), color=_COL_Y, size=4)
        img.draw_line(INNER_CX, INNER_CY, int(y_pt[0]), int(y_pt[1]), color=_COL_LINK, thickness=1)
    if b_pt is not None:
        img.draw_cross(int(b_pt[0]), int(b_pt[1]), color=_COL_B, size=4)
        img.draw_line(INNER_CX, INNER_CY, int(b_pt[0]), int(b_pt[1]), color=_COL_LINK, thickness=1)
    if y_pt is not None and b_pt is not None:
        img.draw_line(int(y_pt[0]), int(y_pt[1]), int(b_pt[0]), int(b_pt[1]), color=_COL_LINK, thickness=1)


def find_all_lines(img):
    segments = img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    )

    counter = 0
    sum_x = sum_y = 0
    sum_r = 0.0
    valid_segments = []
    for l in segments:
        if l.length() < LINE_MIN_LENGTH:
            continue
        x1, y1, x2, y2 = l.line()
        mx = (x1 + x2) // 2
        my = (y1 + y2) // 2
        if segment_invalid(x1, y1, x2, y2, mx, my):
            continue
        sum_x += mx
        sum_y += my
        sum_r += segmentRadius(x1, y1, x2, y2)
        counter += 1
        valid_segments.append(l)

    if not counter:
        return None, valid_segments, None
    return (sum_x // counter, sum_y // counter), valid_segments, sum_r / counter


def calcDirect(A, B, C):
    ax, ay = A
    diff = math.atan2(C[1] - ay, C[0] - ax) - math.atan2(B[1] - ay, B[0] - ax)
    if diff > _PI:
        diff -= _TWO_PI
    elif diff < _PI_NEG:
        diff += _TWO_PI
    return diff


def perpAngles(A, B, C):
    dx = C[0] - B[0]
    dy = C[1] - B[1]
    bax = A[0] - B[0]
    bay = A[1] - B[1]
    dd = dx * dx + dy * dy
    if dd == 0:
        return None
    t = (bax * dx + bay * dy) / dd
    o = (B[0] + t * dx, B[1] + t * dy)
    cross = bax * dy - bay * dx
    dot = bax * dx + bay * dy
    abo = math.atan2(abs(cross), abs(dot)) * RAD_TO_DEG
    ao = abs(cross) / math.sqrt(dd)
    return abo, 90.0 - abo, o, ao


def edgePoint(blob, target):
    cx = blob.cx()
    cy = blob.cy()
    ux = target[0] - cx
    uy = target[1] - cy
    ln = math.sqrt(ux * ux + uy * uy)
    if ln == 0:
        return (cx, cy)
    ux /= ln
    uy /= ln
    bx, by, bw, bh = blob.rect()
    s = 1e9
    if ux > 0:
        s = min(s, (bx + bw - cx) / ux)
    elif ux < 0:
        s = min(s, (bx - cx) / ux)
    if uy > 0:
        s = min(s, (by + bh - cy) / uy)
    elif uy < 0:
        s = min(s, (by - cy) / uy)
    return (cx + s * ux, cy + s * uy)


def lineX(A, B, C):
    dx = C[0] - B[0]
    dy = C[1] - B[1]
    dd = dx * dx + dy * dy
    if dd == 0:
        return None
    t = ((A[0] - B[0]) * dx + (A[1] - B[1]) * dy) / dd
    x = X_RANGE * (1 - 2 * t)
    return max(-X_RANGE, min(X_RANGE, x))


def lineY(ao, ang):
    ao = min(ao, Y_RANGE)
    return ao if ang >= 0 else -ao


def linkPoints(b_blob, y_blob):
    bp = (b_blob.cx(), b_blob.cy()) if b_blob is not None else None
    yp = (y_blob.cx(), y_blob.cy()) if y_blob is not None else None
    if USE_BLOB_EDGE and bp is not None and yp is not None:
        return edgePoint(b_blob, yp), edgePoint(y_blob, bp)
    return bp, yp


def analyze(A, B, C):
    ang = calcDirect(A, B, C)
    pb = perpAngles(A, B, C)
    pc = perpAngles(A, C, B)
    xv = lineX(A, B, C)
    if pb is None or pc is None or xv is None:
        return None
    return (ang * RAD_TO_DEG, pb[0], pb[1], pc[0], pc[1], pb[2], pb[3], xv, lineY(pb[3], ang))


def fmtAnalysis(r):
    return (f"by={r[0]:.1f} ABO={r[1]:.1f} OAB={r[2]:.1f} ACO={r[3]:.1f} CAO={r[4]:.1f} "
            f"O=({r[5][0]:.1f},{r[5][1]:.1f}) AO={min(r[6], Y_RANGE):.1f} X={r[7]:.1f} Y={r[8]:.1f}")


def fmtVec(v):
    return f"({v[0]:.1f},{v[1]:.1f})"

# --------------------------------------------------------------------- main --

def main():
    global ballVec, yellowVec, blueVec, lineVec, ball_detect

    clock = time.clock()
    last_print = time.ticks_ms()
    prof = [0, 0, 0, 0, 0, 0]
    prof_n = prof_blobs = 0
    line_point, line_segments, line_radius, line_wait = None, [], None, 0

    while True:
        clock.tick()
        t0 = time.ticks_us()

        try:
            img = sensor.snapshot()
            img.to_rgb565()
        except Exception as e:
            print("Snapshot error:", e)
            __init__()
            continue
        t1 = time.ticks_us()

        if line_wait <= 0:
            line_point, line_segments, line_radius = find_all_lines(img)
            line_wait = LINE_EVERY
        line_wait -= 1
        seen_l = line_point is not None
        t2 = time.ticks_us()

        blobs = img.find_blobs(
            THRESHOLDS,
            pixels_threshold=1,
            area_threshold=1,
            x_stride=2,
            y_stride=2,
        )
        t3 = time.ticks_us()

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
                max_o = area; best_o_blob = b   # <-- было "ba" — NameError
            if (code & 4) and area > max_b:
                max_b = area; best_b_blob = b

        seen_o = best_o_blob is not None
        seen_y = best_y_blob is not None
        seen_b = best_b_blob is not None

        if seen_o:
            cx, cy = best_o_blob.cx(), best_o_blob.cy()
            ball_detect = getCollide(cx, cy, _ROI=COLLIDE_ROI)
            ballVec = vecFromBlobPoint(cx, cy)

        blue_pt, yellow_pt = linkPoints(best_b_blob, best_y_blob)

        if seen_y:
            cx, cy = yellow_pt
            yellowVec = vecFromBlobPoint(cx, cy)

        if seen_b:
            cx, cy = blue_pt
            blueVec = vecFromBlobPoint(cx, cy)

        if seen_l:
            lx, ly = line_point
            lineVec = toVector(lx, ly, line_radius, flip=False)
        else:
            lineVec = (0.0, 0.0)

        t4 = time.ticks_us()

        if SHOW_DEBUG:
            draw_debug(img, best_o_blob, best_y_blob, best_b_blob, line_point, line_segments,
                       yellow_pt, blue_pt)
        t5 = time.ticks_us()

        # ---- сборка 18-байтного пакета ----------------------------------
        flags = ((FLAG_BALL if seen_o else 0) |
                 (FLAG_YELLOW if seen_y else 0) |
                 (FLAG_BLUE if seen_b else 0) |
                 (FLAG_LINE if seen_l else 0) |
                 (FLAG_BALL_DETECT if ball_detect else 0))

        struct.pack_into(
            '<8hB', packet, 0,
            _i16(ballVec[0]), _i16(ballVec[1]),
            _i16(yellowVec[0]), _i16(yellowVec[1]),
            _i16(blueVec[0]), _i16(blueVec[1]),
            _i16(lineVec[0]), _i16(lineVec[1]),
            flags,                                  # <-- 9-й аргумент (был пропущен)
        )
        packet[_PACKET_LEN - 1] = sum(packet[:_PACKET_LEN - 1]) & 0xFF

        if ball_detect:
            trig.high()
        else:
            trig.low()

        try:
            bus.send(packet, timeout=1000)
        except OSError as err:
            print(err)

        t6 = time.ticks_us()

        if PROFILE:
            prof[0] += time.ticks_diff(t1, t0)
            prof[1] += time.ticks_diff(t2, t1)
            prof[2] += time.ticks_diff(t3, t2)
            prof[3] += time.ticks_diff(t4, t3)
            prof[4] += time.ticks_diff(t5, t4)
            prof[5] += time.ticks_diff(t6, t5)
            prof_n += 1
            prof_blobs += len(blobs)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) < PRINT_INTERVAL_MS:
            continue
        last_print = now

        if PROFILE and prof_n:
            d = 1000.0 * prof_n
            print(f"ms/кадр: snap={prof[0] / d:.1f} lines={prof[1] / d:.1f} blobs={prof[2] / d:.1f} "
                  f"pick={prof[3] / d:.1f} draw={prof[4] / d:.1f} i2c={prof[5] / d:.1f} "
                  f"| всего={sum(prof) / d:.1f} | блобов/кадр={prof_blobs / prof_n:.0f}")
            prof = [0, 0, 0, 0, 0, 0]
            prof_n = prof_blobs = 0

        seen = (("B" if seen_o else "-") + ("Y" if seen_y else "-") +
                ("b" if seen_b else "-") + ("L" if seen_l else "-"))
        vecs = (f"ball={fmtVec(ballVec)} y={fmtVec(yellowVec)} b={fmtVec(blueVec)} "
                f"line={fmtVec(lineVec)} seen={seen} bd={ball_detect} fps={clock.fps():.1f}")

        if seen_y and seen_b:
            inner = (INNER_CX, INNER_CY)
            edge = analyze(inner, blue_pt, yellow_pt)
            ctr  = analyze(inner, (best_b_blob.cx(), best_b_blob.cy()),
                                  (best_y_blob.cx(), best_y_blob.cy()))
            if edge is not None and ctr is not None:
                print(f"{vecs} | edge {fmtAnalysis(edge)} | ctr {fmtAnalysis(ctr)}")
                continue
        print(f"{vecs} | нет синего/жёлтого")


if __name__ == "__main__":
    main()
