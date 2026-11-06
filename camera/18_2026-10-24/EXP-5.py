import gc
import sys
import pyb
import struct
import sensor
import time
import math
from micropython import const
trig = pyb.Pin('P6', pyb.Pin.OUT_PP)
ORANGE_THRESHOLD = (77, 100, 2, 127, 6, 127)
YELLOW_THRESHOLD = (0, 100, -128, 127, 42, 127)
BLUE_THRESHOLD = (0, 60, -128, 33, -128, -20)
BALL_THRESHOLDS = [ORANGE_THRESHOLD]
GATE_THRESHOLDS = [YELLOW_THRESHOLD, BLUE_THRESHOLD]
_CODE_BALL = const(1)
_CODE_YELLOW = const(1)
_CODE_BLUE = const(2)
BALL_PIXELS_MIN = const(1)
GATE_PIXELS_MIN = const(15)
BALL_IN_YELLOW_MARGIN = const(2)   # px вокруг жёлтого блоба, где оранжевый блоб — не мяч
SWAP_GATES = False
CENTER_X = const(100)
CENTER_Y = const(95)
OUTER_R = const(98)
INNER_CX = const(98)
INNER_CY = const(97)
INNER_R = const(32)
INNER_R2 = const(INNER_R * INNER_R)
OUTER_R2 = const(OUTER_R * OUTER_R)
BLOB_ROI = (CENTER_X - OUTER_R, CENTER_Y - OUTER_R, 2 * OUTER_R, 2 * OUTER_R)
RAD_TO_DEG = 57.2957795
_PI = math.pi
_TWO_PI = 2.0 * math.pi
EXCLUDE_R = const(33)
EXCLUDE_R2 = const(EXCLUDE_R * EXCLUDE_R)
LINE_ROI_R = const(38)
LINE_ROI = (INNER_CX - LINE_ROI_R, INNER_CY - LINE_ROI_R, LINE_ROI_R * 2, LINE_ROI_R * 2)
LINE_MERGE_DISTANCE = const(10)
LINE_MAX_THETA_DIFF = const(35)
LINE_MIN_LENGTH = const(15)
LINE_MAX_INSIDE = const(1)
LINE_EVERY = const(2)
COLLIDE_ROI = None
_PACKET_FMT = '<10h'
_POS_NONE = const(-32768)
packet = bytearray(struct.calcsize(_PACKET_FMT))
I2C_RESET_MS = const(500)
_ORANGE = (255, 165, 0)
_YELLOW = (255, 255, 0)
_BLUE = (0, 128, 255)
_WHITE = (255, 255, 255)
_GREEN = (0, 255, 0)
_MAGENTA = (255, 0, 255)
_RED = (255, 0, 0)
_CYAN = (0, 255, 255)
_TURQUOISE = (64, 224, 208)
SHOW_DEBUG = True
DEBUG_ONLY_WITH_USB = True
WATCHDOG_MS = const(2500)
PRINT_INTERVAL_MS = const(500)
PROFILE = False
PRINT_ANALYSIS = False
CALIBRATE = False
X_RANGE = const(110)
Y_RANGE = const(80)
MIRROR_CAL = (
    (49.9, 29.5),
    (70.4, 69.5),
    (78.5, 109.5),
    (82.6, 149.5),
    (85.7, 189.5),
)
FIELD_CX = 93.2
FIELD_CY = 94.4
def _merge_cal(pairs):
    groups = {}
    for r, d in pairs:
        groups.setdefault(round(r, 1), []).append(d)
    return tuple((r, sum(ds) / len(ds)) for r, ds in sorted(groups.items()))
MIRROR_CAL = _merge_cal(MIRROR_CAL)
SPAN_TOL = 0
USE_BLOB_EDGE = True
SHOW_OLD_CENTERS = False
wdt = None
def feed():
    if wdt is not None:
        wdt.feed()
def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.YUV422)
    sensor.set_framesize(sensor.HVGA)
    sensor.set_windowing(158, 80, 200, 200)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True) # False, exposure_us=10000
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    feed()
    sensor.skip_frames(time=2000)
def init_i2c():
    i2c = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
    i2c.deinit()
    return pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
def reset_i2c(old):
    try:
        old.deinit()
    except Exception:
        pass
    return init_i2c()
init_sensor()
bus = init_i2c()
usb = pyb.USB_VCP()
if WATCHDOG_MS and not usb.isconnected():
    from machine import WDT
    wdt = WDT(timeout=WATCHDOG_MS)
def in_circle(x, y, cx, cy, r2):
    dx = x - cx
    dy = y - cy
    return dx * dx + dy * dy <= r2
def in_ring(x, y):
    return (not in_circle(x, y, INNER_CX, INNER_CY, INNER_R2)
            and in_circle(x, y, CENTER_X, CENTER_Y, OUTER_R2))
def bearing(dx, dy):
    a = (int(math.atan2(dx, dy) * RAD_TO_DEG) - 180) % 360
    if a > 180:
        a -= 360
    return a
def polar(x, y):
    # Угол и расстояние — от центра зеркала (FIELD_CX/FIELD_CY, замер 19.10), а не от CENTER:
    # от CENTER угол мяча уходил на 5-10° по-разному для разных направлений и расстояний
    dx = x - FIELD_CX
    dy = y - FIELD_CY
    return bearing(dx, dy), max(math.sqrt(dx * dx + dy * dy) - INNER_R, 0)
def radius(p):
    dx = p[0] - FIELD_CX
    dy = p[1] - FIELD_CY
    return math.sqrt(dx * dx + dy * dy)
def ground_dist(r):
    cal = MIRROR_CAL
    i = 1
    while i < len(cal) - 1 and cal[i][0] < r:
        i += 1
    r0, d0 = cal[i - 1]
    r1, d1 = cal[i]
    return max(d0 + (d1 - d0) * (r - r0) / (r1 - r0), 0.0)
def ground(p):
    dx = p[0] - FIELD_CX
    dy = p[1] - FIELD_CY
    r = math.sqrt(dx * dx + dy * dy)
    if r == 0:
        return (0.0, 0.0)
    k = ground_dist(r) / r
    return (dx * k, dy * k)
def ball_captured(x, y):
    if COLLIDE_ROI is None:
        return 0
    rx, ry, rw, rh = COLLIDE_ROI
    return 1 if (rx <= x <= rx + rw and ry <= y <= ry + rh) else 0
def segment_invalid(x1, y1, x2, y2, mx, my):
    inside = (in_circle(x1, y1, INNER_CX, INNER_CY, EXCLUDE_R2)
              + in_circle(x2, y2, INNER_CX, INNER_CY, EXCLUDE_R2)
              + in_circle(mx, my, INNER_CX, INNER_CY, EXCLUDE_R2))
    return inside > LINE_MAX_INSIDE
def yellow_zones(gates):
    # рамки жёлтых блобов с запасом: оранжевое внутри них — край жёлтых ворот, а не мяч
    m = BALL_IN_YELLOW_MARGIN
    zones = []
    for b in gates:
        if b.code() == _CODE_YELLOW:
            x, y, w, h = b.rect()
            zones.append((x - m, y - m, x + w + m, y + h + m))
    return zones
def in_zones(x, y, zones):
    for x0, y0, x1, y1 in zones:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False
def biggest_in_ring(blobs, code, exclude=None):
    best = None
    best_area = 0
    for b in blobs:
        if b.code() != code:
            continue
        if exclude and in_zones(b.cx(), b.cy(), exclude):
            continue
        area = b.area()
        if area > best_area and in_ring(b.cx(), b.cy()):
            best = b
            best_area = area
    return best
def blob_center(blob):
    return (blob.cx(), blob.cy()) if blob is not None else None
def pack(ball, yellow, blue, out, ball_dist, out_dist, captured, pos_x10, pos_y10, axis):
    gate1, gate2 = (yellow, blue) if SWAP_GATES else (blue, yellow)
    struct.pack_into(_PACKET_FMT, packet, 0, ball, gate2, gate1, out, int(ball_dist), captured,
                     pos_x10, pos_y10, axis, int(out_dist))
def find_all_lines(img):
    sum_x = sum_y = 0
    segments = []
    for l in img.find_line_segments(
        roi=LINE_ROI,
        merge_distance=LINE_MERGE_DISTANCE,
        max_theta_diff=LINE_MAX_THETA_DIFF,
    ):
        if l.length() < LINE_MIN_LENGTH:
            continue
        x1, y1, x2, y2 = l.line()
        mx = (x1 + x2) // 2
        my = (y1 + y2) // 2
        if segment_invalid(x1, y1, x2, y2, mx, my):
            continue
        sum_x += mx
        sum_y += my
        segments.append((x1, y1, x2, y2))
    n = len(segments)
    point = (sum_x // n, sum_y // n) if n else None
    return point, segments
def edge_point(blob, target):
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
def analyze(A, B, C):
    dx = C[0] - B[0]
    dy = C[1] - B[1]
    dd = dx * dx + dy * dy
    if dd == 0:
        return None
    bax = A[0] - B[0]
    bay = A[1] - B[1]
    dot = bax * dx + bay * dy
    cross = abs(bax * dy - bay * dx)
    t = dot / dd
    o = (B[0] + t * dx, B[1] + t * dy)
    ao = cross / math.sqrt(dd)
    abo = math.atan2(cross, abs(dot)) * RAD_TO_DEG
    aco = math.atan2(cross, abs(dd - dot)) * RAD_TO_DEG
    ang = math.atan2(C[1] - A[1], C[0] - A[0]) - math.atan2(B[1] - A[1], B[0] - A[0])
    if ang > _PI:
        ang -= _TWO_PI
    elif ang < -_PI:
        ang += _TWO_PI
    x = max(-X_RANGE, min(X_RANGE, X_RANGE * (1 - 2 * t)))
    y = min(ao * (2 * X_RANGE) / math.sqrt(dd), Y_RANGE)
    if ang < 0:
        y = -y
    return (ang * RAD_TO_DEG, abo, 90.0 - abo, aco, 90.0 - aco, o, ao, x, y)
def fmtAnalysis(r):
    return (f"by={r[0]:.1f} ABO={r[1]:.1f} OAB={r[2]:.1f} ACO={r[3]:.1f} CAO={r[4]:.1f} "
            f"O=({r[5][0]:.1f},{r[5][1]:.1f}) AO={min(r[6], Y_RANGE):.1f} X={r[7]:.1f} Y={r[8]:.1f}")
def fmt_cal(name, p):
    if not p:
        return f"{name} нет"
    r = radius(p)
    return f"{name} r={r:.1f} px ~{ground_dist(r):.0f} см"
def fmt_seen(name, value, seen):
    return f"{name} {value}°" if seen else f"{name} ({value}°)"
def _mark(img, p, cross_color, line_color):
    x = int(p[0])
    y = int(p[1])
    img.draw_cross(x, y, color=cross_color, size=4)
    img.draw_line(INNER_CX, INNER_CY, x, y, color=line_color, thickness=1)
def draw_debug(img, o_blob, y_blob, b_blob, line_point, line_segments,
               y_pt, b_pt, y_ctr, b_ctr):
    img.draw_circle(INNER_CX, INNER_CY, INNER_R, color=_GREEN, thickness=1)
    img.draw_circle(CENTER_X, CENTER_Y, OUTER_R, color=_GREEN, thickness=1)
    img.draw_rectangle(LINE_ROI, color=_MAGENTA, thickness=1)
    img.draw_circle(INNER_CX, INNER_CY, EXCLUDE_R, color=_RED, thickness=1)
    if COLLIDE_ROI is not None:
        img.draw_rectangle(COLLIDE_ROI, color=_CYAN, thickness=1)
    for seg in line_segments:
        img.draw_line(seg, color=_GREEN, thickness=2)
    if line_point is not None:
        img.draw_cross(line_point[0], line_point[1], color=_WHITE, size=4)
    if o_blob is not None:
        img.draw_cross(o_blob.cx(), o_blob.cy(), color=_ORANGE, size=4)
        img.draw_rectangle(o_blob.rect(), color=_ORANGE, thickness=1)
    if y_blob is not None:
        img.draw_rectangle(y_blob.rect(), color=_YELLOW, thickness=1)
    if b_blob is not None:
        img.draw_rectangle(b_blob.rect(), color=_BLUE, thickness=1)
    if SHOW_OLD_CENTERS:
        if y_ctr:
            _mark(img, y_ctr, _TURQUOISE, _TURQUOISE)
        if b_ctr:
            _mark(img, b_ctr, _TURQUOISE, _TURQUOISE)
        if y_ctr and b_ctr:
            img.draw_line(y_ctr[0], y_ctr[1], b_ctr[0], b_ctr[1], color=_TURQUOISE, thickness=1)
    if y_pt:
        _mark(img, y_pt, _YELLOW, _WHITE)
    if b_pt:
        _mark(img, b_pt, _BLUE, _WHITE)
    if y_pt and b_pt:
        img.draw_line(int(y_pt[0]), int(y_pt[1]), int(b_pt[0]), int(b_pt[1]),
                      color=_WHITE, thickness=1)
def main():
    global bus
    clock = time.clock()
    last_print = time.ticks_ms()
    prof = [0, 0, 0, 0, 0, 0]
    prof_n = prof_blobs = 0
    i2c_errors = 0
    i2c_resets = 0
    last_sent = time.ticks_ms()
    line_point, line_segments, line_wait = None, [], 0
    orangeDirect = yellowDirect = blueDirect = 0
    orangeDist = 0
    ball_detect = 0
    console = usb.isconnected() or not DEBUG_ONLY_WITH_USB
    debug = SHOW_DEBUG and console
    print("EXP-5: ворота 1 (цель атаки) —", "жёлтые" if SWAP_GATES else "синие")
    while True:
        feed()
        clock.tick()
        t0 = time.ticks_us()
        try:
            img = sensor.snapshot()
            img.to_rgb565()
        except Exception as e:
            print("Snapshot error:", e)
            init_sensor()
            continue
        t1 = time.ticks_us()
        if line_wait <= 0:
            line_point, line_segments = find_all_lines(img)
            line_wait = LINE_EVERY
        line_wait -= 1
        t2 = time.ticks_us()
        balls = img.find_blobs(
            BALL_THRESHOLDS, roi=BLOB_ROI,
            pixels_threshold=BALL_PIXELS_MIN, area_threshold=BALL_PIXELS_MIN,
            x_stride=2, y_stride=2,
        )
        gates = img.find_blobs(
            GATE_THRESHOLDS, roi=BLOB_ROI,
            pixels_threshold=GATE_PIXELS_MIN, area_threshold=GATE_PIXELS_MIN,
            x_stride=2, y_stride=2,
        )
        t3 = time.ticks_us()
        best_o_blob = biggest_in_ring(balls, _CODE_BALL, yellow_zones(gates))
        best_y_blob = biggest_in_ring(gates, _CODE_YELLOW)
        best_b_blob = biggest_in_ring(gates, _CODE_BLUE)
        n_blobs = len(balls) + len(gates)
        balls = gates = None
        if best_o_blob is not None:
            cx, cy = best_o_blob.cx(), best_o_blob.cy()
            ball_detect = ball_captured(cx, cy)
            orangeDirect, orangeDist = polar(cx, cy)
        yellow_ctr = blob_center(best_y_blob)
        blue_ctr = blob_center(best_b_blob)
        if USE_BLOB_EDGE and yellow_ctr and blue_ctr:
            yellow_pt = edge_point(best_y_blob, blue_ctr)
            blue_pt = edge_point(best_b_blob, yellow_ctr)
        else:
            yellow_pt, blue_pt = yellow_ctr, blue_ctr
        if yellow_pt:
            yellowDirect = polar(yellow_pt[0], yellow_pt[1])[0]
        if blue_pt:
            blueDirect = polar(blue_pt[0], blue_pt[1])[0]
        if line_point is not None:
            outDirect = (polar(line_point[0], line_point[1])[0] + 180) % 360
            outDist = polar(line_point[0], line_point[1])[1]
        else:
            outDirect = -1
            outDist = -1
        field = None
        span = 0.0
        if yellow_pt and blue_pt:
            g1, g2 = (yellow_pt, blue_pt) if SWAP_GATES else (blue_pt, yellow_pt)
            b, c = ground(g1), ground(g2)
            span = math.sqrt((b[0] - c[0]) ** 2 + (b[1] - c[1]) ** 2)
            if not SPAN_TOL or abs(span / (2 * X_RANGE) - 1) <= SPAN_TOL:
                field = analyze((0.0, 0.0), b, c)
        if field is not None:
            posX10 = int(field[7] * 10)
            posY10 = int(field[8] * 10)
            fieldAxis = bearing(b[0] - c[0], b[1] - c[1])
        else:
            posX10 = posY10 = _POS_NONE
            fieldAxis = 0
        t4 = time.ticks_us()
        if debug:
            draw_debug(img, best_o_blob, best_y_blob, best_b_blob, line_point, line_segments,
                       yellow_pt, blue_pt, yellow_ctr, blue_ctr)
        t5 = time.ticks_us()
        pack(orangeDirect, yellowDirect, blueDirect, outDirect, orangeDist, outDist, ball_detect,
             posX10, posY10, fieldAxis)
        trig.value(ball_detect)
        try:
            bus.send(packet, timeout=100)
            last_sent = time.ticks_ms()
        except OSError:
            i2c_errors += 1
            if time.ticks_diff(time.ticks_ms(), last_sent) > I2C_RESET_MS:
                bus = reset_i2c(bus)
                last_sent = time.ticks_ms()
                i2c_resets += 1
        t6 = time.ticks_us()
        if PROFILE:
            prof[0] += time.ticks_diff(t1, t0)
            prof[1] += time.ticks_diff(t2, t1)
            prof[2] += time.ticks_diff(t3, t2)
            prof[3] += time.ticks_diff(t4, t3)
            prof[4] += time.ticks_diff(t5, t4)
            prof[5] += time.ticks_diff(t6, t5)
            prof_n += 1
            prof_blobs += n_blobs
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) < PRINT_INTERVAL_MS:
            continue
        last_print = now
        if console:
            line = f"{outDirect}°" if outDirect >= 0 else "нет"
            pos = (f"X {posX10 / 10:.1f} Y {posY10 / 10:.1f} ось {fieldAxis}°"
                   if posX10 != _POS_NONE else "нет")
            print(f"fps {clock.fps():.1f} | {fmt_seen('мяч', orangeDirect, best_o_blob is not None)} "
                  f"d={int(orangeDist)} | {fmt_seen('жёлт', yellowDirect, yellow_pt is not None)} | "
                  f"{fmt_seen('син', blueDirect, blue_pt is not None)} | от линии {line} | "
                  f"захват {ball_detect} | поле {pos} | i2c ошибок {i2c_errors}, "
                  f"перезапусков {i2c_resets}")
            if PROFILE and prof_n:
                d = 1000.0 * prof_n
                print(f"  мс/кадр: snap={prof[0] / d:.1f} lines={prof[1] / d:.1f} "
                      f"blobs={prof[2] / d:.1f} pick={prof[3] / d:.1f} draw={prof[4] / d:.1f} "
                      f"i2c={prof[5] / d:.1f} | всего={sum(prof) / d:.1f} | "
                      f"блобов/кадр={prof_blobs / prof_n:.0f}")
            if PRINT_ANALYSIS and field is not None:
                print("  " + fmtAnalysis(field))
            if CALIBRATE:
                print(f"  калибровка: {fmt_cal('жёлт', yellow_pt)} | {fmt_cal('син', blue_pt)} | "
                      f"между воротами {span:.0f} см (должно быть ~{2 * X_RANGE})")
        i2c_errors = i2c_resets = 0
        prof_n = prof_blobs = 0
        for i in range(6):
            prof[i] = 0
        console = usb.isconnected() or not DEBUG_ONLY_WITH_USB
        debug = SHOW_DEBUG and console
if __name__ == "__main__":
    while True:
        try:
            main()
        except Exception as e:
            sys.print_exception(e)
            gc.collect()
            bus = reset_i2c(bus)
            time.sleep_ms(100)
