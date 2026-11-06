# EXP-5 — камера OpenMV H7 R2: мяч, ворота, линия -> пакет 12 Б по I2C (slave 0x12).
# Отличия от EXP-4:
#   - мяч и ворота ищутся двумя вызовами find_blobs: у ворот свой порог размера;
#   - дубли функций и констант убраны, углы и расстояние считает одна функция polar();
#   - polar() считает углы от CENTER_X/CENTER_Y (в EXP-4 оси центра были перепутаны,
#     фактический центр был (98, 100)) — углы могут сдвинуться на несколько градусов;
#   - SWAP_GATES меняет синие и жёлтые ворота местами в пакете;
#   - в консоль — одна строка состояния; отрисовка и печать только при подключённом USB.
import pyb
import struct
import sensor
import time
import math
from micropython import const

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

# ── Цвета (пороги LAB) ───────────────────────────────────────────────────────
ORANGE_THRESHOLD = (0, 100, 17, 127, 25, 127)
YELLOW_THRESHOLD = (0, 100, -128, 127, 42, 127)
BLUE_THRESHOLD = (0, 100, -128, 127, -128, -16)

# code() блоба = бит индекса порога в списке, переданном в find_blobs
BALL_THRESHOLDS = [ORANGE_THRESHOLD]
GATE_THRESHOLDS = [YELLOW_THRESHOLD, BLUE_THRESHOLD]
_CODE_BALL = const(1)
_CODE_YELLOW = const(1)
_CODE_BLUE = const(2)

# Минимальный размер блоба (пикселей и площади рамки). Мяч издалека — несколько
# пикселей, ворота всегда крупнее, поэтому у ворот шум отсекается сильнее.
BALL_PIXELS_MIN = const(1)
GATE_PIXELS_MIN = const(20)    # подобрать по самым дальним воротам в кадре

# ── Ворота в пакете ──────────────────────────────────────────────────────────
# На RP2350 ворота 1 (gate_dir_1) — цель атаки, ворота 2 (gate_dir_2) — свои.
# False: ворота 1 — синие, ворота 2 — жёлтые (как в EXP-4). True — наоборот.
SWAP_GATES = False

# ── Геометрия кадра ──────────────────────────────────────────────────────────
# Центр робота: от него считаются углы и расстояния; вокруг него же внешний круг
CENTER_X = const(100)
CENTER_Y = const(98)
OUTER_R = const(98)

# Внутренний круг (корпус робота в зеркале) настраивается отдельно от центра
INNER_CX = const(98)
INNER_CY = const(97)
INNER_R = const(32)

INNER_R2 = const(INNER_R * INNER_R)
OUTER_R2 = const(OUTER_R * OUTER_R)

# Квадрат вокруг внешнего круга: за его пределами блобы не нужны
BLOB_ROI = (CENTER_X - OUTER_R, CENTER_Y - OUTER_R, 2 * OUTER_R, 2 * OUTER_R)

RAD_TO_DEG = 57.2957795
_PI = math.pi
_TWO_PI = 2.0 * math.pi

# ── Линия ────────────────────────────────────────────────────────────────────
# Зона исключения сегментов — вокруг центра внутреннего круга, со своим радиусом
EXCLUDE_R = const(33)
EXCLUDE_R2 = const(EXCLUDE_R * EXCLUDE_R)

LINE_ROI_R = const(42)
LINE_ROI = (INNER_CX - LINE_ROI_R, INNER_CY - LINE_ROI_R, LINE_ROI_R * 2, LINE_ROI_R * 2)

LINE_MERGE_DISTANCE = const(10)
LINE_MAX_THETA_DIFF = const(35)
LINE_MIN_LENGTH = const(15)

# Сегмент проверяется по 3 точкам (2 конца + середина).
# Если внутри зоны исключения больше LINE_MAX_INSIDE точек — сегмент некорректный.
LINE_MAX_INSIDE = const(1)

# Линии ищем раз в LINE_EVERY кадров (в остальные кадры берётся предыдущий результат)
LINE_EVERY = const(2)

# ── Захват мяча ──────────────────────────────────────────────────────────────
# Зона захвата (x, y, w, h), как и остальные ROI: центр мяча внутри — ball_detect = 1
# и пин P6 в HIGH. None — зона не задана, ball_detect всегда 0 (как в EXP-4).
COLLIDE_ROI = None

# ── Пакет для RP2350 ─────────────────────────────────────────────────────────
# мяч°, ворота 2°, ворота 1°, от линии° (-1 — линии нет), дистанция до мяча, захват
_PACKET_FMT = '<6h'
packet = bytearray(struct.calcsize(_PACKET_FMT))

# ── Отладка ──────────────────────────────────────────────────────────────────
_ORANGE = (255, 165, 0)
_YELLOW = (255, 255, 0)
_BLUE = (0, 128, 255)
_WHITE = (255, 255, 255)
_GREEN = (0, 255, 0)
_MAGENTA = (255, 0, 255)
_RED = (255, 0, 0)
_CYAN = (0, 255, 255)
_TURQUOISE = (64, 224, 208)   # прежние центры блобов

# Отладочная отрисовка колец/линий/зон в кадре (видна в OpenMV IDE)
SHOW_DEBUG = True

# Рисовать и печатать, только когда камера подключена к компьютеру по USB:
# на поле время кадра не уходит на отладку, даже если SHOW_DEBUG забыли выключить
DEBUG_ONLY_WITH_USB = True

# Как часто печатать строку состояния (мс)
PRINT_INTERVAL_MS = const(500)

# Печатать среднее время этапов кадра (мс), чтобы понять, что тормозит fps
PROFILE = False

# Печатать геометрию ворот: углы ABO/OAB/ACO/CAO, точку O, X и Y (см. analyze)
PRINT_ANALYSIS = False

# Диапазон условной точки X на прямой синий—жёлтый: от -X_RANGE до +X_RANGE
X_RANGE = const(110)

# Y = знак * AO, AO от 0 до Y_RANGE
Y_RANGE = const(80)

# True — синий/жёлтый берём не по центру блоба, а по его краю на прямой между блобами
USE_BLOB_EDGE = True

# Показывать прежние центры блобов и линии по ним (бирюзовым)
SHOW_OLD_CENTERS = False

# System functions


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.YUV422)
    sensor.set_framesize(sensor.HVGA)         # 480×320 — аппаратный буфер
    sensor.set_windowing(158, 80, 200, 200)   # окно 200×200 вокруг зеркала

    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=10000)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.skip_frames(time=2000)


def init_i2c():
    # Создать, сбросить и создать заново (как в примере OpenMV): шина в исходном состоянии
    i2c = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
    i2c.deinit()
    return pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)


init_sensor()
bus = init_i2c()
usb = pyb.USB_VCP()

# Fast functions


def in_circle(x, y, cx, cy, r2):
    dx = x - cx
    dy = y - cy
    return dx * dx + dy * dy <= r2


def in_ring(x, y):
    """Точка между внутренним и внешним кругами — там ищем мяч и ворота."""
    return (not in_circle(x, y, INNER_CX, INNER_CY, INNER_R2)
            and in_circle(x, y, CENTER_X, CENTER_Y, OUTER_R2))


def polar(x, y):
    """
    Угол и расстояние до точки (x, y) от центра робота.
    Угол — целые градусы: 0 — вперёд (вверх по кадру), плюс — против часовой, (-180, 180].
    Расстояние — от края внутреннего круга, не меньше 0.
    """
    dx = x - CENTER_X
    dy = y - CENTER_Y
    a = (int(math.atan2(dx, dy) * RAD_TO_DEG) - 180) % 360   # atan2(dx, dy): 0 — вниз по кадру
    if a > 180:
        a -= 360
    return a, max(math.sqrt(dx * dx + dy * dy) - INNER_R, 0)


def ball_captured(x, y):
    if COLLIDE_ROI is None:
        return 0
    rx, ry, rw, rh = COLLIDE_ROI
    return 1 if (rx <= x <= rx + rw and ry <= y <= ry + rh) else 0


def segment_invalid(x1, y1, x2, y2, mx, my):
    """Больше LINE_MAX_INSIDE из трёх точек сегмента (концы и середина) в зоне исключения."""
    inside = (in_circle(x1, y1, INNER_CX, INNER_CY, EXCLUDE_R2)
              + in_circle(x2, y2, INNER_CX, INNER_CY, EXCLUDE_R2)
              + in_circle(mx, my, INNER_CX, INNER_CY, EXCLUDE_R2))
    return inside > LINE_MAX_INSIDE


def biggest_in_ring(blobs, code):
    """Самый большой (по площади рамки) блоб нужного цвета, чей центр лежит в кольце."""
    best = None
    best_area = 0
    for b in blobs:
        if b.code() != code:
            continue
        area = b.area()
        if area > best_area and in_ring(b.cx(), b.cy()):
            best = b
            best_area = area
    return best


def blob_center(blob):
    return (blob.cx(), blob.cy()) if blob is not None else None


def pack(ball, yellow, blue, out, ball_dist, captured):
    """Заполнить packet: мяч, ворота 2, ворота 1, от линии, дистанция до мяча, захват."""
    gate1, gate2 = (yellow, blue) if SWAP_GATES else (blue, yellow)
    struct.pack_into(_PACKET_FMT, packet, 0, ball, gate2, gate1, out, int(ball_dist), captured)

# Logik functions


def find_all_lines(img):
    """Средняя точка корректных сегментов линии (None — линии нет) и сами сегменты."""
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
        segments.append((x1, y1, x2, y2))   # кортеж: при отрисовке не вызывать l.line() снова

    n = len(segments)
    point = (sum_x // n, sum_y // n) if n else None
    return point, segments


def edge_point(blob, target):
    """
    Точка на границе рамки блоба (rect), где луч от центра блоба к точке target
    выходит наружу. Лежит на прямой центр блоба — target.
    """
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
    """
    Все величины по трём точкам: A — центр кольца, B — синие ворота, C — жёлтые.
    O — основание перпендикуляра из A на прямую B–C.
    Возвращает (угол от A→B до A→C°, ABO, OAB, ACO, CAO, O, AO, X, Y)
    или None, если B и C совпадают. Углы треугольников острые: ABO + OAB = ACO + CAO = 90°.
    X — положение O на отрезке: +X_RANGE в синих, -X_RANGE в жёлтых, 0 — середина
    (за пределами отрезка обрезается). Y — AO (не больше Y_RANGE) со знаком угла.
    """
    dx = C[0] - B[0]
    dy = C[1] - B[1]
    dd = dx * dx + dy * dy
    if dd == 0:
        return None
    bax = A[0] - B[0]
    bay = A[1] - B[1]
    dot = bax * dx + bay * dy             # |BO| * |BC|, со знаком
    cross = abs(bax * dy - bay * dx)      # AO * |BC|
    t = dot / dd                          # 0 — O в синих, 1 — в жёлтых
    o = (B[0] + t * dx, B[1] + t * dy)
    ao = cross / math.sqrt(dd)
    abo = math.atan2(cross, abs(dot)) * RAD_TO_DEG
    aco = math.atan2(cross, abs(dd - dot)) * RAD_TO_DEG     # dd - dot = |CO| * |BC|

    # Знаковый угол от луча A→B до луча A→C, [-π, π]: плюс — против часовой
    ang = math.atan2(C[1] - A[1], C[0] - A[0]) - math.atan2(B[1] - A[1], B[0] - A[0])
    if ang > _PI:
        ang -= _TWO_PI
    elif ang < -_PI:
        ang += _TWO_PI

    x = max(-X_RANGE, min(X_RANGE, X_RANGE * (1 - 2 * t)))
    y = min(ao, Y_RANGE)
    if ang < 0:
        y = -y
    return (ang * RAD_TO_DEG, abo, 90.0 - abo, aco, 90.0 - aco, o, ao, x, y)


def fmtAnalysis(r):
    return (f"by={r[0]:.1f} ABO={r[1]:.1f} OAB={r[2]:.1f} ACO={r[3]:.1f} CAO={r[4]:.1f} "
            f"O=({r[5][0]:.1f},{r[5][1]:.1f}) AO={min(r[6], Y_RANGE):.1f} X={r[7]:.1f} Y={r[8]:.1f}")


def fmt_seen(name, value, seen):
    """Угол для строки состояния; в скобках — последний известный (объект сейчас не виден)."""
    return f"{name} {value}°" if seen else f"{name} ({value}°)"


def _mark(img, p, cross_color, line_color):
    """Крест в точке p и линия к ней от центра внутреннего круга."""
    x = int(p[0])
    y = int(p[1])
    img.draw_cross(x, y, color=cross_color, size=4)
    img.draw_line(INNER_CX, INNER_CY, x, y, color=line_color, thickness=1)


def draw_debug(img, o_blob, y_blob, b_blob, line_point, line_segments,
               y_pt, b_pt, y_ctr, b_ctr):
    """Вся отладочная визуализация рисуется одним проходом в конце кадра."""
    # Кольцо поиска блобов, ROI поиска линий, зона исключения сегментов
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

    # Прежние центры блобов и линии по ним — бирюзовым, по умолчанию скрыты
    if SHOW_OLD_CENTERS:
        if y_ctr:
            _mark(img, y_ctr, _TURQUOISE, _TURQUOISE)
        if b_ctr:
            _mark(img, b_ctr, _TURQUOISE, _TURQUOISE)
        if y_ctr and b_ctr:
            img.draw_line(y_ctr[0], y_ctr[1], b_ctr[0], b_ctr[1], color=_TURQUOISE, thickness=1)

    # Рабочие точки ворот (краевые, если USE_BLOB_EDGE) и линии между ними
    if y_pt:
        _mark(img, y_pt, _YELLOW, _WHITE)
    if b_pt:
        _mark(img, b_pt, _BLUE, _WHITE)
    if y_pt and b_pt:
        img.draw_line(int(y_pt[0]), int(y_pt[1]), int(b_pt[0]), int(b_pt[1]),
                      color=_WHITE, thickness=1)


def main():
    clock = time.clock()
    last_print = time.ticks_ms()
    prof = [0, 0, 0, 0, 0, 0]       # суммы по этапам, мкс (для PROFILE)
    prof_n = prof_blobs = 0
    i2c_errors = 0                  # пакетов, которые RP2350 не забрал, с прошлой печати
    line_point, line_segments, line_wait = None, [], 0

    # Пропавший мяч или ворота не сбрасываются: в пакет уходит последнее значение,
    # и робот едет туда, где видел объект в последний раз
    orangeDirect = yellowDirect = blueDirect = 0
    orangeDist = 0
    ball_detect = 0

    # Подключён ли компьютер — проверяется раз в PRINT_INTERVAL_MS
    console = usb.isconnected() or not DEBUG_ONLY_WITH_USB
    debug = SHOW_DEBUG and console
    print("EXP-5: ворота 1 (цель атаки) —", "жёлтые" if SWAP_GATES else "синие")

    while True:
        clock.tick()
        t0 = time.ticks_us()

        # do img with error protect
        try:
            img = sensor.snapshot()
            img.to_rgb565()
        except Exception as e:
            print("Snapshot error:", e)
            init_sensor()
            continue
        t1 = time.ticks_us()

        # Линии ищем не каждый кадр; в остальные кадры остаётся прошлый результат
        if line_wait <= 0:
            line_point, line_segments = find_all_lines(img)
            line_wait = LINE_EVERY
        line_wait -= 1
        t2 = time.ticks_us()

        # Мяч и ворота — отдельными вызовами: у ворот выше порог размера
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

        best_o_blob = biggest_in_ring(balls, _CODE_BALL)
        best_y_blob = biggest_in_ring(gates, _CODE_YELLOW)
        best_b_blob = biggest_in_ring(gates, _CODE_BLUE)
        n_blobs = len(balls) + len(gates)
        balls = gates = None    # не держать списки блобов в куче до следующего кадра

        # ── Обновление углов ──────────────────────────────────────────────────
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

        # Направление ОТ линии, [0, 360); -1 — линии нет
        if line_point is not None:
            outDirect = (polar(line_point[0], line_point[1])[0] + 180) % 360
        else:
            outDirect = -1
        t4 = time.ticks_us()

        # ── Вся отладочная отрисовка — одним проходом в конце кадра ───────────
        if debug:
            draw_debug(img, best_o_blob, best_y_blob, best_b_blob, line_point, line_segments,
                       yellow_pt, blue_pt, yellow_ctr, blue_ctr)
        t5 = time.ticks_us()

        pack(orangeDirect, yellowDirect, blueDirect, outDirect, orangeDist, ball_detect)
        trig.value(ball_detect)

        try:
            bus.send(packet, timeout=100)
        except OSError:
            i2c_errors += 1     # RP2350 не забрал пакет за 100 мс; число — в строке состояния
        t6 = time.ticks_us()

        if PROFILE:
            prof[0] += time.ticks_diff(t1, t0)     # snapshot + to_rgb565 (+ JPEG для IDE)
            prof[1] += time.ticks_diff(t2, t1)     # find_line_segments
            prof[2] += time.ticks_diff(t3, t2)     # find_blobs: мяч + ворота
            prof[3] += time.ticks_diff(t4, t3)     # выбор блобов + расчёты
            prof[4] += time.ticks_diff(t5, t4)     # отрисовка
            prof[5] += time.ticks_diff(t6, t5)     # pack + trig + I2C
            prof_n += 1
            prof_blobs += n_blobs

        # Вывод не чаще раза в PRINT_INTERVAL_MS, сам цикл не тормозим
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) < PRINT_INTERVAL_MS:
            continue
        last_print = now

        if console:
            line = f"{outDirect}°" if outDirect >= 0 else "нет"
            print(f"fps {clock.fps():.1f} | {fmt_seen('мяч', orangeDirect, best_o_blob is not None)} "
                  f"d={int(orangeDist)} | {fmt_seen('жёлт', yellowDirect, yellow_pt is not None)} | "
                  f"{fmt_seen('син', blueDirect, blue_pt is not None)} | от линии {line} | "
                  f"захват {ball_detect} | i2c ошибок {i2c_errors}")

            if PROFILE and prof_n:
                d = 1000.0 * prof_n
                print(f"  мс/кадр: snap={prof[0] / d:.1f} lines={prof[1] / d:.1f} "
                      f"blobs={prof[2] / d:.1f} pick={prof[3] / d:.1f} draw={prof[4] / d:.1f} "
                      f"i2c={prof[5] / d:.1f} | всего={sum(prof) / d:.1f} | "
                      f"блобов/кадр={prof_blobs / prof_n:.0f}")

            if PRINT_ANALYSIS and yellow_pt and blue_pt:
                r = analyze((INNER_CX, INNER_CY), blue_pt, yellow_pt)
                if r is not None:
                    print("  " + fmtAnalysis(r))

        i2c_errors = 0
        prof_n = prof_blobs = 0
        for i in range(6):
            prof[i] = 0

        console = usb.isconnected() or not DEBUG_ONLY_WITH_USB
        debug = SHOW_DEBUG and console


if __name__ == "__main__":
    main()
