import pyb
import struct
import sensor
import image
import time
import math

# ═══════════════════════════════════════════════════════════════════════════════
#  mirror_coordPlate_v1.py
#  МЕТОД: таблица goalCoords + realDistance — 100% как в коде анализа.
#
#  Отличие от анализа только в:
#    - камера OpenMV H7 R2 (не QVGA, а HVGA + windowing 190×190)
#    - детекция через кольцо (blob_in_ring) а не mask_circle
#    - связь I2C вместо UART
#    - три цвета (yellow/orange/blue) вместо двух
#
#  Таблицу goalCoords нужно заполнить под ваше зеркало.
#  Используйте скрипт generate_goalCoords.py (аналитика)
#  или измерьте вручную (см. README).
# ═══════════════════════════════════════════════════════════════════════════════
# ── Инициализация камеры ──────────────────────────────────────────────────────
def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HVGA)
    sensor.set_windowing(168, 80, 190, 180)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=3000)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.skip_frames(time=2000)

init_sensor()

# ── Геометрия кадра ──────────────────────────────────────────────────────────
CENTER_X = 93
CENTER_Y = 92
INNER_R  = 30
OUTER_R  = 100
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R
BLOB_ROI = (CENTER_X - OUTER_R, CENTER_Y - OUTER_R, OUTER_R * 2, OUTER_R * 2)
RAD_TO_DEG = 57.2957795

# ── Параметры эквиугольного зеркала ──────────────────────────────────────────
MIRROR_ALPHA     = 4.0    # угловое увеличение α (из Zerkala v2.3)
MIRROR_THETA_MAX = 26.0   # максимальный угол поля зеркала (°)
MIRROR_HEIGHT    = 30.0   # высота зеркала над плоскостью поля (см) — ИЗМЕРЬТЕ!
#
# MIRROR_HEIGHT — самый важный параметр для точности.
# Это расстояние от центра зеркала до поверхности, по которой ездит робот.
# Измерьте рулеткой и запишите в сантиметрах.

# ── Цветовые пороги LAB ───────────────────────────────────────────────────────
THRESHOLDS = [
    (0, 100, -30, -3, 19, 39),   # Yellow  bit0
    (0, 100, 41, 86, -29, 77),   # Orange  bit1
    (0, 100, 4, 65, -106, -56),   # Blue    bit2
]
_COL_O = (255, 165,   0)
_COL_Y = (255, 255,   0)
_COL_B = (  0, 128, 255)
_COL_W = (255, 255, 255)
_COL_R = (255,   0,   0)

# ── I2C ───────────
# ═══════════════════════════════════════════════════════════════════════════════
#  ТАБЛИЦА КАЛИБРОВКИ: пиксельный радиус → реальное расстояние (см)
#
#  Формат: [[реальное_см, пиксели], [реальное_см, пиксели], ...]
#  Строки должны идти по возрастанию пикселей (второй столбец).
#
#  КАК ЗАПОЛНИТЬ:
#    Способ 1 (ручной): поставьте маркер на расстояние X см от точки
#    прямо под зеркалом. Запустите скрипт в режиме отладки —
#    в консоли увидите raw_r=NN. Запишите пару [X, NN].
#    Повторите для 8-15 расстояний.
#
#    Способ 2 (аналитический): запустите generate_goalCoords.py
#    на ПК — он напечатает готовую таблицу под ваши параметры зеркала.
#
#  ЗАГЛУШКА ниже — для запуска без калибровки, расстояние будет неверным!
#  Замените на реальные данные перед использованием.
# ═══════════════════════════════════════════════════════════════════════════════
goalCoords = [
    [ 0,   0],
    [10,   8],
    [20,  16],
    [30,  23],
    [40,  30],
    [50,  37],
    [60,  43],
    [70,  49],
    [80,  54],
    [90,  59],
    [100, 64],
    [120, 72],
    [140, 79],
    [160, 85],   # край зеркала (OUTER_R)
]
# ─────────────────────────────────────────────────────────────────────────────
# ВАЖНО: последняя строка должна соответствовать OUTER_R.
# При расстоянии > последнего значения realDistance экстраполирует линейно —
# точность снижается. Добавляйте точки в таблицу как можно дальше.
# ═══════════════════════════════════════════════════════════════════════════════


def realDistance(x):
    """
    Кусочно-линейная интерполяция: пиксельный радиус → реальное расстояние (см).
    Полная копия функции из кода анализа.
    x — пиксельный радиус от центра кадра (всегда >= 0).
    """
    xAbs = abs(x)
    for i in range(len(goalCoords) - 1):
        if goalCoords[i][1] <= xAbs < goalCoords[i + 1][1]:
            xOne = goalCoords[i][1]
            xTwo = goalCoords[i + 1][1]
            yOne = goalCoords[i][0]
            yTwo = goalCoords[i + 1][0]
            return ((xAbs - xOne) * (yTwo - yOne)) / (xTwo - xOne) + yOne
    # экстраполяция за последней точкой
    xOne = goalCoords[-2][1]
    xTwo = goalCoords[-1][1]
    yOne = goalCoords[-2][0]
    yTwo = goalCoords[-1][0]
    if xTwo == xOne:
        return yTwo
    return ((xAbs - xOne) * (yTwo - yOne)) / (xTwo - xOne) + yOne


# ── I2C ───────────────────────────────────────────────────────────────────────
def make_i2c():
    b = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
    b.deinit()
    return pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)

bus    = make_i2c()
packet = bytearray(8)

def send_packet(ao, ay, ab, flags):
    struct.pack_into('<hhhB', packet, 0, ao, ay, ab, flags)
    csum = packet[0]^packet[1]^packet[2]^packet[3]^packet[4]^packet[5]^packet[6]
    packet[7] = csum
    try:
        bus.send(packet, timeout=1)
    except OSError:
        pass


# ── Вспомогательные ───────────────────────────────────────────────────────────
def blob_in_ring(cx, cy):
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    return INNER_R2 < dx*dx + dy*dy < OUTER_R2


def pixel_to_angle(cx, cy):
    """Азимут 0–359° CCW. Через зеркало азимут не искажается."""
    a = math.atan2(CENTER_Y - cy, cx - CENTER_X) * RAD_TO_DEG
    return int(a + 360.0) % 360


def pixel_to_dist(cx, cy):
    """
    Реальное расстояние через таблицу goalCoords.
    Пиксельный радиус → realDistance → сантиметры.
    Логика 100% как в коде анализа.
    """
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    r_px = math.sqrt(dx*dx + dy*dy)
    return realDistance(r_px)


# ── Состояние ─────────────────────────────────────────────────────────────────
angle_o = angle_y = angle_b = 0
dist_o  = dist_y  = dist_b  = 0
seen_o  = seen_y  = seen_b  = False

# Координаты на поле (как x,y в коде анализа)
# Вычисляются только для синего маркера (как в анализе).
# Для жёлтого — аналогично, раскомментируйте.
field_x = field_y = 0.0

clock = time.clock()
print("v1 (goalCoords) | ROI %s" % str(BLOB_ROI))

# ── Основной цикл ─────────────────────────────────────────────────────────────
while True:
    clock.tick()

    try:
        img = sensor.snapshot()
    except Exception as e:
        print("Snapshot error:", e)
        init_sensor(); bus = make_i2c(); continue

    blobs = img.find_blobs(
        THRESHOLDS, roi=BLOB_ROI,
        pixels_threshold=5, area_threshold=5,
        merge=True, margin=5, x_stride=2, y_stride=2,
    )

    best_o = best_y = best_b = None
    max_o  = max_y  = max_b  = 0

    for b in blobs:
        cx, cy = b.cx(), b.cy()
        if not blob_in_ring(cx, cy): continue
        code = b.code(); area = b.area()
        if (code & 1) and area > max_y: max_y = area; best_y = b
        if (code & 2) and area > max_o: max_o = area; best_o = b
        if (code & 4) and area > max_b: max_b = area; best_b = b

    seen_o = best_o is not None
    seen_y = best_y is not None
    seen_b = best_b is not None

    cx_o = best_o.cx() if seen_o else CENTER_X
    cy_o = best_o.cy() if seen_o else CENTER_Y
    cx_y = best_y.cx() if seen_y else CENTER_X
    cy_y = best_y.cy() if seen_y else CENTER_Y
    cx_b = best_b.cx() if seen_b else CENTER_X
    cy_b = best_b.cy() if seen_b else CENTER_Y

    # Азимуты
    if seen_o:
        angle_o = pixel_to_angle(cx_o, cy_o)
        img.draw_cross(cx_o, cy_o, color=_COL_O, size=8)
    if seen_y:
        angle_y = pixel_to_angle(cx_y, cy_y)
        dist_y  = pixel_to_dist(cx_y, cy_y)
        img.draw_cross(cx_y, cy_y, color=_COL_Y, size=8)
    if seen_b:
        angle_b = pixel_to_angle(cx_b, cy_b)
        dist_b  = pixel_to_dist(cx_b, cy_b)
        img.draw_cross(cx_b, cy_b, color=_COL_B, size=8)

    # Координаты на поле — 100% как в анализе:
    #   x = dist * sin(angle)
    #   y = BASELINE - dist * cos(angle)
    # BASELINE — расстояние от точки прямо под зеркалом до базовой линии поля.
    # В анализе было 85 (см). Измерьте своё.
    BASELINE = 85.0   # см, замените под своё поле
    if seen_b:
        field_x = dist_b * math.sin(angle_b / RAD_TO_DEG)
        field_y = BASELINE - dist_b * math.cos(angle_b / RAD_TO_DEG)

    # Отладочные круги маски
    img.draw_circle(CENTER_X, CENTER_Y, OUTER_R, color=_COL_W, thickness=1)
    img.draw_circle(CENTER_X, CENTER_Y, INNER_R, color=_COL_R, thickness=1)

    # I2C
    flags = int(seen_o) | (int(seen_y) << 1) | (int(seen_b) << 2)
    send_packet(angle_o, angle_y, angle_b, flags)

    # Отладка
    print("O:%3d%s Y:%3d d=%.1f B:%3d d=%.1f pos=(%.1f,%.1f) FPS:%d" % (
        angle_o, "*" if seen_o else ".",
        angle_y, dist_y,
        angle_b, dist_b,
        field_x, field_y,
        clock.fps()
    ))
