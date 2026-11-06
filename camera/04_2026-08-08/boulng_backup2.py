import pyb
import struct
import sensor
#  import time
import math

#
#  _____constants_____
#

KEGL_THRESHOLD = (0, 100, 37, 127, -128, 127)
THRESHOLDS = [KEGL_THRESHOLD]
_PACKET_LEN = 4  # float32 вместо int16 - для точности угла (доли градуса не теряются)

# Блобы с cy БОЛЬШЕ этого считаются "нижними" (ближе к роботу/приоритетные).
# Если таких нет вообще - используем все блобы подряд, а не сообщаем "не вижу".
# Подберите под своё окно (sensor.set_windowing даёт высоту окна 80px, cy в [0,80)).
Y_LEVEL_THRESHOLD = 16


#
# _____variables_____
#

packet = bytearray(_PACKET_LEN)


def __init__():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HVGA)       # 480×320 — аппаратный буфер
    sensor.set_windowing(0, 110, 480, 80)   # Полное окно, без потери разрешения

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


def main():
    while True:
        try:
            img = sensor.snapshot()
        except Exception as e:
            print("Snapshot error:", e)
            __init__()
            continue

        blobs = img.find_blobs(
                THRESHOLDS,
                pixels_threshold=15,
                area_threshold=15,
                merge=False,
                #margin=5,
                x_stride=1,
                y_stride=1,
            )

        # ── Шаг 1: Фильтруем только те блобы, у которых правильный цвет ────────
        # (b.code/.cx/.cy - без скобок, как было в исходнике: не меняю стиль
        # обращения к blob, чтобы не задеть то, что у вас уже отлажено)
        color_blobs = [b for b in blobs if b.code & 1]

        # ── Шаг 2: Из них приоритет "нижним" (cy > Y_LEVEL_THRESHOLD) блобам -
        # если таких нет вообще, откатываемся на все цветные блобы ("основные"),
        # а не остаёмся без цели.
        low_blobs = [b for b in color_blobs if b.cy > Y_LEVEL_THRESHOLD]
        use_blobs = low_blobs if low_blobs else color_blobs

        v_x = [b.cx for b in use_blobs]
        v_y = [b.cy for b in use_blobs]

        # Инициализируем переменные по умолчанию
        sum_cx = 0
        sum_cy = 0
        count_k = len(use_blobs)

        # ── Шаг 3: Усредняем координаты выбранных блобов ───────────────────────
        if count_k >= 1:
            sum_cx = sum(v_x)
            sum_cy = sum(v_y)

        # ── Обновление углов + визуализация ───────────────────────────────────────
        seen_k = count_k > 0
        angle = -1.0
        if seen_k:
            cx = sum_cx / count_k
            cy = sum_cy / count_k
            val = (cx-240) / 480
            val = max(-1.0, min(1.0, val))
            angle = (math.degrees(math.asin(val)) + 360) % 360
            img.draw_cross((int(cx), int(cy)), color=(255, 0, 0), size=8)

        struct.pack_into(
            '<f', packet, 0,
            float(angle),
        )

        try:
            bus.send(packet, timeout=1)
        except OSError:
            pass

        print(angle)


if __name__ == "__main__":
    main()
