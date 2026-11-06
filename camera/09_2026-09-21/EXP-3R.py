import pyb
import struct
import sensor
import time
import math
import os

try:
    import mjpeg
except ImportError:
    mjpeg = None

trig = pyb.Pin('P6', pyb.Pin.OUT_PP)

# Constants
YELLOW_THRESHOLD = (34, 100, 10, 69, -128, 43)
ORANGE_THRESHOLD = (53, 100, 7, 127, 10, 127)
BLUE_THRESHOLD = (29, 100, -128, 127, -128, -6)
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
_COL_LINK = (255, 255, 255)
_COL_OLD = (64, 224, 208)   # бирюзовый — прежние центры блобов

# Показывать отладочную визуализацию колец/линий/зон
SHOW_DEBUG = True

# Линии ищем раз в LINE_EVERY кадров (в остальные кадры берётся предыдущий результат)
LINE_EVERY = 2

# Как часто печатать в консоль (мс)
PRINT_INTERVAL_MS = 500

# Печатать среднее время этапов кадра (мс), чтобы понять, что тормозит fps
PROFILE = True

# ── Запись на SD-карту (JPEG в один файл .mjpeg + лог .csv) ────────────────────────────────────────
REC_ON = True                   # False — не писать вообще
REC_ROOTS = ("/SD1", "/sd", "/sd1", "/SD")   # где может быть карта (по порядку); дальше ищется любой диск, кроме flash
REC_SUBDIR = "rec"              # папка на карте: r0000.csv (лог) и r0000_00.mjpeg, r0000_01.mjpeg ... (картинки)
REC_IMAGE_EVERY = 3             # картинку писать раз в N кадров (1 — каждый; 0 — картинки не писать)
REC_LOG_EVERY = 1               # строку лога писать раз в N кадров (0 — лог не писать)
REC_QUALITY = 70                # качество JPEG
REC_SEGMENT_IMAGES = 600        # картинок в одном .mjpeg, потом файл закрывается и открывается следующий
REC_FLUSH_MS = 1000             # как часто сбрасывать лог на карту
REC_MIN_FREE_MB = 20            # когда свободно меньше, картинки не пишутся
REC_WITH_HEADING = False
REC_HEADER = "t,f,seg,img,fps,o,bd,blue,yellow,out,ex,ey,cx,cy\n"

# Диапазон условной точки X на прямой синий—жёлтый: от -X_RANGE до +X_RANGE
X_RANGE = 110

# Y = знак * AO, AO от 0 до Y_RANGE
Y_RANGE = 80

# True — синий/жёлтый берём не по центру блоба, а по его краю на прямой между блобами
USE_BLOB_EDGE = True

# Показывать прежние центры блобов и линии по ним (бирюзовым)
SHOW_OLD_CENTERS = False

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


def draw_debug(img, o_blob, y_blob, b_blob, line_point, line_segments, y_pt, b_pt):
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
        img.draw_rectangle(y_blob.rect(), color=_COL_Y, thickness=1)

    if b_blob is not None:
        img.draw_rectangle(b_blob.rect(), color=_COL_B, thickness=1)

    if line_point is not None:
        img.draw_cross(line_point[0], line_point[1], color=_COL_W, size=4)

    # Прежние центры блобов и линии по ним — бирюзовым, по умолчанию скрыты
    if SHOW_OLD_CENTERS:
        if y_blob is not None:
            img.draw_cross(y_blob.cx(), y_blob.cy(), color=_COL_OLD, size=4)
            img.draw_line(INNER_CX, INNER_CY, y_blob.cx(), y_blob.cy(), color=_COL_OLD, thickness=1)
        if b_blob is not None:
            img.draw_cross(b_blob.cx(), b_blob.cy(), color=_COL_OLD, size=4)
            img.draw_line(INNER_CX, INNER_CY, b_blob.cx(), b_blob.cy(), color=_COL_OLD, thickness=1)
        if y_blob is not None and b_blob is not None:
            img.draw_line(y_blob.cx(), y_blob.cy(), b_blob.cx(), b_blob.cy(), color=_COL_OLD, thickness=1)

    # Рабочие точки (краевые, если USE_BLOB_EDGE) как центры блобов и линии между ними
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

def calcDirect(A, B, C):
    """
    Знаковый угол в радианах [-π, π] от луча A→B до луча A→C.
    Положительный — CCW, отрицательный — CW.
    Возвращает радианы: именно их ожидает math.tan() в reloc().
    """
    ax, ay = A
    diff = math.atan2(C[1] - ay, C[0] - ax) - math.atan2(B[1] - ay, B[0] - ax)
    # Нормализация в [-π, π]
    if diff > _PI:
        diff -= _TWO_PI
    elif diff < _PI_NEG:
        diff += _TWO_PI
    return diff

def perpAngles(A, B, C):
    """
    A — центр внутреннего кольца, B — синий блоб, C — жёлтый блоб.
    O — основание перпендикуляра из A на прямую B–C (прямой угол в O).
    Возвращает (угол ABO, угол OAB, O, длина AO); углы в градусах, острые,
    угол ABO + угол OAB = 90°.
    """
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


def lineX(A, B, C):
    """
    Условная точка X на прямой от жёлтых ворот (-X_RANGE) до синих (+X_RANGE).
    A — центр внутреннего кольца, B — синий, C — жёлтый; O — основание
    перпендикуляра из A на B–C. Положение O на отрезке считается через углы:
    |OB| = AO*tan(OAB), |OC| = AO*tan(CAO), t = |OB| / (|OB| + |OC|),
    что равно проекции A на B–C, поэтому считаем t напрямую по координатам
    (без tan около 90°). X = 0 — середина, -X_RANGE — O в жёлтом, +X_RANGE — O в синем.
    Если O вне отрезка B–C, значение обрезается до ±X_RANGE.
    """
    dx = C[0] - B[0]
    dy = C[1] - B[1]
    dd = dx * dx + dy * dy
    if dd == 0:
        return None
    t = ((A[0] - B[0]) * dx + (A[1] - B[1]) * dy) / dd   # 0 в синем, 1 в жёлтом
    x = X_RANGE * (1 - 2 * t)
    return max(-X_RANGE, min(X_RANGE, x))


def lineY(ao, ang):
    """
    Y = знак * AO, где AO (расстояние от центра кольца до прямой синий—жёлтый)
    обрезается до 0..Y_RANGE, знак = x/abs(x) по направлению на ворота:
    x = ang (calcDirect от синего к жёлтому), т.е. с какой стороны от A проходит прямая.
    """
    ao = min(ao, Y_RANGE)
    return ao if ang >= 0 else -ao


def linkPoints(b_blob, y_blob):
    """
    Рабочие «центры» синего и жёлтого блобов. Если найдены оба и USE_BLOB_EDGE —
    это края блобов на прямой между их центрами, иначе обычные центры.
    Возвращает (синяя точка, жёлтая точка); None, если блоб не найден.
    """
    bp = (b_blob.cx(), b_blob.cy()) if b_blob is not None else None
    yp = (y_blob.cx(), y_blob.cy()) if y_blob is not None else None
    if USE_BLOB_EDGE and bp is not None and yp is not None:
        return edgePoint(b_blob, yp), edgePoint(y_blob, bp)
    return bp, yp


def analyze(A, B, C):
    """
    Все величины по трём точкам: A — центр кольца, B — синий, C — жёлтый.
    Возвращает (by°, ABO, OAB, ACO, CAO, O, AO, X, Y) или None, если точки совпадают.
    """
    ang = calcDirect(A, B, C)
    pb = perpAngles(A, B, C)            # треугольник AOB
    pc = perpAngles(A, C, B)            # треугольник AOC
    xv = lineX(A, B, C)
    if pb is None or pc is None or xv is None:
        return None
    return (ang * RAD_TO_DEG, pb[0], pb[1], pc[0], pc[1], pb[2], pb[3], xv, lineY(pb[3], ang))


def fmtAnalysis(r):
    return (f"by={r[0]:.1f} ABO={r[1]:.1f} OAB={r[2]:.1f} ACO={r[3]:.1f} CAO={r[4]:.1f} "
            f"O=({r[5][0]:.1f},{r[5][1]:.1f}) AO={min(r[6], Y_RANGE):.1f} X={r[7]:.1f} Y={r[8]:.1f}")


class Recorder:
    """
    Запись на SD-карту. Картинки — JPEG-кадры в .mjpeg (по REC_SEGMENT_IMAGES штук в файле: закрытый файл
    читается всегда, при отключении питания теряется только последний). Лог — .csv, по строке на кадр:
    t (мс), f (номер кадра), seg и img (файл и номер картинки в нём, -1 — картинки для этого кадра нет),
    fps, o (мяч), bd, blue, yellow (ворота), out (линия), координаты X,Y: ex,ey — по краям блобов,
    cx,cy — по центрам. Пустое значение — блобов синего/жёлтого не было.
    """

    def __init__(self):
        self.ok = False              # карта найдена, сессия открыта
        self.img_on = False          # картинки пишутся
        self.log_on = False          # лог пишется
        self.root = None
        self.dir = None
        self.sess = 0
        self.seg = -1
        self.seg_imgs = 0
        self.mj = None
        self.logf = None
        self.buf = []
        self.last_flush = time.ticks_ms()
        self._q = True               # mjpeg.write принимает quality (проверяется на первом кадре)

    def _candidates(self):
        """Куда пробовать писать: REC_ROOTS, затем остальные диски в корне (кроме flash и rom — rom только для чтения)."""
        c = list(REC_ROOTS)
        try:
            for nm in os.listdir("/"):
                p = "/" + nm
                if nm.lower() not in ("flash", "rom") and p not in c:
                    c.append(p)
        except Exception:
            pass
        return c

    def _try_root(self, root):
        """Годится ли диск: можно создать папку и прочитать её. Возвращает имена файлов в папке или None."""
        d = root + "/" + REC_SUBDIR
        try:
            os.listdir(root)
        except Exception:
            return None                      # такого диска нет
        try:
            os.mkdir(d)
        except OSError:
            pass                             # папка уже есть
        except Exception as e:               # например, VfsRom — диск только для чтения
            print("REC:", root, "не подходит:", e)
            return None
        try:
            return os.listdir(d)
        except Exception as e:
            print("REC:", root, "не подходит:", e)
            return None

    def _find(self):
        for r in self._candidates():
            names = self._try_root(r)
            if names is not None:
                return r, names
        return None, None

    def _mount_sd(self):
        """Прошивка карту не смонтировала (вставлена после включения, не определилась или «залипла») — пробуем сами."""
        try:
            sd = pyb.SDCard()
        except Exception as e:
            print("REC: pyb.SDCard() не создаётся:", e)
            return False
        try:
            print("REC: датчик карты в слоте:", sd.present())
        except Exception:
            pass
        last = None
        for how in range(3):
            try:
                if how == 1:
                    try:
                        os.umount("/sd")                 # вдруг остался «залипший» монтаж
                    except Exception:
                        pass
                if how == 2:
                    os.mount(os.VfsFat(sd), "/sd")       # без автоопределения файловой системы
                else:
                    os.mount(sd, "/sd")
                print("REC: карта смонтирована вручную в /sd (попытка", how + 1, ")")
                return True
            except Exception as e:
                # 16 EBUSY — карта в слоте есть, но не запустилась (инициализация не прошла); 19 ENODEV — нет файловой системы FAT;
                # 5 EIO — ошибка чтения
                print("REC: mount, попытка", how + 1, "не удалась:", e)
                last = e
        if last is not None and last.args and last.args[0] == 19:
            print("REC: карта отвечает, но на ней нет файловой системы FAT — отформатируй её в FAT32 на компьютере "
                  "(или os.VfsFat.mkfs(sd) в консоли: это сотрёт всё на карте)")
        try:
            print("REC: sd.power(True) ->", sd.power(True))
            print("REC: sd.info() =", sd.info())          # None — карта не запустилась
            b = bytearray(512)
            r = sd.readblocks(0, b)                       # при неработающей карте ошибки нет, буфер просто остаётся нулевым
            print("REC: readblocks ->", r, "подпись", b[510], b[511], "(85 170 — таблица разделов/FAT на месте, 0 0 — ничего не прочитано)")
        except Exception as e:
            print("REC: карта не инициализируется:", e)
        return False

    def start(self):
        if not REC_ON:
            return
        self.root, names = self._find()
        if self.root is None and self._mount_sd():
            self.root, names = self._find()      # карта появилась после ручного монтирования
        if self.root is None:
            try:
                disks = os.listdir("/")
            except Exception:
                disks = "?"
            print("REC: карта не найдена, в корне только", disks, "— запись выключена")
            return
        self.dir = self.root + "/" + REC_SUBDIR
        last = -1
        for nm in names:
            if len(nm) >= 5 and nm[0] == "r" and nm[1:5].isdigit():
                n = int(nm[1:5])
                if n > last:
                    last = n
        self.sess = last + 1
        try:
            self.logf = open("%s/r%04d.csv" % (self.dir, self.sess), "w")
            self.logf.write(REC_HEADER)
        except Exception as e:
            print("REC: не открыть лог:", e)
            return
        self.ok = True
        self.log_on = REC_LOG_EVERY > 0
        self.img_on = REC_IMAGE_EVERY > 0 and mjpeg is not None
        if REC_IMAGE_EVERY > 0 and mjpeg is None:
            print("REC: модуля mjpeg нет, пишется только лог")
        print("REC: сессия r%04d в %s, картинка раз в %d кадр, лог раз в %d кадр" % (self.sess, self.dir, REC_IMAGE_EVERY, REC_LOG_EVERY))

    def _free_mb(self):
        try:
            st = os.statvfs(self.root)
            return st[1] * st[4] / 1048576
        except Exception:
            return 1e9               # не смогли узнать — считаем, что место есть

    def _close_seg(self):
        if self.mj is not None:
            try:
                self.mj.close()
            except Exception as e:
                print("REC: mjpeg.close:", e)
            self.mj = None

    def _open_seg(self):
        self._close_seg()
        if self._free_mb() < REC_MIN_FREE_MB:
            print("REC: мало места на карте, картинки больше не пишутся")
            self.img_on = False
            return False
        self.seg += 1
        try:
            self.mj = mjpeg.Mjpeg("%s/r%04d_%02d.mjpeg" % (self.dir, self.sess, self.seg))
        except Exception as e:
            print("REC: не открыть mjpeg:", e)
            self.mj = None
            self.img_on = False
            return False
        self.seg_imgs = 0
        return True

    def add_image(self, img):
        """Дописывает кадр в текущий .mjpeg. Возвращает номер картинки в файле или -1."""
        if self.mj is None or self.seg_imgs >= REC_SEGMENT_IMAGES:
            if not self._open_seg():
                return -1
        try:
            if self._q:
                try:
                    self.mj.write(img, quality=REC_QUALITY)
                except TypeError:
                    self._q = False
                    self.mj.write(img)
            else:
                self.mj.write(img)
        except Exception as e:
            print("REC: картинка не записана:", e)
            self.img_on = False
            self._close_seg()
            return -1
        i = self.seg_imgs
        self.seg_imgs += 1
        return i

    def log(self, now, f, img_idx, fps, o, bd, blue, yellow, out, edge, ctr):
        if edge is not None and ctr is not None:
            xy = "%.1f,%.1f,%.1f,%.1f" % (edge[7], edge[8], ctr[7], ctr[8])
            hd = "%.1f,%.1f," % (edge[9], ctr[9]) if REC_WITH_HEADING else ""
        else:
            xy = ",,,"
            hd = ",," if REC_WITH_HEADING else ""
        self.buf.append("%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%s%s\n" % (
            now, f, self.seg if img_idx >= 0 else -1, img_idx, fps, o, bd, blue, yellow, out, hd, xy))
        if time.ticks_diff(now, self.last_flush) >= REC_FLUSH_MS:
            self.flush()

    def flush(self):
        if self.logf is not None and self.buf:
            try:
                self.logf.write("".join(self.buf))
                self.logf.flush()
            except OSError as e:
                print("REC: лог не записан:", e)
                self.log_on = False
            self.buf = []
        self.last_flush = time.ticks_ms()

    def stop(self):
        self._close_seg()
        self.flush()
        if self.logf is not None:
            try:
                self.logf.close()
            except OSError:
                pass
            self.logf = None
        self.ok = False


def _loop(rec):
    global orangeDirect, blueDirect, yellowDirect, outDirect
    global orangeDist, blueDist, yellowDist, outDist
    global ball_detect

    clock = time.clock()
    last_print = time.ticks_ms()
    prof = [0, 0, 0, 0, 0, 0, 0, 0]       # суммы по этапам, мкс (для PROFILE)
    prof_n = prof_blobs = 0
    line_point, line_segments, line_wait = None, [], 0
    frame_no = 0

    while True:
        clock.tick()
        t0 = time.ticks_us()

        # do img with error protect
        try:
            img = sensor.snapshot()
            img.to_rgb565()
        except Exception as e:
            print("Snapshot error:", e)
            __init__()
            continue
        frame_no += 1
        t1 = time.ticks_us()

        # logic
        # Линии ищем не каждый кадр; в остальные кадры остаётся прошлый результат
        if line_wait <= 0:
            line_point, line_segments = find_all_lines(img)
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

        blue_pt, yellow_pt = linkPoints(best_b_blob, best_y_blob)

        if seen_y:
            cx, cy = yellow_pt
            yellowDirect = normalization(getDirect(cy, cx))
            yellowDist = getDistance(cx, cy)

        if seen_b:
            cx, cy = blue_pt
            blueDirect = normalization(getDirect(cy, cx))
            blueDist = getDistance(cx, cy)

        if seen_l:
            lx, ly = line_point
            a = getDirect(ly, lx)
            outDirect = a + 360 if a < 0 else a
            outDist = getDistance(lx, ly)
        else:
            outDirect = -1

        t4 = time.ticks_us()

        # ── Вся отладочная отрисовка — одним проходом в конце кадра ───────────────
        if SHOW_DEBUG:
            draw_debug(img, best_o_blob, best_y_blob, best_b_blob, line_point, line_segments,
                       yellow_pt, blue_pt)
        t5 = time.ticks_us()

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
        t6 = time.ticks_us()

        # ── Положение по краям блобов (edge) и по их центрам (ctr): нужно и для печати, и для лога ────
        now = time.ticks_ms()
        print_due = time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS
        edge = ctr = None
        if seen_y and seen_b and (print_due or rec.log_on):
            inner = (INNER_CX, INNER_CY)
            edge = analyze(inner, blue_pt, yellow_pt)      # blue_pt/yellow_pt: края (или центры, если USE_BLOB_EDGE=False)
            ctr = analyze(inner, (best_b_blob.cx(), best_b_blob.cy()), (best_y_blob.cx(), best_y_blob.cy()))
        t7 = time.ticks_us()

        # ── Запись на SD-карту: картинка (JPEG в один файл .mjpeg) и строка лога ─────────────────────
        # Картинка пишется в конце кадра, поэтому с SHOW_DEBUG = True в неё попадает и отладочная рисовка.
        if rec.ok:
            img_idx = -1
            if rec.img_on and frame_no % REC_IMAGE_EVERY == 0:
                img_idx = rec.add_image(img)
            if rec.log_on and frame_no % REC_LOG_EVERY == 0:
                rec.log(now, frame_no, img_idx, clock.fps(), orangeDirect, ball_detect,
                        blueDirect, yellowDirect, outDirect, edge, ctr)
        t8 = time.ticks_us()

        if PROFILE:
            prof[0] += time.ticks_diff(t1, t0)     # snapshot + to_rgb565
            prof[1] += time.ticks_diff(t2, t1)     # find_line_segments
            prof[2] += time.ticks_diff(t3, t2)     # find_blobs
            prof[3] += time.ticks_diff(t4, t3)     # перебор блобов + расчёты
            prof[4] += time.ticks_diff(t5, t4)     # отрисовка
            prof[5] += time.ticks_diff(t6, t5)     # pack + trig + I2C
            prof[6] += time.ticks_diff(t7, t6)     # расчёт edge/ctr
            prof[7] += time.ticks_diff(t8, t7)     # запись на SD
            prof_n += 1
            prof_blobs += len(blobs)

        # Вывод не чаще раза в PRINT_INTERVAL_MS, сам цикл не тормозим
        if not print_due:
            continue
        last_print = now

        if PROFILE and prof_n:
            d = 1000.0 * prof_n
            print(f"ms/кадр: snap={prof[0] / d:.1f} lines={prof[1] / d:.1f} blobs={prof[2] / d:.1f} "
                  f"pick={prof[3] / d:.1f} draw={prof[4] / d:.1f} i2c={prof[5] / d:.1f} "
                  f"| всего={sum(prof) / d:.1f} | блобов/кадр={prof_blobs / prof_n:.0f} "
                  f"| calc={prof[6] / d:.1f} rec={prof[7] / d:.1f}")
            prof = [0, 0, 0, 0, 0, 0, 0, 0]
            prof_n = prof_blobs = 0

        # Всё для анализа — одной строкой: по краям блобов (edge) и по их центрам (ctr)
        if edge is not None and ctr is not None:
            print(f"o={orangeDirect} bd={ball_detect} fps={clock.fps():.1f} | "
                  f"edge {fmtAnalysis(edge)} | ctr {fmtAnalysis(ctr)}")
            continue
        print(f"o={orangeDirect} bd={ball_detect} fps={clock.fps():.1f} | нет синего/жёлтого")


def main():
    rec = Recorder()
    rec.start()
    try:
        _loop(rec)
    finally:
        rec.stop()        # закрыть .mjpeg и сбросить лог (и при остановке скрипта из IDE)


if __name__ == "__main__":
    main()
