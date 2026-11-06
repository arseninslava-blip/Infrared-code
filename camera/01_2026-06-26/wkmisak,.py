# ═══════════════════════════════════════════════════════════════════════════════
#  НАСТРОЙКИ ЭКСПОЗИЦИИ И ПОРОГОВ
# ═══════════════════════════════════════════════════════════════════════════════
EXPOSURE_US = 8000

thresholds = [(0, 100, -31, 14, 28, 94),   # Yellow
              (0, 100, -128, 127, -93, -48)]  # Blue

cX = 65
cY = 65

INNER_R  = 10
OUTER_R  = 80
INNER_R2 = INNER_R * INNER_R
OUTER_R2 = OUTER_R * OUTER_R

import sensor, image, math, pyb, time

led = pyb.LED(3)

# ═══════════════════════════════════════════════════════════════════════════════
#  ИНИЦИАЛИЗАЦИЯ КАМЕРЫ
# ═══════════════════════════════════════════════════════════════════════════════
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(108, 65, 130, 130)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=1500)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.set_hmirror(False)
sensor.set_vflip(False)
sensor.skip_frames(time=2000)

clock = time.clock()

# ═══════════════════════════════════════════════════════════════════════════════
#  ТАБЛИЦА РЕАЛЬНОГО РАССТОЯНИЯ
# ═══════════════════════════════════════════════════════════════════════════════
goalCoords = [[0, 0],
              [10, 22], [15, 29], [20, 34], [30, 40], [40, 46],
              [50, 49], [60, 51], [70, 55], [80, 55], [90, 57],
              [100, 57], [110, 57], [120, 58], [130, 58],
              [140, 59], [150, 59], [160, 60], [170, 60]]

def realDistance(x):
    fSign = 1
    if x < 0:
        xAbs = -x
        fSign = -1
    else:
        xAbs = x
    for i in range(len(goalCoords) - 1):
        if goalCoords[i][1] <= xAbs < goalCoords[i + 1][1]:
            x1, x2 = goalCoords[i][1], goalCoords[i + 1][1]
            y1, y2 = goalCoords[i][0], goalCoords[i + 1][0]
            return ((xAbs - x1) * (y2 - y1) / (x2 - x1) + y1) * fSign
    x1 = goalCoords[-2][1]; x2 = goalCoords[-1][1]
    y1 = goalCoords[-2][0]; y2 = goalCoords[-1][0]
    return ((xAbs - x1) * (y2 - y1) / (x2 - x1) + y1) * fSign

# ═══════════════════════════════════════════════════════════════════════════════
#  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ═══════════════════════════════════════════════════════════════════════════════
def dot(x, y, color=(255, 0, 0), sz=5):
    x -= sz / 2
    y -= sz / 2
    for i in range(sz * sz):
        img.set_pixel(int(x) + (i % sz) - 1, int(y) + int(i / sz) - 1, color)

def blob_in_ring(cx, cy):
    dx = cx - cX
    dy = cy - cY
    r2 = dx * dx + dy * dy
    return INNER_R2 < r2 < OUTER_R2

def getCenterBetween(blob1, blob2):
    if blob1[2] * blob1[3] == 0 or blob2[2] * blob2[3] == 0:
        return (0, 0)
    if blob1[0] > blob2[0]:
        gkX = -(blob1[0] + blob2[0] + blob2[2]) / 2 + cX
    else:
        gkX = -(blob1[0] + blob1[2] + blob2[0]) / 2 + cX
    gkY = -(blob1[1] + blob1[3] / 2 + blob2[1] + blob2[3] / 2) / 2 + cY
    return (gkX, gkY)

def partition(array, begin, end):
    pivot = begin
    for i in range(begin + 1, end + 1):
        if array[i][0] <= array[begin][0]:
            pivot += 1
            array[i], array[pivot] = array[pivot], array[i]
    array[pivot], array[begin] = array[begin], array[pivot]
    return pivot

def quicksort(array, begin=0, end=None):
    if end is None:
        end = len(array) - 1
    def _qs(array, begin, end):
        if begin >= end:
            return
        p = partition(array, begin, end)
        _qs(array, begin, p - 1)
        _qs(array, p + 1, end)
    return _qs(array, begin, end)

# ═══════════════════════════════════════════════════════════════════════════════
#  ОСНОВНОЙ ЦИКЛ
# ═══════════════════════════════════════════════════════════════════════════════
while True:
    img = sensor.snapshot()
    img.draw_circle(cX, cY-1, OUTER_R, color=(0, 0, 0), thickness=32)
    img.draw_circle(cX,  cY, INNER_R,      color=(0, 0, 0), thickness=18)

    yFlag = bFlag = 0
    trueyX = trueyY = truebX = truebY = 0
    yGkX = yGkY = bGkX = bGkY = 0
    yAngle = yDist = bAngle = bDist = 0
    yBiggestBlobAngle = bBiggestBlobAngle = 0

    yBlob = [[0] * 4, [0] * 4]
    bBlob = [[0] * 4, [0] * 4]
    firstYArea = secondYArea = firstBArea = secondBArea = 0
    blobsY = []
    blobsB = []

    for blob in img.find_blobs(thresholds,
                                pixels_threshold=50,
                                area_threshold=50,
                                merge=True,
                                margin=20):
        bcx, bcy = blob.cx(), blob.cy()
        if not blob_in_ring(bcx, bcy):
            continue

        if blob.code() == 1:
            area = blob[2] * blob[3]
            if area > firstYArea:
                secondYArea = firstYArea; yBlob[1] = yBlob[0]
                firstYArea = area;        yBlob[0] = blob.rect()
            elif area > secondYArea:
                secondYArea = area;       yBlob[1] = blob.rect()
            yFlag = 1
            blobsY.append(blob.rect())

        if blob.code() == 2:
            area = blob[2] * blob[3]
            if area > firstBArea:
                secondBArea = firstBArea; bBlob[1] = bBlob[0]
                firstBArea = area;        bBlob[0] = blob.rect()
            elif area > secondBArea:
                secondBArea = area;       bBlob[1] = blob.rect()
            bFlag = 1
            blobsB.append(blob.rect())

    # ── Жёлтые блобы ──────────────────────────────────────────────────────────
    if yFlag:
        quicksort(blobsY, 0, len(blobsY) - 1)
        for i in range(len(blobsY) - 1):
            gkDot = getCenterBetween(blobsY[i], blobsY[i + 1])
            img.draw_rectangle(blobsY[i], color=(255, 255, 0), fill=0)
            dot(cX - gkDot[0], cY - gkDot[1], color=(255, 255, 255))
        img.draw_rectangle(blobsY[-1], color=(255, 255, 0), fill=0)

        biggestBlob = max(blobsY, key=lambda b: b[2] * b[3])
        dot(biggestBlob[0] + biggestBlob[2] / 2,
            biggestBlob[1] + biggestBlob[3] / 2, color=(54, 123, 11))
        yBiggestBlobAngle = int(
            math.atan2(cX - (biggestBlob[0] + biggestBlob[2] / 2),
                       cY - (biggestBlob[1] + biggestBlob[3] / 2)) / 3.14 * 180)

        yBlob[0] = min(blobsY, key=lambda b: b[0])
        yBlob[1] = max(blobsY, key=lambda b: b[0] + b[2])

        if yBlob[1][2] * yBlob[1][3] == 0:
            trueyX = -(yBlob[0][0] + yBlob[0][2] / 2) + cX
            trueyY = -(yBlob[0][1] + yBlob[0][3] / 2) + cY
        elif yBlob[0][0] < yBlob[1][0]:
            trueyX = -(yBlob[0][0] + yBlob[1][0] + yBlob[1][2]) / 2 + cX
            trueyY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY
        else:
            trueyX = -(yBlob[0][0] + yBlob[0][2] + yBlob[1][0]) / 2 + cX
            trueyY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY

        if yBlob[1][2] * yBlob[1][3] != 0:
            if yBlob[0][0] > yBlob[1][0]:
                yGkX = -(yBlob[0][0] + yBlob[1][0] + yBlob[1][2]) / 2 + cX
                yGkY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY
            else:
                yGkX = -(yBlob[0][0] + yBlob[0][2] + yBlob[1][0]) / 2 + cX
                yGkY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY

    # ── Синие блобы ───────────────────────────────────────────────────────────
    if bFlag:
        quicksort(blobsB, 0, len(blobsB) - 1)
        for i in range(len(blobsB) - 1):
            gkDot = getCenterBetween(blobsB[i], blobsB[i + 1])
            img.draw_rectangle(blobsB[i], color=(0, 0, 255), fill=0)
            dot(cX - gkDot[0], cY - gkDot[1], color=(255, 255, 255))
        img.draw_rectangle(blobsB[-1], color=(0, 0, 255), fill=0)

        biggestBlob = max(blobsB, key=lambda b: b[2] * b[3])
        dot(biggestBlob[0] + biggestBlob[2] / 2,
            biggestBlob[1] + biggestBlob[3] / 2, color=(54, 123, 11), sz=10)
        bBiggestBlobAngle = int(
            math.atan2(cX - (biggestBlob[0] + biggestBlob[2] / 2),
                       cY - (biggestBlob[1] + biggestBlob[3] / 2)) / 3.14 * 180)

        bBlob[0] = min(blobsB, key=lambda b: b[0])
        bBlob[1] = max(blobsB, key=lambda b: b[0] + b[2])

        img.draw_rectangle(bBlob[0], color=(0, 0, 255), fill=0)
        img.draw_rectangle(bBlob[1], color=(0, 0, 255), fill=0)

        if bBlob[1][2] * bBlob[1][3] == 0:
            truebX = -(bBlob[0][0] + bBlob[0][2] / 2) + cX
            truebY = -(bBlob[0][1] + bBlob[0][3] / 2) + cY
        elif bBlob[0][0] < bBlob[1][0]:
            truebX = -(bBlob[0][0] + bBlob[1][0] + bBlob[1][2]) / 2 + cX
            truebY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY
        else:
            truebX = -(bBlob[0][0] + bBlob[0][2] + bBlob[1][0]) / 2 + cX
            truebY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY

        if bBlob[1][2] * bBlob[1][3] != 0:
            if bBlob[0][0] > bBlob[1][0]:
                bGkX = -(bBlob[0][0] + bBlob[1][0] + bBlob[1][2]) / 2 + cX
                bGkY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY
            else:
                bGkX = -(bBlob[0][0] + bBlob[0][2] + bBlob[1][0]) / 2 + cX
                bGkY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY

    # ── Визуализация центра и точек ───────────────────────────────────────────
    dot(cX, cY, color=(0, 255, 0), sz=10)
    dot(cX - trueyX, cY - trueyY)
    dot(cX - truebX, cY - truebY)

    # ── Вычисление углов и расстояний ─────────────────────────────────────────
    yAngle = int(math.atan2(trueyX, trueyY) / 3.14 * 180)
    yDist  = int(math.sqrt(trueyX * trueyX + trueyY * trueyY))
    bAngle = int(math.atan2(truebX, truebY) / 3.14 * 180)
    bDist  = int(math.sqrt(truebX * truebX + truebY * truebY))

    yDist = realDistance(yDist)
    bDist = realDistance(bDist)

    if yFlag == 0: yDist = 0
    if bFlag == 0: bDist = 0

    if yAngle < 0: yAngle = int(yAngle + 360)
    if bAngle < 0: bAngle = int(bAngle + 360)
    if yAngle < 0: yAngle = 0
    if bAngle < 0: bAngle = 0
    if yDist  < 0: yDist  = 0
    if bDist  < 0: bDist  = 0

    bDist = int(bDist)
    yDist = int(yDist)

    print(yAngle, yDist, bAngle, bDist)
