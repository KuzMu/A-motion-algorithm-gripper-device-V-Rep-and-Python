import vrep
import sys
import time
import cv2
import numpy as np
import math

def nothing(x):
    pass
def objectSelection(image_RGB,resolution_RGB):
    img = np.array(image_RGB, dtype=np.uint8)
    img.resize([resolution_RGB[0], resolution_RGB[1], 3])
    img = np.flipud(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # вызываем фильтр для выделения объекта
    cv2.namedWindow("result", cv2.WINDOW_NORMAL)  # главное окно с маской
    cv2.namedWindow("settings", cv2.WINDOW_AUTOSIZE)  # окно настроек
    cv2.namedWindow("RGB", cv2.WINDOW_NORMAL) # RGB изображение

    # создание бегунков для настройки начального и конечного цвета фильтра
    cv2.createTrackbar('h1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('s1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('v1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('h2', 'settings', 255, 255, nothing)
    cv2.createTrackbar('s2', 'settings', 255, 255, nothing)
    cv2.createTrackbar('v2', 'settings', 255, 255, nothing)

    while True:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # считываем значения бегунков
        h1 = cv2.getTrackbarPos('h1', 'settings')
        s1 = cv2.getTrackbarPos('s1', 'settings')
        v1 = cv2.getTrackbarPos('v1', 'settings')
        h2 = cv2.getTrackbarPos('h2', 'settings')
        s2 = cv2.getTrackbarPos('s2', 'settings')
        v2 = cv2.getTrackbarPos('v2', 'settings')

        #начальный и конечный цвет фильтра
        h_min = np.array((h1, s1, v1), np.uint8)
        h_max = np.array((h2, s2, v2), np.uint8)

        #фильтр
        thresh = cv2.inRange(hsv, h_min, h_max)
        cv2.imshow('result', thresh)
        cv2.imshow('RGB', img)

        exi_but = cv2.waitKey(5)
        if exi_but == 27:
            cv2.destroyWindow("settings")
            cv2.destroyWindow("result")
            cv2.destroyWindow("RGB")
            break
    return h_min, h_max


def quatRotation(y_x, x_x, y_y, x_y, y_z, x_z, y_w, x_w):
    res_w = y_w * x_w - y_x * x_x - y_y * x_y - y_z * x_z
    res_x = y_w * x_x + y_x * x_w + y_y * x_z - y_z * x_y
    res_y = y_w * x_y - y_x * x_z + y_y * x_w + y_z * x_x
    res_z = y_w * x_z + y_x * x_y - y_y * x_x + y_z * x_w
    return [res_x, res_y, res_z, res_w]

def TorF(value1, value2):
    bool_ = True if value2 - value1 > 0 else False
    return bool_

#выделение внешних контуров и внутренних
def pixInContour(copy_array_depth, color_pix, x_or_y):
    if x_or_y == 'x':
        i = 0
        while i < copy_array_depth.shape[1]:
            j = 0
            firs_in_line_contour = 0
            while j < copy_array_depth.shape[0] - 1:
                if copy_array_depth[i][j] != color_pix:
                    if copy_array_depth[i][j + 1] != color_pix:
                        if firs_in_line_contour == 0 and copy_array_depth[i][j - 1] == color_pix:
                            firs_in_line_contour = 1
                            sign_pix_x = TorF(copy_array_depth[i][j + 1], copy_array_depth[i][j])
                        else:
                            if sign_pix_x == TorF(copy_array_depth[i][j + 1], copy_array_depth[i][j]):
                                copy_array_depth[i][j] = color_pix
                            else:
                                if copy_array_depth[i][j + 1] - copy_array_depth[i][j] != 0:
                                    sign_pix_x = TorF(copy_array_depth[i][j + 1], copy_array_depth[i][j])
                                else:
                                    copy_array_depth[i][j] = color_pix
                    else:
                        firs_in_line_contour = 0
                j += 1
            i += 1
        return copy_array_depth
    else:
        j = 0
        while j < copy_array_depth.shape[0]:
            i = 0
            firs_in_line_contour = 0
            while i < copy_array_depth.shape[1] - 1:
                if copy_array_depth[i][j] != color_pix:
                    if copy_array_depth[i + 1][j] != color_pix:
                        if firs_in_line_contour == 0 and copy_array_depth[i - 1][j] == color_pix:
                            firs_in_line_contour = 1
                            sign_pix_y = TorF(copy_array_depth[i+1][j], copy_array_depth[i][j])
                        else:
                            if sign_pix_y == TorF(copy_array_depth[i+1][j], copy_array_depth[i][j]):
                                copy_array_depth[i][j] = color_pix
                            else:
                                if copy_array_depth[i+1][j] - copy_array_depth[i][j] != 0 :
                                    sign_pix_y = TorF(copy_array_depth[i+1][j], copy_array_depth[i][j])
                                else:
                                    copy_array_depth[i][j] = color_pix
                    else:
                        firs_in_line_contour = 0
                i += 1
            j += 1
        return copy_array_depth

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote server")
else:
    print('Connection not successful')
    sys.exit('Could not connect')

err, target_handle = vrep.simxGetObjectHandle(clientID, 'target',
                                              vrep.simx_opmode_oneshot_wait) # запрос на использвоание целевой точки
err, driven_pint = vrep.simxGetObjectHandle(clientID, 'Dummy1',
                                            vrep.simx_opmode_oneshot_wait) # запрос на использование ведомой точки
err, camera_RGB_handle = vrep.simxGetObjectHandle(clientID, 's1',
                                                  vrep.simx_opmode_oneshot_wait) # запрос на использование камеры
err, camera_depth_handle = vrep.simxGetObjectHandle(clientID, 's2',
                                                    vrep.simx_opmode_oneshot_wait) # запрос на использование камеры глубины
err, laser_handle = vrep.simxGetObjectHandle(clientID, 'laserR',
                                                    vrep.simx_opmode_oneshot_wait)
err, VelocityU_handle = vrep.simxGetObjectHandle(clientID, 'active1',
                                                    vrep.simx_opmode_oneshot_wait)
err, VelocityD_handle = vrep.simxGetObjectHandle(clientID, 'active2',
                                                    vrep.simx_opmode_oneshot_wait)

_, resolution_RGB, image_RGB = vrep.simxGetVisionSensorImage(clientID, camera_RGB_handle, 0, vrep.simx_opmode_streaming)
_, resolution_Depth, image_depth = vrep.simxGetVisionSensorDepthBuffer(clientID, camera_depth_handle,
                                                                       vrep.simx_opmode_streaming)
_, _, laser, _, _ = vrep.simxReadProximitySensor(clientID, laser_handle, vrep.simx_opmode_streaming)
time.sleep(1)

camera_angle = 60  # задается в V-Rep
angle_in_pixel = camera_angle / 256 # делим на количество пикселей
rad_in_pixel = math.radians(angle_in_pixel)

# создаем кватернион тангажа, рысканья, крена
# рысканье
y_x = 0
y_y = 0
y_z = 0
y_w = 0
# тангаж
x_x = 0
x_y = 0
x_z = 0
x_w = 0
# крен
z_x = 0
z_y = 0
z_z = 0
z_w = 0

# габаритные ограничения захватного устройства
S1 = 9.8  # 9.799957275390625
S2 = 37  # 36.99999302625656
S3 = 16.1  # 16.100287437438965
S4 = 143  # 142.99997710622847
S5 = 84.3  # размер между рабочими элементами
S6 = 12  # 11.999979615211487
S7 = 96  # 95.99971771240234
S8 = 180  # 180.00018596649417
S9 = 53  # 52.999913692474365
S10 = 52  # 51.99992656707764
S11 = 63 #расстояние от лазерного датчика до конца рабочих элементов

L1 = 24  # 23.999691009521484
L2 = 66  # 65.99986553192139

door = 0 #триггер входа в основной цикл

while vrep.simxGetConnectionId(clientID) != -1:

    # vrep.simxSetJointTargetVelocity(clientID, VelocityD_handle, -0.001,vrep.simx_opmode_oneshot_wait) #скорость движения выходных веньев
    # vrep.simxSetJointTargetVelocity(clientID, VelocityU_handle, -0.001, vrep.simx_opmode_oneshot_wait)

    _, resolution_RGB, image_RGB = vrep.simxGetVisionSensorImage(clientID, camera_RGB_handle, 0,
                                                                 vrep.simx_opmode_buffer)
    img = np.array(image_RGB, dtype=np.uint8)
    img.resize([resolution_RGB[0], resolution_RGB[1], 3])
    img = np.flipud(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # преобразуем изображение в цветовую схему BGR, для OpenCV
    # обнаружение объекта по зеленому цвету
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # low_border_green = np.array([49, 50, 50], dtype=np.uint8)
    # high_border_green = np.array([80, 255, 255], dtype=np.uint8)
    if door == 0:
        low_border_green, high_border_green = objectSelection(image_RGB, resolution_RGB)
        door = 1
    mask = cv2.inRange(hsv, low_border_green, high_border_green)  # задается диапазон зеленого оттенка

    # вычисление центра объекта в пикселях и выводим на экран
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 200):
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
        cv2.circle(img, (x, y), 1, (0, 0, 255), -1)  # рисуем центр объекта
    cv2.namedWindow('Image', cv2.WINDOW_NORMAL)  # создаем окно с изменяемым размером
    cv2.imshow('Image', img)  # загружаем изображением в цветовой сехме BGR в окно
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask', mask)
    exi_but = cv2.waitKey(5) & 0xFF
    if exi_but == 27:
        break
    print(x,y)

    # определяем необходимый угол поворота ЗУ до объекта относительно камеры
    if not(x-1 <= resolution_RGB[1] / 2 <= x+1)or not(y-1 <= resolution_RGB[0] / 2 <= y+1):
        # расчет углов поворота
        angle_y = rad_in_pixel * ((resolution_RGB[0] / 2) - x)
        angle_x = rad_in_pixel * (y - (resolution_RGB[1] / 2))
        print(angle_x, angle_y)
        y_y = math.sin(angle_y / 2)
        y_w = math.cos(angle_y / 2)

        x_x = math.sin(angle_x / 2)
        x_w = math.cos(angle_x / 2)

        quat_res = quatRotation(y_x, x_x, y_y, x_y, y_z, x_z, y_w, x_w)
        vrep.simxSetObjectQuaternion(clientID, target_handle, target_handle, quat_res,
                                       vrep.simx_opmode_oneshot_wait)
        time.sleep(0.04)
        continue

    # запрос изображения с камеры глубины, движение к объекту
    _, resolution_RGB, image_depth = vrep.simxGetVisionSensorDepthBuffer(clientID, camera_depth_handle,
                                                                         vrep.simx_opmode_buffer)

    Array_depth = np.array(image_depth)
    Array_depth = Array_depth.reshape((resolution_RGB[0], resolution_RGB[1]))
    Array_depth = np.flipud(Array_depth)

    distance_centroid = Array_depth[y, x] * 10
    print(distance_centroid)
    print(area)

    if distance_centroid > S8 * 0.001 + 0.05 and area < 1600000:
        if distance_centroid > 2:
            distance_step_grip = 0.5  # в симулятор подается дистанция в метрах
        elif 0.2 < distance_centroid:
            distance_step_grip = 0.01
        else:
            distance_step_grip = 0.001
        vrep.simxSetObjectPosition(clientID, target_handle, target_handle, [0, 0, distance_step_grip],
                                   vrep.simx_opmode_oneshot_wait)
        continue

    color_pix = 255
    copy_array_depth = Array_depth.copy()
    i = 0
    j = 0
    while i < copy_array_depth.shape[1]:
        for k in mask[i]:
            if k == 0:
                copy_array_depth[i][j] = color_pix
            j += 1
        i += 1
        j = 0
    # copy_array_depth = [[j if j !=0 else color_pix for j in i] for i in Array_depth.copy()]
    # copy_array_depth = np.array(copy_array_depth)

    contour_x = pixInContour(copy_array_depth.copy(), color_pix, x_or_y='x')
    contour_y = pixInContour(copy_array_depth.copy(), color_pix, x_or_y='y')

    #внешний контур в сборе
    i = 0
    while i < copy_array_depth.shape[1]:
        j = 0
        while j < copy_array_depth.shape[0]:
            if contour_x[i][j] != contour_y[i][j]:
                if contour_x[i][j] == color_pix:
                    contour_x[i][j] = contour_y[i][j]
            j += 1
        i += 1

    cv2.namedWindow('Mask_depth_object', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask_depth_object', contour_x)

    k = 0
    for i in contour_x[y]: #опредление формы объекта; в зависимости от линий пересечения
        if i != color_pix:
            k += 1

    f_in_line = 0
    s_pix = 0
    for i in contour_x[y]: #подсчет пикселей
        if f_in_line != 0 and f_in_line != i:
            s_pix += 1
            continue
        elif f_in_line == i:
            s_pix += 1
            break
        if i != color_pix:
            f_in_line = i
            s_pix += 1
    _, _, laser, _, _ = vrep.simxReadProximitySensor(clientID, laser_handle, vrep.simx_opmode_buffer)
    measure = ((4.5821 * 10 ** (-3)) * laser[2] + (-9.6641 * 10 ** (-5))) * s_pix  #вычисление реальных размеров объекта


vrep.simxFinish(clientID)
