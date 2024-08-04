"""
Данный файл нужен для работы с метками Aruco. 

В данном коде представлены подключение к серверу, обнаружение меток AruCo с помощью Open cv и rtsp потока, а так же обрисовка самой метки

"""
# импорт библиотек
import cv2
import sys
import socket
from cv2 import aruco

# подключение к серверу с помощью sys с атрибутами через общий файл video_processing.py 
if len(sys.argv) > 1:
    port_data = sys.argv[-1] 
    print(port_data)
    port_data = port_data.split(',')
    ports = port_data[0].split(':')
    rtsp_url = port_data[1]
    socks = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socks.connect((ports[0], int(ports[1])))
else:
    print("данных нет")
    exit()

# Инициализация камеры с помощью rtps - потока и работоспособность распознования AruCo меток
try:
    cap = cv2.VideoCapture(rtsp_url)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters()

    while True:
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Обрисовка обнаруженных маркеров ArUco
            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
                # Отправка данных через сокет, если необходимо
                socks.sendall(f"{ids.flatten().tolist()}".encode('utf-8'))
            else:
            #     # Отправка данных через сокет, если не обнаружено маркеров
                socks.sendall("null".encode())

                

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Не удалось захватить изображение")
            break
        cv2.imshow('Frame with ArUco Markers', frame)
except Exception as e:
    pass

# закрытие потоков, окна камеры, и подключения к серверу
finally:
    cap.release()
    socks.close()
    cv2.destroyAllWindows()
