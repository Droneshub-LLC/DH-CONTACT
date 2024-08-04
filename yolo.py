# инициализация библиотек
import sys
import socket 
import json 
from ultralytics import YOLO  # Убедитесь, что у вас установлена версия, поддерживающая YOLOv8n 
 

 
# Инициализация нейросети YOLOv8n 
model = YOLO('yolov8n.pt')  

# подключение к серверу с помощью sys с аргументами к общему файлу video_processing.py
if len(sys.argv) > 1:
    port_data = sys.argv[-1]
    # print(port_data)
    port_data = port_data.split(',')
    ports = port_data[0].split(':')
    rtsp_url=port_data[1]
    socks = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    socks.connect((ports[0], int(ports[1]))) 
else:
    print("данных нет")
    exit()

# основной код: здесь YOLO работает по предварительно обученной модели.
# Так же высылает данные на сервер в формате .json, а именно: boxes, conf, и cls
try: 

    results = model(rtsp_url, stream=True,verbose=False)  # Обработка кадра через модель YOLO 
    # Обработка результатов 
    for result in results: 
        boxes = result.boxes.xyxy.tolist()  # Boxes object for bounding box outputs 
        conf = result.boxes.conf.tolist() 
        YOLO_cls = result.boxes.cls.tolist() 

        data = { 
        'boxes': boxes, 
        'conf': conf, 
        'class': YOLO_cls         
        } 
        if(boxes==[] and conf==[] and YOLO_cls==[]):
            data=None
        # print('boxes') 
        json_data = json.dumps(data) 
        # sock.sendall(json_data.encode('utf-8')) 
        socks.sendall(json_data.encode('utf-8')) 

except Exception as e: 
    print(f"Произошла ошибка: {e}") 

finally: 
    # cap.release() 
    # sock.close()
    socks.close()