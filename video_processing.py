import asyncio
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
import cv2
import cv2.aruco as aruco
import logging
from ultralytics import YOLO

# Настройка логирования для отслеживания информации о работе программы
logging.basicConfig(level=logging.INFO)

# Инициализация модели YOLO
model = YOLO("yolov8n")

class Realsense:
    def __init__(self,IP,PORT): #надо порты сделшать
        host = IP
        port = PORT
        self.buffer_size = 10000

        # Инициализация клиентского сокета
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048)
        
    def receive_data(self,sock):
        sock.sendall(f"{0}".encode('utf-8'))
        data = sock.recv(self.buffer_size).decode('utf-8')
        return data
            
    def give_mass(self):
        try:
            # Получение данных от сервера
            data = self.receive_data(self.client_socket)
            return eval(data)
        except Exception as e:
            print(f"Ошибка: {e}")
            
class Connect_aruco: 
    """
    Класс для установления соединения с сервером и обработки данных ArUco.

    Атрибуты:
    - ports (int): Порт, используемый для соединения.
    - data_aruco (str): Данные, полученные от сервера.
    - host (str): IP-адрес сервера.
    - port (int): Порт сервера.
    - server_socket (socket.socket): Сокет сервера.
    - socket_thread (threading.Thread): Поток для асинхронной задачи.

    Методы:
    - __init__(self, IP, PORT, rtsp): Конструктор класса.
    - stop(self): Останавливает соединение и поток.
    - async_task(self): Асинхронная задача для обработки данных.
    - aruco_detect(self, timeout=5): Асинхронно обрабатывает данные от сервера.
    """
    def __init__(self,IP,PORT,rtsp): 
        """
        Конструктор класса.

        Параметры:
        - IP (str): IP-адрес сервера.
        - PORT (int): Порт сервера.
        - rtsp (str): RTSP адрес потока.

        Описание:
        Инициализирует сокет сервера, создает поток для асинхронной задачи и запускает процесс ArUco.
        """
        self.ports=0 
        self.data_aruco=None 
        self.host = IP 
        self.port = PORT 
        port_data = f'{self.host}:{self.port},{rtsp}' # создаем переменную для работы с аргументами между файлов 
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024) 
        self.server_socket.bind((self.host, self.port)) 
        self.server_socket.listen(5) 
        self.socket_thread = threading.Thread(target=self.async_task, daemon=True) 
        self.socket_thread.start() 
        sys.argv.append(port_data) # объявляем метод через sys 
        commands = f'python3.11 aruco.py {" ".join(sys.argv[1:])}'  
        processes = subprocess.Popen(commands, shell=True)
        time.sleep(3) 
 
    def stop(self): 
        """
        Останавливает соединение и поток.

        Описание:
        Завершает процесс ArUco, используя PID, и ожидает завершения потока.
        """
        command = f'netstat -a -n -o | find "{self.ports}"' 
        output = subprocess.check_output(command, shell=True, text=True) 
        pattern = r'ESTABLISHED\s+(\d+)' 
        pids = re.findall(pattern, output) 
        sys='taskkill /F /PID '+str(pids[1]) 
        os.system(sys) 
        self.socket_thread.join() 
 
    def async_task(self): 
        """
        Асинхронная задача для обработки данных.

        Описание:
        Принимает соединения от клиентов и обрабатывает полученные данные.
        """
        try:
            while True: 
                client_socket, addr = self.server_socket.accept() 
                print(f"Connected by {addr}") 
                self.ports=addr[1] 
    
                try: 
                    while True: 
                        data = client_socket.recv(1024).decode('utf-8') 
                        if not data: 
                            break 
                        if not data=='None': 
                            # print(f"Получены данные: {data}") 
                            self.data_aruco=data 
                        else: 
                            # print("Даты нет") 
                            self.data_aruco="" 
                finally: 
                    client_socket.close() 
                    print("Connection closed.") 
        except Exception:
            pass
     
    async def aruco_detect(self,timeout=5):
        """
        Асинхронно обрабатывает данные от сервера.

        Параметры:
        - timeout (int): Время ожидания в секундах.

        Возвращает:
        - str: Данные, полученные от сервера, связанные с маркерами ArUco.
        """ 
        start_time = asyncio.get_event_loop().time() 
        while True: 
            if self.data_aruco[:4] != "null" and self.data_aruco != "":
                return(self.data_aruco)
            else:
                self.data_aruco=""
            await asyncio.sleep(0.1)  # Краткая пауза между итерациями обработки кадров 
 
            if (asyncio.get_event_loop().time() - start_time) > timeout: 
                logging.info("Detection timeout - proceeding with next commands.") 
                break 


class Realsense:
    """
    Класс для работы с сетевыми операциями и получением данных от сервера.

    Атрибуты:
    - buffer_size (int): Размер буфера для приема данных.
    - client_socket (socket.socket): Клиентский сокет для соединения с сервером.

    Методы:
    - __init__(self, IP, PORT): Конструктор класса.
    - receive_data(self, sock): Получение данных от сервера.
    - give_mass(self): Возвращает данные, полученные от сервера.
    """
    def __init__(self,IP,PORT):
        """
        Конструктор класса.

        Параметры:
        - IP (str): IP-адрес сервера.
        - PORT (int): Порт сервера.

        Описание:
        Инициализирует клиентский сокет и устанавливает соединение с сервером.
        """
        host = IP
        port = PORT
        self.buffer_size = 10000

        # Инициализация клиентского сокета
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048)
        
    def receive_data(self,sock):
        """
        Получение данных от сервера.

        Параметры:
        - sock (socket.socket): Сокет, через который происходит прием данных.

        Возвращает:
        - str: Данные, полученные от сервера в виде строки.
        """
        sock.sendall(f"{0}".encode('utf-8'))
        data = sock.recv(self.buffer_size).decode('utf-8')
        return data
            
    def give_mass(self):
        """
        Возвращает данные, полученные от сервера.

        Возвращает:
        - any: Данные, полученные от сервера, преобразованные из строки в объект Python.
        """
        try:
            # Получение данных от сервера
            data = self.receive_data(self.client_socket)
            return eval(data)
        except Exception as e:
            print(f"Ошибка: {e}")
            
class Connect_yolo: 
    """
    Класс для установления соединения с YOLO сервером и обработки данных.

    Атрибуты:
    - ports (int): Порт, используемый для соединения.
    - data_yolo (str): Данные, полученные от YOLO сервера.
    - host (str): IP-адрес сервера.
    - port (int): Порт сервера.
    - server_socket (socket.socket): Сокет сервера.
    - socket_thread (threading.Thread): Поток для асинхронной задачи.

    Методы:
    - __init__(self, IP, PORT, rtsp): Конструктор класса.
    - stop(self): Останавливает соединение и поток.
    - async_task(self): Асинхронная задача для обработки данных.
    - return_port(self): Возвращает используемый порт.
    - yolo_detect(self, timeout=5): Асинхронно обрабатывает данные от YOLO сервера.
    """
    def __init__(self,IP,PORT,rtsp): 
        """
        Конструктор класса.

        Параметры:
        - IP (str): IP-адрес сервера.
        - PORT (int): Порт сервера.
        - rtsp (str): RTSP адрес потока.

        Описание:
        Инициализирует сокет сервера, создает поток для асинхронной задачи и запускает процесс YOLO.
        """
        self.ports=0 
        self.data_yolo=None 
        self.host = IP 
        self.port = PORT 
        port_data = f'{self.host}:{self.port},{rtsp}' # создаем переменную для работы с аргументами между файлов 
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024) 
        self.server_socket.bind((self.host, self.port)) 
        self.server_socket.listen(5) 
        self.socket_thread = threading.Thread(target=self.async_task, daemon=True) 
        self.socket_thread.start() 
        sys.argv.append(port_data) # объявляем метод через sys 
        commands = f'python3.11 yolo.py {" ".join(sys.argv[1:])}'  
        processes = subprocess.Popen(commands, shell=True)
        time.sleep(10) 
 
    def stop(self): 
        """
        Останавливает соединение и поток.

        Описание:
        Завершает процесс YOLO, используя PID, и ожидает завершения потока.
        """
        command = f'netstat -a -n -o | find "{self.ports}"' 
        output = subprocess.check_output(command, shell=True, text=True) 
        pattern = r'ESTABLISHED\s+(\d+)' 
        pids = re.findall(pattern, output) 
        sys='taskkill /F /PID '+str(pids[1]) 
        os.system(sys) 
        self.socket_thread.join() 
 
    def async_task(self): 
        """
        Асинхронная задача для обработки данных.

        Описание:
        Принимает соединения от клиентов и обрабатывает полученные данные.
        """
        try:
            while True: 
                client_socket, addr = self.server_socket.accept() 
                print(f"Connected by {addr}") 
                self.ports=addr[1] 
    
                try: 
                    while True: 
                        data = client_socket.recv(1024).decode('utf-8') 
                        if not data: 
                            break 
                        if not data=='null': 
                            # print(f"Получены данные: {data}") 
                            self.data_yolo=data 
                        else: 
                            # print("Даты нет") 
                            self.data_yolo=None 
                finally: 
                    client_socket.close() 
                    print("Connection closed.") 
        except Exception:
            pass
     
    async def yolo_detect(self,timeout=5): 
        """
        Асинхронно обрабатывает данные от YOLO сервера.

        Параметры:
        - timeout (int): Время ожидания в секундах.

        Возвращает:
        - tuple: Кортеж с классами и ограничивающими рамками, обнаруженными YOLO.
        """
        start_time = asyncio.get_event_loop().time() 
        while True: 
            if not self.data_yolo==None: 
                data = json.loads(self.data_yolo) 
                # Извлечение массивов из словаря 
                boxes = data['boxes'] 
                config = data['conf'] 
                classes = data['class'] 
                return (classes, boxes) 
            await asyncio.sleep(0.1)  # Краткая пауза между итерациями обработки кадров 
 
            if (asyncio.get_event_loop().time() - start_time) > timeout: 
                logging.info("Detection timeout - proceeding with next commands.") 
                break 

        
