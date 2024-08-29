import asyncio
import contextlib
import socket
import threading
import aiofiles
import logging
from contextlib import suppress
from Server_Controller import Attachments, Site_run
from queue import SimpleQueue
from Contact_state import ContactState
import plotly.graph_objs as go
from Contact_controller import SimulationController, RoverController
from video_processing import Connect_aruco, Connect_yolo

# Настройка логирования для отслеживания информации о работе программы
logging.basicConfig(level=logging.INFO)

# Конфигурация подключения
IP_ADDRESS      = socket.gethostbyname(socket.gethostname())
SERVER_PORT     = 3006
RTSP_PORT       = 8554
PORT_YOLO       = 12345
PORT_ARUCO      = 6690
PORT_REALSENSE  = 5527
PORT_SERVO      = 5520
PORT_RELE       = 5525
SERVER_URL      = f'http://{IP_ADDRESS}:{SERVER_PORT}/'
RTSP_LINK       = f'rtsp://{IP_ADDRESS}:{RTSP_PORT}/web'
TOKEN           = 'eyJhbGciOiJIUzI1NiIsInR5cCI6Imp3dCJ9.eyJ1c2VybmFtZSI6InRlc3QiLCJpZCI6MX0=.OrS7h72zhJH1EY49vpxiO1FizR71r/SHSoSky87I728='  # Токен аутентификации
HOST_SOCKET     = 'localhost'
PORT_SOCKET     = 5530
PORT_SITE       = 8888


# создание события. В дальнейшем пригодится для корректного закрытия потоков
event = asyncio.new_event_loop()
asyncio.set_event_loop(event)

video_frame_queue = SimpleQueue()  # Создаем очередь для хранения кадров видео
robot_state = ContactState(0, 0, 0)  # Инициализируем состояние робота с начальными координатами и ориентацией
sayt_start=asyncio.Event() 
sayt_stop=asyncio.Event()# Создаем событие для остановки видеопотока
thread_running = threading.Event()
thread_running.set()
          
async def load_aruco_map(filename):
    """
    Загрузка данных карты ArUco из файла.

    Библиотеки:
    - aiofiles: для асинхронного чтения файлов

    Параметры:
    - filename (str): Имя файла, содержащего данные карты ArUco.

    Возвращает:
    - dict: Словарь с данными карты ArUco, где ключом является идентификатор маркера, а значениями - координаты (x, y).

    Описание:
    Функция асинхронно считывает файл, содержащий данные карты ArUco, и возвращает их в виде словаря. 
    Каждая строка файла должна содержать идентификатор маркера и его координаты.
    """
    aruco_map = {}
    async with aiofiles.open(filename, mode='r', encoding='utf-8') as file:
        async for line in file:
            parts = line.strip().split()
            if len(parts) == 8:  # Проверка формата строки
                marker_id = int(parts[0])
                x = float(parts[2])
                y = float(parts[3])
                aruco_map[marker_id] = {'x': x, 'y': y}
    return aruco_map    
    
    
async def main(use_simulator):
    """
    Основная функция, координирующая работу всех компонентов.

    Библиотеки:
    - asyncio: для асинхронного выполнения задач
    - logging: для ведения логов
    - concurrent.futures.ThreadPoolExecutor: для параллельного выполнения потоков
    - queue.SimpleQueue: для организации очереди кадров видео
    - matplotlib: для визуализации в симуляторе

    Параметры:
    - use_simulator (bool): Флаг для использования симулятора.

    Описание:
    Основная функция, которая координирует работу всех компонентов системы: загрузку карты ArUco, управление 
    состоянием робота, поток видеопотока, обнаружение маркеров ArUco и объектов, и управление роботом или 
    симулятором. 
    """
    
    
    aruco_map = await load_aruco_map("region.txt")  # Загружаем карту маркеров ArUco из файла        
    if use_simulator:
        # Импортируем matplotlib для визуализации в симуляторе
        site=Site_run(HOST_SOCKET,PORT_SITE,HOST_SOCKET,PORT_SOCKET,RTSP_LINK)
        ax = go.Figure(layout=go.Layout(width=700,height=800))  
        ax.update_layout(showlegend=False)
        controller = SimulationController(aruco_map, robot_state, ax,HOST_SOCKET,PORT_SOCKET)
    else:
        # Инициализируем контроллер робота для реального управления
        controller = RoverController(aruco_map, robot_state)
    
    # инициализация сервера yolo для дальнейшей работы с ней
    yolo=Connect_yolo(IP_ADDRESS,PORT_YOLO,RTSP_LINK)
    # инициализация сервера aruco для дальнейшей работы с ней
    aruco=Connect_aruco(IP_ADDRESS,PORT_ARUCO,RTSP_LINK)
    marker_ids = await aruco.aruco_detect()  # Обнаруживаем маркеры ArUco в видеопотоке
    if marker_ids:
        logging.info(f"Detected ArUco markers: {marker_ids}")  # Логируем обнаруженные маркеры
        
    detect = await yolo.yolo_detect()
    if detect:
        logging.info(f"Detected Objects: {detect}")  # Логируем обнаруженные маркеры
        
    # Создаем переменую для внешних источников    
    attachments=Attachments(IP_ADDRESS,PORT_RELE,PORT_SERVO)
    await attachments.rele_status(True)
    await attachments.servo_grad(0)
    
    await controller.control_lights(True)  # Опционально: включаем свет на роботе
    await controller.navigate_to_marker(36)  # Навигация к маркеру с идентификатором 36
    await controller.control_lights(False)  # Опционально: выключаем свет на роботе

    await controller.navigate_to_marker(47)  # Навигация к маркеру с идентификатором 47
    await controller.perform_movement(100)  # Выполняем движение робота на 100 см
    await controller.perform_spin_angle(90) #выполняем поворот робота на 90 градусов
    await controller.control_lights(False)  # Опционально: включаем свет на роботе
    site.stop()
    yolo.stop()
    

if __name__ == "__main__":
    controller = None
    with suppress(contextlib):
        event.run_until_complete(main(use_simulator=True))
        event.close()
        thread_running.clear()


        
