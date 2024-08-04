# инициализаци библиотек
import asyncio
import json
import logging
import socket
import matplotlib.pyplot as plt
import numpy as np
import math
import asyncio
from plotly.utils import PlotlyJSONEncoder
import aiofiles
import gpsd
from geopy.distance import geodesic
import plotly.graph_objs as go

from Math import Math
# from Frenet_frame import Run_Robot

# класс представляет собой шаблон для контроллера робота
class RobotController:
    async def perform_movement(self, distance_cm):
        raise NotImplementedError

    async def perform_spin_angle(self, angle):
        raise NotImplementedError

    async def navigate_to_marker(self, goal_id):
        raise NotImplementedError

class SimulationController(RobotController, Math):
    """
    Класс для симуляции управления роботом с использованием графического отображения.

    Библиотеки:
    - asyncio: для асинхронного выполнения задач
    - math: для математических операций
    - heapq: для работы с очередями с приоритетом
    - numpy: для научных вычислений
    - matplotlib.pyplot: для графического отображения

    Методы:
    - __init__: Инициализация объекта SimulationController
    - build_graph: Построение графа смежности для маркеров
    - plot_map: Отображение карты маркеров на графике
    - update_robot_position: Обновление позиции робота на графике
    - plot_orientation_line: Отображение линии ориентации робота на графике
    - perform_movement: Выполнение движения робота на заданное расстояние
    - perform_spin_angle: Выполнение поворота робота на заданный угол
    - navigate_to_marker: Навигация к указанному маркеру
    - move_to_marker: Движение к указанному маркеру
    - find_closest_marker_id: Нахождение ближайшего маркера к текущей позиции робота
    - astar_algorithm: Реализация алгоритма A* для поиска пути
    - heuristic: Эвристическая функция для оценки расстояния между двумя маркерами
    - move_to_position: Движение к указанной позиции
    - show: Отображение графического интерфейса
    """

    def __init__(self, aruco_map, robot_state, ax,HOST_SOCKET='loclalhost',PORT_SOCKET=5530):
        """
        Конструктор класса.

        Параметры:
        - aruco_map (dict): Карта с координатами маркеров ArUco.
        - robot_state (ContactState): Состояние робота (координаты и ориентация).
        - ax (matplotlib.axes.Axes): Объект matplotlib для отображения графики.

        Описание:
        Инициализирует карту маркеров ArUco, состояние робота, графический объект matplotlib и строит граф смежности маркеров.
        Также отображает карту маркеров на графике и добавляет маркер для позиции робота и линию для отображения пути.
        """
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # создание сокета для сайта
        self.client_socket.connect((HOST_SOCKET, PORT_SOCKET)) # подключение к сайту
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536) # постановка буфера
        self.aruco_map = aruco_map  # Карта с координатами маркеров ArUco
        self.robot_state = robot_state  # Состояние робота (координаты и ориентация)
        self.ax = ax  # Объект matplotlib для отображения графики

        self.robot_marker=ax.add_trace(go.Scatter(x=[robot_state.x],y=[robot_state.y],marker=dict(
            size=20,  # Размер точек
            color='rgb(255, 0, 0)',  # Цвет точек
            symbol='triangle-up',  # Использование треугольных маркеров
            line=dict(
                width=2,  # Ширина обводки точек
                color='rgb(0, 0, 0)'  # Цвет обводки точек
            ))))# Маркер для позиции робота
        
        self.light_indicator = None  # Индикатор света
        self.lights_on = False  # Состояние света
        self.path_line = ax.add_trace(go.Scatter(x=[],y=[]))  # Линия для отображения пути
        self.graph = Math.build_graph(self=self)  # Построение графа на основе карты маркеров
        self.closestmarkers = Math.find_closest_marker_id(self=self)
        self.plot_map()  # Отображение карты маркеров на графике 

    def update_site(self):
        """
        Отображение графика на сайте
        
        Описание:
        Отправляет график на сайт в ввиде json строки
        """
        fig_json = json.dumps(self.ax, cls=PlotlyJSONEncoder)
        self.client_socket.sendall(f"{fig_json}".encode('utf-8'))
    
    def plot_map(self):
        """
        Отображение карты маркеров на графике.

        Описание:
        Рисует маркеры ArUco и их идентификаторы на графике. Также настраивает оси и добавляет сетку и легенду.
        """
        for marker_id, marker_info in self.aruco_map.items():
            self.ax.add_trace(go.Scatter(x=[marker_info['x']], y=[marker_info['y']]))
            self.ax.add_annotation(
                x=marker_info['x'],
                y=marker_info['y'],
                text=f'{marker_id}',
                showarrow=False,
                yshift=-10
            )

    def update_robot_position(self, x, y, orientation):
        """
        Обновление позиции робота на графике.

        Параметры:
        - x (float): Новая координата X робота.
        - y (float): Новая координата Y робота.
        - orientation (float): Новая ориентация робота.

        Описание:
        Обновляет координаты и ориентацию робота, а также перерисовывает маркер и линию пути на графике.
        """
        self.robot_state.update_position(x, y)
        self.robot_state.update_orientation(orientation)
        self.robot_marker.data[0].x=[x]
        self.robot_marker.data[0].y=[y]
        if self.lights_on:
            if self.light_indicator:
                self.light_indicator.layout.annotations = self.light_indicator.layout.annotations[:-1]
            self.light_indicator = self.ax.add_annotation(x=x, y=y, text="Lights On", 
                                    showarrow=True, arrowhead=1, arrowsize=2, arrowwidth=2,
                                    arrowcolor='yellow')
        self.update_site()
        plt.pause(0.5)  

    def plot_orientation_line(self, x, y, orientation):
        """
        Отображение линии ориентации робота на графике.

        Параметры:
        - x (float): Координата X начальной точки линии ориентации.
        - y (float): Координата Y начальной точки линии ориентации.
        - orientation (float): Ориентация робота.

        Описание:
        Рисует линию, показывающую направление ориентации робота, исходя из текущей позиции и ориентации.
        """
        length = 0.1  # Длина линии ориентации
        end_x = x + length * np.cos(np.radians(orientation))
        end_y = y + length * np.sin(np.radians(orientation))
        self.ax.plot([x, end_x], [y, end_y], 'r-', label='Orientation')  # Отрисовка линии ориентации

    async def perform_movement(self, distance_cm):
        """
        Выполнение движения робота на заданное расстояние.

        Параметры:
        - distance_cm (float): Расстояние для перемещения в сантиметрах.

        Описание:
        Вычисляет новые координаты робота на основе текущей позиции и ориентации, затем обновляет состояние робота
        и перерисовывает его на графике. Имитирует время движения с помощью asyncio.sleep.
        """
        distance_m = distance_cm / 100
        rad = np.radians(self.robot_state.orientation)
        new_x = self.robot_state.x + distance_m * np.cos(rad)
        new_y = self.robot_state.y + distance_m * np.sin(rad)
        x_values = [float(self.robot_marker.data[0].x[0]), new_x]
        y_values = [float(self.robot_marker.data[0].y[0]), new_y]
        self.robot_state.x, self.robot_state.y = new_x, new_y
        self.ax.add_trace(go.Scatter(
            x=x_values,
            y=y_values,
            mode='lines',
            line=dict(color='red')  # Указываем цвет линии
        ))
        self.robot_marker.data[0].x=[new_x]
        self.robot_marker.data[0].y=[new_y]
        if self.lights_on:
            if self.light_indicator:
                self.light_indicator.layout.annotations = self.light_indicator.layout.annotations[:-1]
            self.light_indicator = self.ax.add_annotation(x=new_x, y=new_y, text="Lights On", 
                                    showarrow=True, arrowhead=1, arrowsize=2, arrowwidth=2,
                                    arrowcolor='yellow')
            
        logging.info(f"Moving: {distance_cm}cm forward.")
        self.update_site()
        await asyncio.sleep(0.5)  # Simulate time it takes to move

    async def perform_spin_angle(self, angle):
        """
        Выполнение поворота робота на заданный угол.

        Параметры:
        - angle (float): Угол поворота в градусах.

        Описание:
        Вычисляет новую ориентацию робота и обновляет его состояние. Затем перерисовывает робота на графике.
        Имитация времени поворота осуществляется с помощью asyncio.sleep.
        """
        current_orientation = self.robot_state.orientation % 360
        new_orientation = (current_orientation + angle) % 360
        angle_difference = (new_orientation - current_orientation + 360) % 360
        if angle_difference > 180:
            angle_difference -= 360  # Наименьший угол поворота
        
        self.robot_state.orientation = new_orientation
        self.update_robot_position(self.robot_state.x, self.robot_state.y, new_orientation)
        logging.info(f"Spinning: {angle_difference} degrees {'clockwise' if angle_difference >= 0 else 'counterclockwise'}.")
        await asyncio.sleep(0.5)  # Имитация времени, необходимого для поворота
        
    async def control_lights(self, on: bool):
        """
        Асинхронная функция для включения/выключения света на роботе (визуализация).

        Параметры:
        - on (bool): Флаг включения/выключения света.

        Описание:
        Визуализирует включение или выключение света на графике путем изменения цвета или добавления аннотации.
        """
        self.lights_on = on
        if on:
            if self.light_indicator:
                self.light_indicator.layout.annotations = self.light_indicator.layout.annotations[:-1]
            self.light_indicator = self.ax.add_annotation(x=self.robot_marker.data[0].x, y=self.robot_marker.data[0].y, text="Lights On", 
                                    showarrow=True, arrowhead=1, arrowsize=2, arrowwidth=2,
                                    arrowcolor='yellow')
        else:
            if self.light_indicator:
                self.light_indicator.layout.annotations = self.light_indicator.layout.annotations[:-1]
        
        action = "включены" if on else "выключены"
        message = f"Свет {action}."
        self.update_site()
        logging.info(message)
    
    async def navigate_to_marker(self, goal_id):
        """
        Навигация к указанному маркеру.

        Параметры:
        - goal_id (int): Идентификатор целевого маркера.

        Описание:
        Использует алгоритм A* для нахождения пути к целевому маркеру и выполняет движение по этому пути.
        Если путь не найден, выводится сообщение об ошибке.
        """
        path = await self.astar_algorithm(self.find_closest_marker_id(), goal_id)
        if not path:
            logging.info("No path found.")
            return

        logging.info(f"Path found: {path}")
        for marker_id in path:
            await self.move_to_marker(marker_id)
            self.update_site()

    async def move_to_marker(self, marker_id):
        """
        Движение к указанному маркеру.

        Параметры:
        - marker_id (int): Идентификатор маркера для движения.

        Описание:
        Рассчитывает угол поворота и расстояние до маркера, выполняет поворот и движение к маркеру.
        Обновляет текущее состояние робота после выполнения движения.
        """
        target = self.aruco_map[marker_id]
        dx = target['x'] - self.robot_state.x
        dy = target['y'] - self.robot_state.y
        target_angle = math.degrees(math.atan2(dy, dx))
        await self.perform_spin_angle(target_angle - self.robot_state.orientation)
        distance = math.sqrt(dx ** 2 + dy ** 2) * 100
        await self.perform_movement(distance)

    def move_to_position(self, x, y):
        """
        Движение к указанной позиции.

        Параметры:
        - x (float): Координата X целевой позиции.
        - y (float): Координата Y целевой позиции.

        Описание:
        Рассчитывает угол поворота и расстояние до целевой позиции, затем выполняет поворот и движение.
        """
        dx = x - self.robot_state.x
        dy = y = self.robot_state.y
        target_heading = math.degrees(math.atan2(dy, dx))
        distance = math.sqrt(dx ** 2 + dy ** 2) * 100

        self.perform_spin_angle((target_heading - self.robot_state.orientation) % 360 - 180, clockwise=True)
        self.perform_movement(distance)

    def show(self):
        """
        Отображение графического интерфейса.

        Описание:
        Вызывает метод show библиотеки matplotlib для отображения графического интерфейса.
        """
        plt.show()

class RoverController(Math):
    """
    Класс для управления роботом через сервер с использованием Socket.IO.

    Библиотеки:
    - asyncio: для асинхронного выполнения задач
    - aiofiles: для асинхронного чтения и записи файлов
    - math: для математических операций
    - heapq: для работы с очередями с приоритетом
    - logging: для ведения логов
    - socketio: для работы с Socket.IO клиентом
    """

    def __init__(self, aruco_map, robot_state, server_url, token):
        """
        Конструктор класса. Инициализирует карту маркеров ArUco, состояние робота, URL сервера и токен авторизации.

        Параметры:
        - aruco_map (dict): Карта маркеров ArUco с их координатами.
        - robot_state (ContactState): Текущее состояние робота (координаты и ориентация).
        - server_url (str): URL сервера для подключения.
        - token (str): Токен авторизации для подключения к серверу.
        """
        self.aruco_map = aruco_map  # Карта с координатами маркеров ArUco
        self.robot_state = robot_state  # Состояние робота (координаты и ориентация)
        self.light_indicator = None  # Индикатор света
        self.lights_on = False  # Состояние света
        self.graph = Math.build_graph(self=self)  # Построение графа на основе карты маркеров
        self.closestmarkers = Math.find_closest_marker_id(self=self)
        self.plot_map()  # Отображение карты маркеров на графике 


    async def initialize_socketio(self):
        """
        Асинхронная функция для инициализации и подключения клиента Socket.IO к серверу.

        Описание:
        Подключается к серверу с использованием URL и токена авторизации. Если подключение успешно, 
        выводится сообщение об успешном подключении. В случае ошибки подключения, выводится сообщение об ошибке.
        """
        # try:
        await self.sio.connect(self.server_url, auth={"token": self.token}, namespaces=['/vehicles'])
        logging.info("Connected to Socket.IO server")
        # except socketio.exceptions.ConnectionError as e:
        #     logging.error(f"Failed to connect to server: {e}")
            
    def call_function0(self):
        self.graph

    async def write_report(message):
        """
        Асинхронная функция для записи сообщений в файл отчета.

        Параметры:
        - message (str): Сообщение для записи в отчет.

        Описание:
        Создает файл отчета и записывает в него сообщения, используемые для ведения журнала действий робота.
        """
        filename = f'Отчет_Фамилия_Имя.txt'
        async with aiofiles.open(filename, mode='a', encoding='utf-8') as file:
            await file.write(message + "\n")

    async def on_move(self, result):
        """
        Колбэк для обработки ответов сервера на команды движения.

        Параметры:
        - result (dict): Результат выполнения команды движения.

        Описание:
        Записывает результаты выполнения команд движения в отчет.
        """
        await RoverController.write_report(f'Command done: {result}')
        
    async def stop_movement(self):
        """
        Асинхронная функция для отправки команды остановки робота через Socket.IO.

        Описание:
        Отправляет команду на сервер для остановки всех движений робота.
        Записывает действие в отчет и лог.
        """
        await self.sio.emit('command', {
            'id': 1,
            'type': "move",
            'value': {
                'x': 0,
                'y': 0,
                'sensitivity': 1,
            },
        }, namespace='/vehicles', callback=self.on_move)

        stop_message = "Команда остановки движения отправлена."
        await RoverController.write_report(stop_message)
        logging.info(stop_message)

        await asyncio.sleep(1)  # Дополнительное ожидание для обеспечения полной остановки

        completion_message = "Робот остановлен."
        await RoverController.write_report(completion_message)
        logging.info(completion_message)

    async def go(self,x,y):
        await self.sio.emit('command', {
            'id': 1,
            'type': "move",
            'value': {
                'x': x,
                'y': y,
                'sensitivity': 1,
            },
        }, namespace='/vehicles', callback=self.on_move)
    
    async def perform_movement(self, distance_cm):
        """
        Асинхронная функция для отправки команды движения роботу через Socket.IO.
        
        НЕОБХОДИМА НАСТРОЙКА КОЭФИЦИЕНТОВ max_speed для коректного перемещения на заданнные расстояния
        
        Параметры:
        - distance_cm (float): Расстояние для перемещения в сантиметрах.

        Описание:
        Вычисляет время, необходимое для перемещения робота на заданное расстояние, и отправляет команду на сервер.
        Ожидает завершения движения и записывает действие в отчет.
        """
        max_speed = 0.5  # Максимальная скорость м/с (примерная, можно изменить)
        distance_m = distance_cm / 100.0  # Переводим сантиметры в метры
        time_to_travel = distance_m / max_speed

        await self.sio.emit('command', {
            'id': 1,
            'type': "move",
            'value': {  
                'x': 0,
                'y': 1,
                'sensitivity': 1,
            },
        }, namespace='/vehicles', callback=self.on_move)

        movement_message = f"Команда движения отправлена, ожидайте {time_to_travel:.2f} секунд."
        await RoverController.write_report(movement_message)
        logging.info(movement_message)
        await asyncio.sleep(time_to_travel)

        await self.sio.emit('command', {
            'id': 1,
            'type': "move",
            'value': {
                'x': 0,
                'y': 0,
                'sensitivity': 1,
            },
        }, namespace='/vehicles', callback=self.on_move)
        
        await asyncio.sleep(1)

        completion_message = f"Движение завершено: Пройденное расстояние {distance_m} м."
        await RoverController.write_report(completion_message)
        logging.info(completion_message)

    async def on_spin(self, result):
        """
        Колбэк для обработки ответов сервера на команды поворота.

        Параметры:
        - result (dict): Результат выполнения команды поворота.

        Описание:
        Записывает результаты выполнения команд поворота в отчет.
        """
        await RoverController.write_report(f'Spin command done: {result}')

    async def perform_spin_angle(self, angle):
        """
        Асинхронная функция для отправки команды поворота роботу.

        НЕОБХОДИМА НАСТРОЙКА КОЭФИЦИЕНТОВ FULL_ROTATION_TIME и ACCELERATION_FACTOR для коректного поворота на заданный угол


        Параметры:
        - angle (float): Угол поворота в градусах.

        Описание:
        Вычисляет время, необходимое для выполнения поворота, и отправляет команду на сервер. 
        Ожидает завершения поворота и записывает действие в отчет.
        """
        FULL_ROTATION_TIME = 2  # Время для полного оборота (360 градусов) в секундах
        ACCELERATION_FACTOR = 0.05  # Фактор ускорения/замедления
        rotation_time = (angle / 360) * FULL_ROTATION_TIME
        total_time = rotation_time + (rotation_time * ACCELERATION_FACTOR)
        clockwise = angle < 0

        await self.sio.emit('command', {
            'id': 1,
            'type': "spin",
            'value': {
                'state': True,
                'direction': clockwise
            },
        }, namespace='/vehicles', callback=self.on_spin)
        
        spin_message = f"Начат поворот на {angle} градусов, {'по часовой стрелке' if clockwise else 'против часовой стрелки'}"
        await RoverController.write_report(spin_message)
        logging.info(spin_message)
        await asyncio.sleep(total_time)
        
        await self.sio.emit('command', {
            'id': 1,
            'type': "spin",
            'value': {
                'state': False,
                'direction': clockwise
            },
        }, namespace='/vehicles', callback=self.on_spin)
        await asyncio.sleep(0.5)

        completion_spin_message = f"Поворот на {angle} градусов завершён."
        await RoverController.write_report(completion_spin_message)
        logging.info(completion_spin_message)

    async def on_lights(self, result):
        """
        Колбэк для обработки ответов сервера на команды управления светом.

        Параметры:
        - result (dict): Результат выполнения команды управления светом.

        Описание:
        Записывает результаты выполнения команд управления светом в отчет.
        """
        await RoverController.write_report(f'Lights command done: {result}')

    async def control_lights(self, on: bool):
        """
        Асинхронная функция для управления освещением робота.

        Параметры:
        - on (bool): Флаг включения/выключения света.

        Описание:
        Отправляет команду управления освещением на сервер и записывает действие в отчет.
        """
        lights_command = {
            'id': 1,
            'type': 'lights',
            'value': on
        }

        await self.sio.emit('command', lights_command, namespace='/vehicles', callback=self.on_lights)

        action = "включены" if on else "выключены"
        message = f"Команда на включение света {action} отправлена."
        await RoverController.write_report(message)
        logging.info(message)

    # async def move_to_gps(self,gpsd):
    #     """
    #     Асинхронная функция для перемещения ровера
    #     """
    #     self.target_location=gpsd
    #     # gps=GPS()
    #     a=True
    #     # Основной цикл
    #     try:
    #         current_location = gps.get_current_location()
    #         dist = gps.distance_to_target(current_location, self.target_location)
    #         x,y=current_location
    #         gx,gy=self.target_location
    #         dx = gx-x
    #         dy = gy-y
    #         target_angle = math.degrees(math.atan2(dy, dx))
    #         logging.debug(f"Target coordinates (x: {gx}, y: {gy}). Current position (x: {x}, y: {y})")
    #         logging.debug(f"Calculated target angle: {target_angle} degrees")

    #         angle_to_turn = (target_angle - self.robot_state.orientation + 360) % 360
    #         if angle_to_turn > 180:
    #             angle_to_turn -= 360  # Оптимизация угла поворота
    #         logging.debug(f"Calculated turn angle: {angle_to_turn} degrees")

    #         await self.perform_spin_angle(angle_to_turn)
    #         distance = math.sqrt(dx ** 2 + dy ** 2) * 100  # Расстояние в сантиметрах
    #         logging.debug(f"Calculated distance to move: {distance}cm")

    #         x=self.robot_state.x
    #         y=self.robot_state.y
            
    #         # Обновляем положение робота
    #         self.robot_state.x += distance * np.cos(np.radians(target_angle)) / 100
    #         self.robot_state.y += distance * np.sin(np.radians(target_angle)) / 100
    #         self.robot_state.orientation = target_angle
    #         logging.debug(f"Updated robot state to (x: {self.robot_state.x}, y: {self.robot_state.y}, orientation: {self.robot_state.orientation})")

    #         # self.robot_state.orientation=Robot_run(self).run_gps(gps,dx,dy,target_angle)
    #         Run_Robot(x,y,gx,gy,self)
    #     except KeyboardInterrupt:
    #         print("Программа остановлена пользователем.")

    async def move_to_marker(self, marker_id):
        """
        Асинхронная функция для выполнения движения к указанному маркеру.

        Параметры:
        - marker_id (int): Идентификатор маркера для движения.

        Описание:
        Рассчитывает угол поворота и расстояние до маркера, выполняет поворот и движение к маркеру.
        Обновляет текущее состояние робота после выполнения движения.
        """
        logging.debug(f"Starting to move to marker {marker_id}")
        target = self.aruco_map[marker_id]
        dx = target['x'] - self.robot_state.x
        dy = target['y'] - self.robot_state.y
        target_angle = math.degrees(math.atan2(dy, dx))
        logging.debug(f"Target coordinates (x: {target['x']}, y: {target['y']}). Current position (x: {self.robot_state.x}, y: {self.robot_state.y})")
        logging.debug(f"Calculated target angle: {target_angle} degrees")

        angle_to_turn = (target_angle - self.robot_state.orientation + 360) % 360
        if angle_to_turn > 180:
            angle_to_turn -= 360  # Оптимизация угла поворота
        logging.debug(f"Calculated turn angle: {angle_to_turn} degrees")

        await self.perform_spin_angle(angle_to_turn)
        distance = math.sqrt(dx ** 2 + dy ** 2) * 100  # Расстояние в сантиметрах
        logging.debug(f"Calculated distance to move: {distance}cm")

        x=self.robot_state.x
        y=self.robot_state.y
        
        # Обновляем положение робота
        self.robot_state.x += distance * np.cos(np.radians(target_angle)) / 100
        self.robot_state.y += distance * np.sin(np.radians(target_angle)) / 100
        self.robot_state.orientation = target_angle
        logging.debug(f"Updated robot state to (x: {self.robot_state.x}, y: {self.robot_state.y}, orientation: {self.robot_state.orientation})")

        # self.robot_state.orientation=Robot_run(self).run(x,y,dx,dy,target_angle)
        Run_Robot(x,y,target['x'],target['y'],self)
        logging.info(f"Moved to marker {marker_id}: rotate to {target_angle}° and move {distance}cm.")

# class gps:
        
#     def __init__(self):
#         self.gpsd=gpsd.connect()
    
#     def get_current_location():
#         packet = gpsd.get_current()
#         if packet.mode >= 2:  # 2D фиксация или лучше
#             lat = packet.lat
#             lon = packet.lon
#             return (lat, lon)
#         else:
#             raise Exception("GPS-сигнал не найден")
    
#     def distance_to_target(current_location, target_location):
#         return geodesic(current_location, target_location).meters    
#         # Допишите логику движения (изменение направления робота/транспортного средства)
        

# def call_function2(self):
#     self.closestmarkers

# def call_function3(self):
#     self.astar

# def call_function4(self):
#     self.heuristick
