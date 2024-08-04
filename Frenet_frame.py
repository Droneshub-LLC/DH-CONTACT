import asyncio
import contextlib
import json
import socket
import threading
import tracemalloc
from matplotlib import markers
from matplotlib.transforms import Affine2D
import numpy as np
import copy
import math
import sys
import pathlib
from video_processing import Realsense
sys.path.append(str(pathlib.Path(__file__).parent.parent))
import plotly.graph_objs as go
from plotly.utils import PlotlyJSONEncoder
import tornado.ioloop
import tornado.web
import tornado.websocket
import json

from quintic_polynomials_planner import \
    QuinticPolynomial
import cubic_spline_planner

transform = Affine2D().rotate_deg(90)

# Создаем маркер в форме треугольника
triangle = markers.MarkerStyle('v')

# клиент сервера реалсенса
try:
    sd=Realsense('localhost',5527)
except Exception as e:
    print(e)
    
    
# фигура для отправки на сайт
global ax
ax = go.Figure()  
ax.update_layout(showlegend=False)

# сервер сайта
host = 'localhost'
port = 5530

# Инициализация клиентского сокета
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

# Определение обработчика запросов к главной странице
def create_plotly_figure():
    global ax,f
    # Создаем фигуру и добавляем график
    while True:
        fig_json = json.dumps(ax, cls=PlotlyJSONEncoder)
        client_socket.sendall(f"{fig_json}".encode('utf-8'))
tracemalloc.start()    
    
SIM_LOOP = 5000

MAX_SPEED = 50.0 / 3.6 # максимальная скорость [м/с]
MAX_ACCEL = 0.0 # максимальное ускорение [м/с]
MAX_CURVATURE = 1000.0 # максимальная кривизна [1/м]
MAX_ROAD_WIDTH = 7.0 # максимальная ширина дороги [м]
D_ROAD_W = 1.0 # длина выборки ширины дороги [м]
DT = 0.5# временной интервал [с]
MAX_T = 8.0 # максимальное время прогнозирования [м]
MIN_T = 2.0 # минимальное время прогнозирования [м]
TARGET_SPEED  = 10.0 / 3.6 # скорость цели [м/с]
D_T_S = 10.0 / 3.6 # длина выборки целевой скорости [м/с]
N_S_SAMPLE = 1 # количество выборок целевой скорости
ROBOT_RADIUS = 0.8 # радиус робота [м]

# cost weights
K_J     = 0.1
K_T     = 0.0
K_D     = 1.0
K_LAT   = 1.0
K_LON   = 1.0

show_animation = True

class QuarticPolynomial:
    """
    Класс для представления квартического полинома.

    Используется для планирования одномерного движения с заданными начальными и конечными условиями.

    Атрибуты:
        a0 (float): Свободный член полинома, начальное положение.
        a1 (float): Коэффициент при первой степени t, начальная скорость.
        a2 (float): Коэффициент при второй степени t, половина начального ускорения.
        a3 (float): Коэффициент при третьей степени t.
        a4 (float): Коэффициент при четвертой степени t.
    """
    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:
    """
    Класс для представления пути в координатах Френе.

    Атрибуты:
        t (list of float): Время, соответствующее каждой точке пути.
        d (list of float): Поперечное смещение от базовой линии.
        d_d (list of float): Первая производная d, поперечная скорость.
        d_dd (list of float): Вторая производная d, поперечное ускорение.
        d_ddd (list of float): Третья производная d, поперечный рывок.
        s (list of float): Продольное расстояние вдоль базовой линии.
        s_d (list of float): Продольная скорость.
        s_dd (list of float): Продольное ускорение.
        s_ddd (list of float): Продольный рывок.
        cd (float): Стоимость изменения поперечного смещения.
        cv (float): Стоимость изменения скорости.
        cf (float): Общая стоимость функции.

        x (list of float): Глобальные координаты X пути.
        y (list of float): Глобальные координаты Y пути.
        yaw (list of float): Углы направления (yaw) для каждой точки пути.
        ds (list of float): Изменение расстояния между последовательными точками.
        c (list of float): Кривизна пути.
    """
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    """
    Преобразование путей из координат Френе в глобальные координаты.

    Параметры:
    - fplist (list of FrenetPath objects): Список объектов путей в системе координат Френе.
    - csp (CubicSpline2D object): Объект кубического сплайна для пути.

    Возвращает:
    - (list of FrenetPath objects): Список путей с преобразованными глобальными координатами.

    Описание:
    Для каждого пути в списке вычисляются глобальные позиции, углы направления (yaw), изменения расстояния (ds) и кривизна.
    """
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    """
    Проверка столкновения траектории с препятствиями.

    Параметры:
    - fp (FrenetPath object): Объект пути в системе координат Френе.
    - ob (numpy.ndarray): Массив препятствий, где каждое препятствие представлено координатами (x, y).

    Возвращает:
    - (bool): Возвращает False, если обнаружено столкновение, иначе True.

    Описание:
    Функция итерируется по каждому препятствию и вычисляет расстояние от каждой точки траектории до препятствия.
    Если расстояние меньше или равно квадрату радиуса робота, считается, что столкновение произошло.
    """
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    """
    Проверка путей на соответствие ограничениям и отсутствие столкновений.

    Параметры:
    - fplist (list of FrenetPath objects): Список объектов путей в системе координат Френе.
    - ob (list of tuples): Список препятствий в формате (x, y).

    Возвращает:
    - (list of FrenetPath objects): Список допустимых путей, прошедших проверку.

    Описание:
    Функция итерируется по каждому пути в списке и проверяет его на соответствие максимальной скорости и кривизне.
    Также проводится проверка на столкновения с препятствиями.
    Пути, прошедшие все проверки, добавляются в список допустимых.
    """
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob):
    """
    Планирование оптимального пути в системе координат Френе.

    Параметры:
    - csp (CubicSpline2D object): Объект кубического сплайна, представляющий путь.
    - s0 (float): Начальное значение длины дуги на сплайне.
    - c_speed (float): Текущая скорость.
    - c_accel (float): Текущее ускорение.
    - c_d (float): Текущее боковое смещение.
    - c_d_d (float): Текущая производная бокового смещения.
    - c_d_dd (float): Текущая вторая производная бокового смещения.
    - ob (list of tuples): Список препятствий в формате (x, y).

    Возвращает:
    - best_path (FrenetPath object): Оптимальный путь в системе координат Френе.

    Описание:
    Функция вычисляет возможные пути в системе координат Френе, преобразует их в глобальные координаты и проверяет на столкновения с препятствиями.
    Затем выбирается путь с минимальной стоимостью, основываясь на различных факторах, таких как безопасность, комфорт и эффективность.
    """
    fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    """
    Генерация целевого курса с использованием кубического сплайна.

    Параметры:
    - x (list): Список X-координат.
    - y (list): Список Y-координат.

    Возвращает:
    - rx (list): Список X-координат пути.
    - ry (list): Список Y-координат пути.
    - ryaw (list): Список углов направления (yaw) пути.
    - rk (list): Список значений кривизны пути.
    - csp (CubicSpline2D object): Объект кубического сплайна.

    Описание:
    Функция инициализирует объект кубического сплайна `CubicSpline2D` с заданными координатами.
    Затем она вычисляет путь, угол направления и кривизну в регулярных интервалах вдоль сплайна.
    Это делается путём итерации по значениям параметра `s`, который представляет собой длину дуги сплайна.
    """
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def calculate_relative_angle(x, center_x=360, max_angle=50):
    # Преобразование диапазона x (0-640) в угол (-60 до +60 градусов)
    relative_angle = (x - center_x)/(center_x/max_angle)
    # Ограничение угла пределами -60 до +60 градусов
    if relative_angle < -max_angle:
        relative_angle = -max_angle
    elif relative_angle > max_angle:
        relative_angle = max_angle
    return -relative_angle

def mass(xs,ys,tar):
    """
    Обработка данных о массе и расчёт новых координат.

    Параметры:
    - xs (float): X-координата начальной точки.
    - ys (float): Y-координата начальной точки.
    - tar (float): Целевой угол в градусах.

    Возвращает:
    - new_points (numpy.ndarray): Массив новых координат точек.

    Описание:
    Функция пытается получить данные с помощью метода `give_mass`. В случае ошибки
    используется заранее определённый массив данных. Затем функция ищет точки,
    у которых расстояние до некоторой цели приблизительно равно 0.5 (с допуском 0.2).
    Для каждой найденной точки рассчитывается новая координата на основе заданного
    целевого угла `tar` и расстояния, которое также задано в функции.
    """
    try:
        data=sd.give_mass()
    except Exception as e: 
        print("error -",e)
        data = np.array([
            [[100, 2, 0.3], [200, 3, 0.3], [300, 4, 0.5], [400, 5, 0.5], [500, 6, 0.6], [600, 7, 0.5]],
            [[100, 2, 0.3], [200, 3, 0.3], [300, 4, 0.5], [400, 5, 0.5], [500, 6, 0.6], [600, 7, 0.4]],
            [[100, 2, 0.3], [200, 3, 0.3], [300, 4, 0.5], [400, 5, 0.5], [500, 6, 0.6], [600, 7, 0.6]],
            [[100, 2, 0.3], [200, 3, 0.3], [300, 4, 0.5], [400, 5, 0.5], [500, 6, 0.6], [600, 7, 0.5]]
        ])

    # Расстояние между точками
    distance = 2

    # Находим точки с dist ~ 0.5
    points = []
    try:
        for row in data:
            x,y,dist = row   
            if y==175:   
                if np.isclose(dist, 0.5, atol=0.2):
                    points.append((x, y))
    except:
        for row in data:
            for x,y,dist in row:
                if y==2:   
                    if np.isclose(dist, 0.6, atol=0.2):
                        points.append((x, y))

    # Функция для вычисления угла из координаты x
    def calculate_angle(x):
        # Пример функции, которая вычисляет угол в радианах
        # Можно заменить на свою функцию
        return np.deg2rad(x)  # Пример: угол = x * 10 градусов
    new_points = np.empty((0, 2))

# Располагаем точки на координатной оси
    for x, y in points:
        angle_radians = np.radians(tar)
        angle = np.radians(calculate_relative_angle(x))  # Вычисление относительного угла
        new_x = xs + distance * np.cos(angle_radians + angle)
        new_y = ys + distance * np.sin(angle_radians + angle)
        new = np.array([[new_x, new_y]])  # Обратите внимание на двойные скобки

        # Добавляем new в new_points
        new_points = np.append(new_points, new, axis=0)
    return new_points
        
def degres_to_mark(x,y,dxs,dys):
    """
    Вычисление угла между начальной и конечной точками.

    Параметры:
    - x (float): X-координата начальной точки.
    - y (float): Y-координата начальной точки.
    - dxs (float): X-координата конечной точки.
    - dys (float): Y-координата конечной точки.

    Возвращает:
    - target_angle (float): Угол между двумя точками в градусах.

    Описание:
    Функция вычисляет разницу координат между начальной и конечной точками.
    Используя функцию `atan2` из модуля `math`, функция вычисляет арктангенс отношения
    разницы Y-координат к разнице X-координат, что дает угол в радианах.
    Затем этот угол преобразуется в градусы с помощью функции `degrees`.
    """
    dx = dxs - x
    dy = dys - y
    target_angle = math.degrees(math.atan2(dy, dx))
    return target_angle

def main(x,y,dx,dy,con=None):
    """
    Планирование и визуализация траектории движения объекта.

    Параметры:
    - x (float): Начальная координата X объекта.
    - y (float): Начальная координата Y объекта.
    - dx (float): Конечная координата X объекта.
    - dy (float): Конечная координата Y объекта.
    - con (object): Объект управления, который может быть использован для отправки команд управления.

    Описание:
    Функция `main` инициализирует начальное состояние объекта и планирует траекторию от начальной до конечной точки.
    Она учитывает статические препятствия и использует метод оптимального планирования Френе для генерации пути.
    В процессе планирования функция может адаптировать траекторию, если обнаруживаются новые препятствия.
    Результаты планирования визуализируются с помощью библиотеки Plotly, если установлен флаг `show_animation`.
    Функция также поддерживает отправку команд управления через объект `con`, если он предоставлен.

    Возвращает:
    - None
    """
    global ax
    print(__file__ + " start!!")

    # way points
    wx = [x, dx]
    wy = [y, dy]
    # obstacle lists
    ob = np.array([
                   [0.0, 5.0]
                   ])

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 2.0 / 3.6 # текущая скорость [м/с]
    c_accel = 0.0  # текущее ускорение [м/с]
    c_d     = 0.0  # текущее боковое положение [м]
    c_d_d   = 0.0  # текущая боковая скорость [м/с]
    c_d_dd  = 0.0  # текущее боковое ускорение [м/с]
    s0      = 0.0  # текущее положение по курсу
    target_angle = degres_to_mark(x,y,dx,dy)
    triangle._transform = triangle.get_transform().rotate_deg(90)
    triangle._transform = triangle.get_transform().rotate_deg(target_angle)
    area = 10.0  # animation area length [m]

    for i in range(SIM_LOOP):
        
        if i>0:
            ob=np.array([[0.0,-50.0]])
            ob = np.append(ob, mass(0, 0,90), axis=0)
            # ob = np.append(ob, mass(path.x[1], path.y[1],target_angle), axis=0)

        path = frenet_optimal_planning(
            csp, 0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        s0 = 0
        c_d = 0.00000000000000001
        c_d_d = 0.0000000000000000001
        c_d_dd = 0.0000000000000000001
        c_speed = 0.00000000000000000000000000001
        c_accel = 0
        angle=degres_to_mark(path.x[0],path.y[0],path.x[1],path.y[1])-target_angle
        triangle._transform = triangle.get_transform().rotate_deg(angle)
        target_angle = degres_to_mark(path.x[0],path.y[0],path.x[1],path.y[1])
        print(c_d_d,c_speed,target_angle)
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break
        
        if show_animation:  # pragma: no cover
            ax.data=[]
            ax.add_trace(go.Scatter(
                x=tx,
                y=ty,
                mode='lines+markers',
                name='Траектория'
            ))
            ax.add_trace(go.Scatter(
                x=ob[:, 0],
                y=ob[:, 1],
                mode='markers',
                name='Препятствия',
                marker=dict(color='black')
            ))
            ax.add_trace(go.Scatter(
                x=path.x[1:],
                y=path.y[1:],
                mode='lines+markers',
                name='Путь',
                marker=dict(color='red')
            ))
            # Для треугольника используйте 'triangle-up' и задайте размер
            ax.add_trace(go.Scatter(
                x=[path.x[1]],
                y=[path.y[1]],
                mode='markers',
                name='Начальная точка',
                marker=dict(symbol='triangle-up', size=15)
            ))

            ax.layout=go.Layout(
                title=f"v[km/h]:{str(c_speed * 3.6)[:4]}",
                xaxis=dict(range=[path.x[1]-area, path.x[1]+area]),
                yaxis=dict(range=[path.y[1]-area, path.y[1]+area]),
                showlegend=False,
                height=800
            )
        
        else:
            con.go(c_d_dd,c_accel)
            
    print("Finish")
    if show_animation: 
        tornado.ioloop.IOLoop.current().stop()

class Run_Robot():
    """
    Класс запуска робота без сайта

    Описание:
    Класс предназначен для запуска кода без сайта для езды по местности
    """
    def __init__(self,x,y,dx,dy,con):
        self.x=x
        self.y=y
        self.dx=dx
        self.dy=dy
        self.con=con
        
    def going_to_coor_aruco(self):
        main(self.x,self.y,self.dx,self.dy,self.con)  
        

async def mains():
    """
    запуск основного процесса и потока отправки данных на сайт

    Библиотеки:
    - threading: для создания отедльного потока

    Описание:
    Запускает основной код и так же появляется отедльный поток с бесконечным циклом на тправку данных на сайт
    """
    socket_thread = threading.Thread(target=create_plotly_figure, daemon=True)
    socket_thread.start()
    with contextlib.suppress(KeyboardInterrupt):
        # Запуск сервера Tornado
        main(0,0,0,70)
        socket_thread.join()


if __name__ == '__main__':
    asyncio.run(mains())
