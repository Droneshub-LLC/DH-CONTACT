# инициализация библиотек
import math
import pyrealsense2 as rs
import numpy as np
import cv2

# инициализация камеры Intel Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Настройка потоков глубины и цвета 

def time_to_rot(angle):
    """
    Функция с расчетами времени поворота

    Параметры: 
    - angle: угол поворота в градусах, для которого необходимо рассчитать время

    Описание:
    Функция вычисляет время, необходимое для поворота объекта на угол, указанный в параметре 'angle'.
    Время поворота пропорционально углу поворота и рассчитывается на основе времени, требуемого для полного оборота.
    К результату добавляется дополнительное время, учитывающее фактор ускорения/замедления.

    Возвращает:
    total_time - общее время в секундах, необходимое для поворота на заданный угол
    """
    FULL_ROTATION_TIME = 2  # Время для полного оборота (360 градусов) в секундах
    ACCELERATION_FACTOR = 0.05  # Фактор ускорения/замедления
    rotation_time = (angle / 360) * FULL_ROTATION_TIME
    total_time = rotation_time + (rotation_time * ACCELERATION_FACTOR)
    return total_time

class Robot_run:
    
    """
    Класс для подъезда к конечной точке с объездом препятствий 

    Методы:
    - __init__: Инициализация объекта Robot_run
    - degres: Поворот робота на определённый градус
    - movment: движение на определённое расстояние (в см) и запись n-ых координат
    - degres_to_mark: поворот к конечной точке
    - distance: расчёт расстояния до конечной точки
    - run: движение до конечной аруко метки
    """
    
    def __init__(self,controller):
        """
        Конструктор класса.

        Параметры:
        controller - контроллер, который управляет роботом и содержит методы для взаимодействия с ним

        Описание:
        Этот метод инициализирует объект класса, устанавливая начальные значения для атрибутов объекта.
        'controller' передается как параметр и сохраняется в атрибуте 'rc', который затем может быть использован
        другими методами класса для управления роботом.
        """
        self.rc=controller
       
    def degres(self,degres):
        """
        Вычисляет оптимальный угол для поворота робота и выполняет поворот.

        Параметры:
        degres - целевой угол в градусах, до которого должен повернуться робот

        Описание:
        Метод вычисляет угол, на который необходимо повернуть робота, чтобы достичь заданного целевого угла 'degres'.
        Расчет угла поворота учитывает текущее положение робота и оптимизирует направление поворота так,
        чтобы было выполнено минимальное движение.

        Выполнение:
        Если расчетный угол поворота больше 180 градусов, метод корректирует его, чтобы поворот был выполнен
        в противоположном направлении, что является более эффективным.
        Затем метод вызывает функцию 'perform_spin_angle' объекта 'rc', чтобы выполнить поворот на расчетный угол.
        После выполнения поворота, метод обновляет 'target_angle' до нового целевого угла 'degres'.
        """
        angle_to_turn = (degres - self.target_angle + 360) % 360
        if angle_to_turn > 180:
            angle_to_turn -= 360  # Оптимизация угла поворота
        self.rc.perform_spin_angle(angle_to_turn)
        self.target_angle=degres
        
    def movment(self,distance):
        """
        Выполняет перемещение робота на заданное расстояние и обновляет его координаты.

        Параметры:
        distance - расстояние, на которое робот должен переместиться

        Описание:
        Метод вызывает функцию 'perform_movement' объекта 'rc', чтобы переместить робота на заданное расстояние.
        Затем метод обновляет координаты 'x' и 'y' робота, учитывая текущий целевой угол поворота и пройденное расстояние.
        Расчет новых координат включает преобразование угла из градусов в радианы и использование тригонометрических функций.

        Выполнение:
        После вызова 'perform_movement', метод вычисляет изменение координат 'x' и 'y' на основе угла 'target_angle'.
        Это делается с помощью функций np.cos и np.sin из библиотеки NumPy, которые принимают угол в радианах.
        Результат умножается на 'distance' и делится на 100 для преобразования в соответствующие единицы измерения.
        """
        self.rc.perform_movement(distance)
        self.x += distance * np.cos(np.radians(self.target_angle)) / 100
        self.y += distance * np.sin(np.radians(self.target_angle)) / 100
    
    def degres_to_mark(self):
        """
        Вычисляет угол от текущего положения робота до целевой точки.

        Описание:
        Метод использует координаты текущего положения робота (self.x, self.y) и координаты целевой точки (self.x_g, self.y_g),
        чтобы вычислить разницу по осям x и y (dx и dy соответственно).
        Затем он применяет функцию atan2 из модуля math, чтобы найти арктангенс отношения dy к dx, что дает угол в радианах.
        Этот угол преобразуется в градусы с помощью функции degrees того же модуля.

        Возвращаемое значение:
        Метод возвращает вычисленный угол в градусах, который может быть использован для направления робота к целевой точке.
        """
        dx = self.x_g - self.x
        dy = self.y_g - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        return target_angle
    
    def distance(self):
        """
        Вычисляет евклидово расстояние от текущего положения робота до целевой точки.

        Описание:
        Метод вычисляет разницу по осям x и y между текущим положением робота (self.x, self.y) и координатами целевой точки (self.x_g, self.y_g).
        Используя эти значения (dx и dy), метод применяет теорему Пифагора для вычисления расстояния: квадратный корень из суммы квадратов dx и dy.
        Полученное расстояние умножается на 100, возможно, для преобразования в другую систему единиц измерения.

        Возвращаемое значение:
        Метод возвращает расстояние, умноженное на 100, что может быть необходимо для соответствия единицам измерения в других частях программы.
        """
        dx = self.x_g - self.x
        dy = self.y_g - self.y
        return math.sqrt(dx ** 2 + dy ** 2) * 100 
    
    def calculate_bearing(self):
        """
        Вычисляет азимут от начальной точки до конечной точки.

        Описание:
        Метод начинается с преобразования географических координат начальной (self.x, self.y) и конечной (self.x_g, self.y_g) точек из градусов в радианы.
        Это необходимо для использования математических функций, которые работают с радианами.

        Затем вычисляется разница долгот между начальной и конечной точками. Используя эту разницу, а также широты начальной и конечной точек, метод вычисляет компоненты x и y для определения азимута.

        После этого азимут переводится из радиан в градусы с помощью функции atan2 и degrees. Результат корректируется так, чтобы азимут всегда был положительным числом от 0 до 360 градусов.

        Возвращаемое значение:
        Метод возвращает вычисленный азимут в градусах.
        """

    # Переводим координаты в радианы
        start_lat = math.radians(self.x)
        start_lon = math.radians(self.y)
        end_lat = math.radians(self.x_g)
        end_lon = math.radians(self.y_g)

        # Вычисляем разницу долгот
        delta_lon = end_lon - start_lon

        # Вычисляем угол поворота
        x = math.sin(delta_lon) * math.cos(end_lat)
        y = math.cos(start_lat) * math.sin(end_lat) - (math.sin(start_lat) * math.cos(end_lat) * math.cos(delta_lon))

        # Переводим угол в градусы
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360

        return bearing
    
    def run(self,x,y,x_goal,y_goal,target_angle):
        """
        Предназначена для управления движением ровера в пространстве с использованием данных с счетчиков глубины.
        Направляет ровера к заданной цели, избегая препятствий и корректируя его направление в зависимости от обнаруженных препятствий.

        Параметры: 
        - x, y: начальные координаты ровера
        - x_goal,y_goal: целевые координаты ровера
        - target_angle: целевой угол

        Завершение: Останавливает поток данных и закрывает окна OpenCV по завершении работы.

        Возвращаемое значение: Возвращает обновленный self.target_angle.
        """
        self.x=x
        self.y=y
        self.x_g=x_goal
        self.y_g=y_goal
        self.target_angle=target_angle
        pipeline.start(config)
        time_to_rotton=0
        a=True
        def sbros(time_to_rotton,a):
            if time_to_rotton!=0 and a:
                return 0
            else:
                return time_to_rotton
        try:
            # Ждем следующего кадра
            while True:
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                # Преобразование изображений глубины и цвета в массивы NumPy
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Применение порога к изображению глубины для обнаружения препятствий
                depth_threshold = 250 # в миллиметрах
                obstacle_mask = np.where((depth_image > 0) & (depth_image < depth_threshold), 1, 0)

                # Определение центра масс препятствия
                moments = cv2.moments(obstacle_mask.astype(np.uint8))
                if moments['m00'] != 0:
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    # Действие в зависимости от положения препятствия
                    if cx < 320: # Препятствие слева
                        time_to_rotton=sbros(time_to_rotton,a)
                        # print("Поворот направо")
                        time_to_rotton=time_to_rotton+time_to_rot(5)
                        self.degres(self.target_angle+5)
                        # print(time_to_rotton)
                        a=False
                    elif cx > 320: # Препятствие справа
                        time_to_rotton=sbros(time_to_rotton,a)
                        # print("Поворот налево")
                        time_to_rotton=time_to_rotton+time_to_rot(5)
                        self.degres(self.target_angle-5)
                        # print(time_to_rotton)
                        a=False
                        
                else:
                    # print("Препятствий нет, продолжаем движение")
                    if time_to_rotton!=0:
                        grad=time_to_rotton*120
                        distance=abs((depth_threshold/10)/math.cos(math.degrees(grad))+60)
                        self.movment(distance)
                        self.degres(self.degres_to_mark())
                        time_to_rotton=0
                        a=True            
                    else:
                        if self.distance()<3:
                            break
                        self.movment(1)
        finally:
            # Остановка потоков
            pipeline.stop()
            cv2.destroyAllWindows()
        return self.target_angle
    

    
        
    
