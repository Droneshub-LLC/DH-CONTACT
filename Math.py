# импорт библиотек
import math
import heapq

class Math():
    
    """
    Класс с математическими функциями для построения графов, алгоритмов и нахождения маркеров.

    В основном нициализируется в файле main.py (ctrl + click для просмотра файла).
    """

    def __init__(self,aruco_map):
        self.aruco_map = aruco_map

    def build_graph(self):
        """
        Построение графа смежности на основе дистанций между маркерами.

        Возвращает:
        - dict: Граф смежности, где ключами являются идентификаторы маркеров, а значениями - словари соседних маркеров и расстояний до них.

        Описание:
        Соединяет маркеры, расположенные на расстоянии менее одного метра друг от друга.
        """

        graph = {}

        for id1, marker1 in self.aruco_map.items():
            neighbors = {}
            for id2, marker2 in self.aruco_map.items():
                dist = math.sqrt((marker2['x'] - marker1['x'])**2 + (marker2['y'] - marker1['y'])**2)
                if dist < 1.0:  # Соединяем маркеры, если они находятся на расстоянии менее 1 метра
                    neighbors[id2] = dist
            graph[id1] = neighbors
        return graph

    def find_closest_marker_id(self):
        """
        Нахождение ближайшего маркера к текущей позиции робота.

        Возвращает:
        - int: Идентификатор ближайшего маркера.

        Описание:
        Рассчитывает расстояние до каждого маркера в aruco_map и возвращает идентификатор маркера с минимальным расстоянием.
        """
        return min(self.aruco_map, key=lambda id: math.hypot(self.aruco_map[id]['x'] - self.robot_state.x, self.aruco_map[id]['y'] - self.robot_state.y))

    async def astar_algorithm(self, start, goal):
        """
        Реализация алгоритма A* для поиска пути.

        Параметры:
        - start (int): Идентификатор начального маркера.
        - goal (int): Идентификатор целевого маркера.

        Возвращает:
        - list: Список идентификаторов маркеров, представляющих кратчайший путь от начального к целевому маркеру.

        Описание:
        Использует алгоритм A* для нахождения кратчайшего пути на графе маркеров. 
        Возвращает путь в виде списка идентификаторов маркеров.
        """
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {node: float('inf') for node in self.aruco_map}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.aruco_map}
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]  # Return reversed path

            for neighbor, weight in self.graph[current].items():
                tentative_g_score = g_score[current] + weight
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []
    
    def heuristic(self, a, b):
        """
        Эвристическая функция для оценки расстояния между двумя маркерами.

        Параметры:
        - a (int): Идентификатор первого маркера.
        - b (int): Идентификатор второго маркера.

        Возвращает:
        - float: Расстояние между маркерами в метрах.

        Описание:
        Рассчитывает евклидово расстояние между двумя маркерами на основе их координат.
        """
        return math.sqrt((self.aruco_map[b]['x'] - self.aruco_map[a]['x'])**2 + (self.aruco_map[b]['y'] - self.aruco_map[a]['y'])**2)
