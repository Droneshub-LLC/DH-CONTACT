import json
import socket
import time
import pyrealsense2 as rs
import numpy as np
import numpy.typing as npt

# Инициализация объекта контекста RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

mass=[]

pipeline.start(config)
    
host = 'localhost'
port = 5527
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
server_socket.bind((host, port))    
server_socket.listen(5)
print(f"Server listening on {host}:{port}")

def poisson_disk_sampling(orig_points: npt.NDArray, n_points: int,
                          min_distance: float, seed: int, MAX_ITER=1000):
    # Инициализация генератора случайных чисел
    rng = np.random.default_rng(seed)
    # Выбор случайной начальной точки
    selected_id = rng.choice(range(orig_points.shape[0]))
    selected_ids = [selected_id]
    loop = 0
    # Пока не выбрано нужное количество точек или не достигнут максимум итераций
    while len(selected_ids) < n_points and loop <= MAX_ITER:
        # Выбор новой случайной точки
        selected_id = rng.choice(range(orig_points.shape[0]))
        base_point = orig_points[selected_id, :]
        # Расчет расстояний от новой точки до уже выбранных
        distances = np.linalg.norm(
            orig_points[np.newaxis, selected_ids] - base_point,
            axis=2).flatten()
        # Если минимальное расстояние подходит, добавляем точку в выборку
        if min(distances) >= min_distance:
            selected_ids.append(selected_id)
        loop += 1
    # Если не удалось найти нужное количество точек, выводим сообщение
    if len(selected_ids) != n_points:
        print("Could not find the specified number of points...")
    return orig_points[selected_ids]

def segment_objects(points, threshold=10):
    """
    Разбивает массив точек на объекты, основываясь на пороговом значении расстояния между точками.
    """
    objects = []
    current_object = []

    for point in points:
        if not current_object:
            current_object.append(point)
            continue

        # Вычисляем расстояние до предыдущей точки
        distance = np.linalg.norm(np.array(point) - np.array(current_object[-1]))

        if distance <= threshold:
            current_object.append(point)
        else:
            objects.append(np.array(current_object))
            current_object = [point]

    if current_object:
        objects.append(np.array(current_object))

    return objects

def summarize_objects(objects):
    """
    Нахождение центра масс (средней точки) для каждого объекта.
    """
    summarized_points = []

    for obj in objects:
        centroid = np.mean(obj, axis=0)
        summarized_points.append(centroid)

    return np.array(summarized_points)
    
def simplify_depth_data(depth_image, width, height):
    # Вычисляем количество строк и столбцов в упрощенном изображении
    num_rows = depth_image.shape[0] // height
    num_cols = depth_image.shape[1] // width
    summarize_points = []

    # Перебираем каждый блок в изображении
    for row in range(num_rows):
        for col in range(num_cols):
            # Выбираем блок данных глубины
            block = depth_image[row * height:(row + 1) * height,
                                col * width:(col + 1) * width]
            # Вычисляем среднее значение глубины для блока
            average_distance = np.mean(block)
            # Определяем центральную точку блока
            center_x = (col * width) + (width // 2)
            center_y = (row * height) + (height // 2)
            # Добавляем упрощенную точку данных с усредненной глубиной
            summarize_points.append([center_x, center_y, average_distance / 1000])

    # Возвращаем список упрощенных точек данных
    return summarize_points

def send_data_in_chunks(connection, data, chunk_size=10000):
    str2=str(data)
    encoded_data=str2.encode()
    connection.sendall(encoded_data)
    
try:
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected by {addr}")

        try:
            while True:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue

                # Преобразование кадра глубины в массив NumPy
                depth_image = np.asanyarray(depth_frame.get_data())
                
                data = simplify_depth_data(depth_image, 100, 50)
                
                send_data_in_chunks(client_socket, data)

                

        finally:
            client_socket.close()
            print("Connection closed.")

finally:
    pipeline.stop()
    server_socket.close()
    print("Server stopped.")