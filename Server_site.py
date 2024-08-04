import asyncio
import base64
import contextlib
import io
import json
import sys
import threading
from PIL import Image
import cv2
import tornado.ioloop
import tornado.web
import tornado.websocket
import socket

global mass
mass=None

if len(sys.argv) > 1:
    # Первый аргумент после имени скрипта будет содержать данные о портах    
    port_data = sys.argv[1]
    port_data = port_data.split(',')
    print(port_data)
    ports = port_data[0].split(':')    
    ports2 = port_data[1].split(':')
    ip_site=ports[0]
    port_site=ports[1]
    host = ports2[0]
    port = int(ports2[1])
    rtsp = port_data[2]
else:    
    print("Данные о портах не были переданы")

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
server_socket.bind((host, port))
server_socket.listen(5)
print(f"Server listening on {host}:{port}")
            

class MainHandler(tornado.web.RequestHandler):
    """
        Обработка GET-запроса.

        Описание:
        Метод вызывается при получении GET-запроса от клиента. Он отвечает за рендеринг HTML-шаблона 'in.html'.
    """
    def get(self):
        self.render("in.html")
        

class SimpleWebSocket(tornado.websocket.WebSocketHandler):
    def open(self):
        """
        Обработка открытия WebSocket.

        Описание:
        Метод вызывается при установлении нового WebSocket соединения с клиентом. Он инициирует асинхронную задачу отправки обновлений клиенту.
        """
        print("WebSocket opened")
        asyncio.create_task(self.send_updates())

    async def send_updates(self):
        """
        Асинхронная отправка обновлений клиенту.

        Описание:
        Метод выполняется в бесконечном цикле, отправляя клиенту кадры (frame) и данные графика (plot), если они доступны. Использует глобальную переменную 'mass' для отправки данных графика.
        """
        while True:
            frame = get_frame()
            
            # plot_data = create_plotly_figure()
            if frame:
                await self.write_message(json.dumps({'type': 'frame', 'frame': frame}))
            global mass
            if not mass==None:
                await self.write_message(json.dumps({'type': 'plot', 'plot': mass}))
            await asyncio.sleep(0)  # Передача управления для обработки других задач

    def on_message(self, message):
        """
        Обработка входящих сообщений от клиента.

        Описание:
        Метод вызывается при получении сообщения от клиента. В текущей реализации метод не выполняет никаких действий.
        """
        # print(message)
        pass

    def on_close(self):
        """
        Обработка закрытия WebSocket.

        Описание:
        Метод вызывается при закрытии WebSocket соединения. Выводит сообщение о закрытии соединения.
        """
        print("WebSocket closed")


cap=cv2.VideoCapture(rtsp)
def get_frame():
    """
    Захват кадра из видеопотока и его преобразование в строку base64.

    Библиотеки:
    - cv2: для захвата видео
    - io: для работы с потоками ввода/вывода
    - PIL: для работы с изображениями
    - base64: для кодирования данных в формат base64

    Возвращает:
    - str: Строка, содержащая изображение в формате JPEG, закодированное в base64.

    Описание:
    Функция захватывает кадр из видеопотока, преобразует его в формат JPEG и кодирует в строку base64.
    Если захват кадра не удался, функция возвращает None.
    """
    _, img = cap.read()
    if img is not None:
        buffer = io.BytesIO()
        pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        pil_img.save(buffer, format='JPEG')
        img_str = base64.b64encode(buffer.getvalue()).decode('utf-8')
        return img_str
    else:
        return None


# Функция для запуска Tornado веб-сервера
async def start_tornado():
    """
    Настройка и запуск асинхронного веб-сервера Tornado.

    Библиотеки:
    - tornado.web: для создания веб-приложения
    - tornado.websocket: для работы с WebSocket
    - asyncio: для асинхронного программирования

    Параметры:
    - Нет параметров.

    Описание:
    Функция создает экземпляр веб-приложения Tornado с обработчиками для HTTP и WebSocket,
    запускает его на указанном IP-адресе и порту, и ожидает событий.
    """
    app= tornado.web.Application([
        (r"/", MainHandler),
        (r"/websocket", SimpleWebSocket),
    ])
    app.listen(port_site,ip_site)
    print(f"Сервер Tornado запущен на http://{ip_site}:{port_site}")
    await asyncio.Event().wait()


def async_task():
    """
    Обработка входящих соединений и данных от клиентов.

    Библиотеки:
    - socket: для работы с сетевыми сокетами
    - threading: для многопоточности

    Параметры:
    - Нет параметров.

    Возвращает:
    - Ничего не возвращает.

    Описание:
    Функция ожидает входящие соединения на сокете сервера. При установлении соединения
    данные от клиента считываются и сохраняются в глобальной переменной `mass`.
    После обработки данных соединение закрывается.
    """
    global mass
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected by {addr}")
        try:
            while True:
                data = client_socket.recv(65536).decode('utf-8')
                if not data:
                    mass=None
                    break
                mass=data
        finally:
            client_socket.close()
            print("Connection closed.")
            # return


async def main():
    # Создание задачи для асинхронной функции
    socket_thread = threading.Thread(target=async_task, daemon=True)
    socket_thread.start()
    with contextlib.suppress(KeyboardInterrupt):
        # Запуск сервера Tornado
        await start_tornado()
        socket_thread.join()


if __name__ == "__main__":
    asyncio.run(main())
    server_socket.close()
    print("Server stopped.")
    
    

        
