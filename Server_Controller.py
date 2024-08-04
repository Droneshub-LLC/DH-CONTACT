import logging
import socket
import os
import re
import subprocess
import sys
import time

class Attachments:
    # Конструктор класса, инициализирующий соединения с серверами
    def __init__(self, ip, port_rele, port_servo) -> None:
        host = ip
        # Попытка подключения к серверу реле
        try:
            self.client_socket_rele = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket_rele.connect((host, port_rele))
            print("подключенно rele")
        except:
            print("error connect server rele")
        # Попытка подключения к серверу сервопривода
        try:
            self.client_socket_servo = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket_servo.connect((host, port_servo))
            print("подключенно servo")
        except:
            print("error connect server servo")

    # Асинхронный метод для отправки статуса реле
    async def rele_status(self, gradus):
        try:
            self.client_socket_rele.sendall(f"{gradus}".encode('utf-8'))
        except Exception as e:
            print("error rele: ", e)

    # Асинхронный метод для отправки угла поворота сервопривода
    async def servo_grad(self, status):
        try:
            self.client_socket_servo.sendall(f"{int(status)}".encode('utf-8'))
        except Exception as e:
            print("error servo: ", e)

    # Асинхронный метод для получения статуса реле
    async def rele_status_never(self):
        self.client_socket_rele.sendall(f"return".encode('utf-8'))
        data = self.client_socket_rele.recv(self.buffer_size).decode('utf-8')
        return data

    # Асинхронный метод для получения статуса сервопривода
    async def servo_status(self):
        self.client_socket_servo.sendall(f"return".encode('utf-8'))
        data = self.client_socket_servo.recv(self.buffer_size).decode('utf-8')
        return data
        
class Site_run:
    # Конструктор класса, запускающий серверный процесс
    def __init__(self, ip_site, port_site, ip_serv, port_serv, rtsp):
        # Формирование строки с данными о портах
        port_data = f'{ip_site}:{port_site},{ip_serv}:{port_serv},{rtsp}'
        # Добавление данных о портах в аргументы командной строки
        sys.argv.append(port_data)
        self.port = port_site
        # Формирование и выполнение команды для запуска сервера
        command = f'python3.11 Server_site.py {" ".join(sys.argv[1:])}'
        self.processes = subprocess.Popen(command, shell=True)
        # Ожидание для установления процесса
        time.sleep(3)
    
    # Метод для остановки серверного процесса
    def stop(self):
        # Формирование и выполнение команды для поиска PID процесса
        command = f'netstat -a -n -o | find "{self.port}"'
        output = subprocess.check_output(command, shell=True, text=True)
        pattern = r'LISTENING\s+(\d+)'
        pids = re.findall(pattern, output)
        # Формирование и выполнение команды для остановки процесса
        sys = 'taskkill /F /PID ' + str(pids[0])
        os.system(sys)

