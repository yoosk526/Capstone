"""노트북이 보낸 ocr_result.txt 받기 (TCP 소켓 서버)

노트북(laptop/echo_client.py)이 연결해 올 때까지 기다렸다가 파일을 받아 저장한다.
"""
import os
import socket

import tqdm

from config import RASPBERRY_PI_IP, PORT

SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096      # 4KB


def server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # TCP
    sock.bind((RASPBERRY_PI_IP, PORT))
    sock.listen(1)

    print('waiting for client')
    connection, client_address = sock.accept()
    print('Connected address: ', client_address)

    # 파일 정보 받기
    received = connection.recv(BUFFER_SIZE).decode()
    filename, filesize = received.split(SEPARATOR)
    filename = os.path.basename(filename)
    filesize = int(filesize)

    # 파일 내용 받기
    progress = tqdm.tqdm(range(filesize), f"Receiving {filename}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(filename, "wb") as f:
        while True:
            bytes_read = connection.recv(BUFFER_SIZE)
            if not bytes_read:
                break
            f.write(bytes_read)
            progress.update(len(bytes_read))

    connection.close()


if __name__ == "__main__":
    server()
