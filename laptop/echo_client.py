"""ocr_result.txt를 라즈베리파이로 보내기 (TCP 소켓 클라이언트)

먼저 "파일 이름<SEPARATOR>파일 크기"를 보내고, 이어서 파일 내용을 4KB씩 나눠 보낸다.
라즈베리파이 쪽 서버(raspberry_pi/server_only.py)가 먼저 기다리고 있어야 한다.
"""
import os
import socket

import tqdm

from config import RASPBERRY_PI_IP, PORT

SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096      # 4KB


def client(filename="ocr_result.txt"):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # TCP
    filesize = os.path.getsize(filename)

    server_address = (RASPBERRY_PI_IP, PORT)
    print('Connect to: {} port: {}'.format(*server_address))
    sock.connect(server_address)

    # 파일 정보 먼저 보내기
    sock.send(f"{filename}{SEPARATOR}{filesize}".encode())

    # 파일 내용 보내기
    progress = tqdm.tqdm(range(filesize), f"Sending {filename}", unit="B", unit_scale=True, unit_divisor=1024)
    with open(filename, "rb") as f:
        while True:
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                break
            sock.sendall(bytes_read)
            progress.update(len(bytes_read))

    sock.close()


if __name__ == "__main__":
    client()
