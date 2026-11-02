# 네트워크 설정
# 시연 때는 노트북의 모바일 핫스팟에 라즈베리파이를 연결해서 두 기기를 같은 네트워크에 두었다.
# IP는 연결할 때마다 바뀔 수 있으니 실행 전에 꼭 확인한다.
#   - 노트북: cmd에서 ipconfig
#   - 라즈베리파이: 터미널에서 ifconfig
# raspberry_pi/config.py 와 같은 값을 써야 한다.

LAPTOP_IP = "192.168.137.1"          # 노트북 (address.jpg를 받는 쪽)
RASPBERRY_PI_IP = "192.168.137.147"  # 라즈베리파이 (ocr_result.txt를 받는 쪽)
PORT = 5001

# Windows에 Tesseract-OCR을 설치한 경로
TESSERACT_CMD = "C:/Tesseract-OCR/tesseract.exe"
