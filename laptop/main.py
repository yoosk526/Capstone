"""노트북 쪽 실행 파일

라즈베리파이보다 먼저 실행한다.
1. address.jpg를 받을 때까지 기다린다.
2. Tesseract로 호수를 읽어 ocr_result.txt를 만든다.
3. ocr_result.txt를 라즈베리파이로 보낸다.
"""
import echo_client
import echo_server
import ocr_tesseract

echo_server.server()
print("receive success 'address.jpg'")

ocr_tesseract.ocr()
print("'ocr_result.txt' was created")

echo_client.client()
print("send success 'ocr_result.txt'")
