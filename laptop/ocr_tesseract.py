"""address.jpg에서 호수 숫자 읽기 (Tesseract OCR)

Tesseract로 글자를 읽고, 정규 표현식으로 숫자만 뽑아 ocr_result.txt에 적는다.
"""
import re

import pytesseract
from PIL import Image

from config import TESSERACT_CMD


def ocr():
    pytesseract.pytesseract.tesseract_cmd = TESSERACT_CMD
    image = Image.open("address.jpg")

    text1 = pytesseract.image_to_string(image)
    print('all_text :', text1)
    print("\n")

    # 숫자만 뽑기
    num = re.findall(r"\d+", text1)
    print('num_only : ', num)
    print("\n")

    result = ' '.join(s for s in num)
    print('str_num :', result)

    with open('ocr_result.txt', 'w', encoding='utf8') as f:
        f.write(result)


if __name__ == "__main__":
    ocr()
