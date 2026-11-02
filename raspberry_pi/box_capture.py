"""상자 캡처와 이미지 처리 (Raspberry Pi 전용 카메라)

1. 화면을 흑백으로 바꿔 밝기 150 기준으로 이진화하고, 가장 큰 윤곽선을 상자로 본다.
2. 상자를 감싸는 사각형이 관심 영역 (100,100)~(530,380) 안에 30프레임 넘게 연속으로 있으면 사진을 찍는다.
3. 상자 테두리에서 15px 바깥은 모두 회색(170)으로 칠한다.
4. 밝기 80 기준으로 다시 이진화해서 호수 숫자만 검게 남기고 address.jpg로 저장한다.
"""
import cv2
import numpy as np

# 관심 영역 (화면에 노란 점 네 개로 표시)
ROI_LEFT, ROI_TOP = 100, 100
ROI_RIGHT, ROI_BOTTOM = 530, 380
STABLE_FRAMES = 30      # 이 값보다 많은 프레임 동안 상자가 관심 영역 안에 있으면 캡처
MARGIN = 15             # 상자 테두리 바깥 여유 (px)
BG_COLOR = (170, 170, 170)


def capture():
    cap = cv2.VideoCapture(0)
    key = 0     # 상자가 관심 영역 안에 연속으로 잡힌 프레임 수

    while cap.isOpened():
        success, frame = cap.read()
        if success:
            origin = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            retval, bin = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # 가장 큰 윤곽선 = 상자
            max_area = 0
            max_index = -1
            index = -1
            for i in contours:
                area = cv2.contourArea(i)
                index = index + 1
                if area > max_area:
                    max_area = area
                    max_index = index

            if max_index == -1:     # 검출된 윤곽선이 없으면
                break

            # 원본 화면에 윤곽선 표시
            cv2.drawContours(frame, contours, max_index, (0, 255, 0), 3)

            # 윤곽선을 감싸는 가장 작은 사각형
            cnt = contours[max_index]
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box1 = np.int0(box)
            box2 = np.float64(box)
            result = cv2.drawContours(frame, [box1], 0, (0, 0, 255), 2)

            # 관심 영역의 네 꼭짓점 표시
            cv2.circle(result, (ROI_LEFT, ROI_TOP), 10, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(result, (ROI_LEFT, ROI_BOTTOM), 10, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(result, (ROI_RIGHT, ROI_TOP), 10, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(result, (ROI_RIGHT, ROI_BOTTOM), 10, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.imshow('Camera Window', result)

            # 상자가 관심 영역 안에 있으면 key를 1씩 올리고, 벗어나면 0으로 되돌린다
            if (box1[0][0] >= ROI_LEFT and box1[2][0] <= ROI_RIGHT
                    and box1[0][1] >= ROI_TOP and box1[2][1] <= ROI_BOTTOM):
                key = key + 1
            else:
                key = 0

            print("key : ", key)

            if key > STABLE_FRAMES:
                cv2.imwrite("opencv.jpg", origin)
                img_bgr = cv2.imread('opencv.jpg')

                # 상자 바깥을 회색으로 칠하기
                for i in range(480):
                    for j in range(640):
                        if not (box2[0][1] - MARGIN < i < box2[2][1] + MARGIN
                                and box2[0][0] - MARGIN < j < box2[2][0] + MARGIN):
                            img_bgr[i, j, :] = BG_COLOR
                cv2.imwrite('pre_address.jpg', img_bgr)

                # 밝기 80 기준 이진화 → 호수 숫자만 검게 남김
                ImageFile = cv2.imread("pre_address.jpg", cv2.IMREAD_GRAYSCALE)
                ret, thresh_cv = cv2.threshold(ImageFile, 80, 255, cv2.THRESH_BINARY)

                cv2.imwrite("address.jpg", thresh_cv)
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    capture()
