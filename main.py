import cv2
import numpy as np
import math
import time
import motor_control as motor
import RPi.GPIO as GPIO
from motor_control import goRight, pwmA, pwmB

# module for server
import box_capture
import echo_client
import server_only

Red, Green = 16, 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(Red,GPIO.OUT)     
GPIO.setup(Green,GPIO.OUT)     
GPIO.output(Red, True)
GPIO.output(Green, False)

REJECT_DEGREE_TH = 4.0

def select_white_yellow(image):                             
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    # white color mask
    lower = np.uint8([0, 150,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # yellow color mask
    lower = np.uint8([20, 70, 70])
    upper = np.uint8([35, 210, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(image, image, mask=mask)

def filter_region(image, vertices):
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:                    
        cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(image, mask)

def select_region_line(image):
    rows, cols = image.shape[:2]            # height, width
    top_left        = [cols*0, rows*0]
    top_right       = [cols*1.0, rows*0]
    bottom_left     = [cols*0.3, rows*0.5]
    bottom_right    = [cols*0.7, rows*0.5]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)

def FilterLines(Lines):
    FinalLines = []
    cntLines = []

    for Line in Lines:
        [[x1, y1, x2, y2]] = Line

        # Calculating equation of the line: y = mx + c
        if x1 != x2:
            m = (y2 - y1) / (x2 - x1)
        else:
            m = 100000000
        c = y2 - m*x2
        # theta will contain values between -90 ~ +90. radian to degree
        theta = math.degrees(math.atan(m))

        if REJECT_DEGREE_TH <= abs(theta) <= (90 - REJECT_DEGREE_TH):
            l = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            FinalLines.append([x1, y1, x2, y2, m, c, l])
        elif abs(theta) < REJECT_DEGREE_TH:
            l = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            cntLines.append([x1, y1, x2, y2, m, c, l])

        if len(FinalLines) > 15:
            FinalLines = sorted(FinalLines, key=lambda x: x[-1], reverse=True)
            FinalLines = FinalLines[:15]
       
    if len(cntLines) != 0:
        cntLines = sorted(cntLines, key=lambda x: x[-1], reverse=True)
        if cntLines[0][-1] > 100:
            cntLines = cntLines[0]
        else:
            cntLines = []

    return FinalLines, cntLines

def GetVanishingPoint(Lines):
    VanishingPoint = None
    MinError = 100000000000

    for i in range(len(Lines)):
        for j in range(i+1, len(Lines)):
            m1, c1 = Lines[i][4], Lines[i][5]
            m2, c2 = Lines[j][4], Lines[j][5]

            if m1 != m2:
                x0 = (c1 - c2) / (m2 - m1)
                y0 = m1 * x0 + c1

                err = 0
                for k in range(len(Lines)):
                    m, c = Lines[k][4], Lines[k][5]
                    m_ = (-1 / m)
                    c_ = y0 - m_ * x0

                    x_ = (c - c_) / (m_ - m)
                    y_ = m_ * x_ + c_

                    l = math.sqrt((y_ - y0)**2 + (x_ - x0)**2)

                    err += l**2

                err = math.sqrt(err)

                if MinError > err:
                    MinError = err
                    VanishingPoint = [x0, y0]

    return VanishingPoint

def average_slope_intercept(lines):
    left_lines = []     # (slope, intercept)
    left_weights = []   # (length,)
    right_lines = []    # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue
            slope = (y2-y1)/(x2-x1)
            intercept = y1 - slope*x1
            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
            if slope < 0:  # y is reversed in image
                right_lines.append((slope, intercept))
                right_weights.append((length))
            else:
                left_lines.append((slope, intercept))
                left_weights.append((length))

    # add more weight to longer lines
    left_lane = np.dot(left_weights,  left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

    # (slope, intercept), (slope, intercept)
    return left_lane, right_lane

def make_line_points(y1, y2, line):
    if line is None:
        return None

    slope, intercept = line

    if slope != 0:
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))
    else:
        return None

def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)      # return m, c

    y1 = 0                          # bottom of the image = height
    y2 = image.shape[0]*0.55        # slightly lower than the middle

    # End position of each line
    left_line = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)

    return left_line, right_line
    
def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
    line_image = np.zeros_like(image)    # create Black background
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)
    return line_image


box_capture.capture()
print('address.jpg file was created. from main')

echo_client.client()
print("send success 'address.jpg'")

server_only.server()
print("receive success 'ocr_result.txt'")

GPIO.output(Red, False) 
GPIO.output(Green, True)

f = open("ocr_result.txt", "r")
address = f.read()

go_home = 0     # flag for going home
flag    = 0     # flag for cnt
cnt     = 0     # count width line
length  = 0     # criteria for cnt
temp = 0

video_capture = cv2.VideoCapture(0)         # webcam size : 480x640
width = video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
height = video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

while video_capture.isOpened():      
    success, frame = video_capture.read()
    if not success:
        break    
    white_yellow    = select_white_yellow(frame)
    gray            = cv2.cvtColor(white_yellow, cv2.COLOR_RGB2GRAY)
    smooth_gray     = cv2.GaussianBlur(gray, (15, 15), 0)
    edges           = cv2.Canny(smooth_gray, 15, 150)
    regions         = select_region_line(edges)
    lines           = cv2.HoughLinesP(regions, rho=1, theta=np.pi / 180, threshold=20, minLineLength=100, maxLineGap=300)
    cv2.rectangle(regions, (260, 0), (380, 480), (120, 120, 120))       # (width/2 - 60, 0), (width / 2 + 60, height),

    if lines is not None:
        line_for_van, line_for_cnt = FilterLines(lines)
        
        VanishingPoint = GetVanishingPoint(line_for_van)
        if VanishingPoint is not None:
            cv2.circle(regions, (int(VanishingPoint[0]), int(VanishingPoint[1])), 8, (255, 0, 0), -1)
            cv2.putText(regions, "x : %d, y : %d" % (int(VanishingPoint[0]), int(VanishingPoint[1])), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))        # 소실점 좌표값 출력
            if (width/2 - 30 <= int(VanishingPoint[0]) <= width / 2 + 30):    #  260 ~ 380
                # cv2.rectangle(regions, (260, 0), (380, 480), (255,255,255), 2)
                motor.goForward(0.01)
            # left, right condition
            elif (0 < int(VanishingPoint[0]) < width / 2 - 30):
                motor.stop(0.1)
                motor.goLeft()
            elif (width / 2 + 30 < int(VanishingPoint[0]) < width):
                motor.stop(0.1)
                motor.goRight()
            
        left_line, right_line = lane_lines(regions, lines)

        line_image = draw_lane_lines(frame, (left_line, right_line))
        cv2.imshow('Add lines', cv2.addWeighted(frame, 1.0, line_image, 0.95, 0.0))

        if len(line_for_cnt) == 0:
            flag = 1
            print(cnt, flag, "no line for cnt\n")
            temp = 1
        elif (flag == 1 and line_for_cnt[-1] > 220):
            flag = 0
            cnt = cnt + 1
            print("cnt up up!! save length", line_for_cnt[-1], "\n\n")
            length = line_for_cnt[-1]
        
        if temp == 0:
            print(cnt, flag, length, "\nThere is line for cnt, but it's not satisfied the condition\n")
        
        temp = 0

        if go_home == 1 and cnt == int(address[2]) - 7:
            motor.stop(1)
            motor.goHome(0.9)
            motor.stop(1)
            GPIO.output(Red, True)
            GPIO.output(Green, False)
            break

        if go_home == 0 and cnt == int(address[2]) - 6:
            motor.turnLeft()
            motor.stop(3)
            GPIO.output(Red, False)
            GPIO.output(Green, False)
            motor.turnLeft()
            motor.goBackward(0.5)
            motor.stop(2)
            cnt = 0
            go_home = 1
            

    
    cv2.imshow('original', frame)
    cv2.imshow('result', regions)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()

f.close()