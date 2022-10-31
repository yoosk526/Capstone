import RPi.GPIO as GPIO
from time import sleep

# PWM PIN
ENA = 25      # left motor
ENB = 22      # right motor

# GPIO PIN
# left motor
IN1 = 23
IN2 = 24
# right motor
IN3 = 17
IN4 = 27

# PIN state
LOW = 0
HIGH = 1

# motor state
STOP = 0
FORWARD = 1
BACKWARD = 2

# motor channel
CH1 = 0     # left
CH2 = 1     # right

# PIN Setting
def setPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)

    pwm = GPIO.PWM(EN, 100)     # frequency setting = 100Hz
    pwm.start(0)
    return pwm

# Motor Control
def setMotorControl(pwm, INA, INB, speed, Control):
    pwm.ChangeDutyCycle(speed)

    if Control == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    elif Control == BACKWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    elif Control == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)


def setMotor(ch, speed, Control):
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, Control)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, Control)


def goForward(time):
    print("forward")
    setMotor(CH1, 16.5, FORWARD)
    setMotor(CH2, 16.5, FORWARD)
    sleep(time)

def goBackward(time):
    print("bakward")
    setMotor(CH1, 25, BACKWARD)
    setMotor(CH2, 25, BACKWARD)
    sleep(time)


def goLeft():
    print("left")
    # setMotor(CH1, 25, BACKWARD)
    setMotor(CH2, 32, FORWARD)
    sleep(0.07)


def goRight():
    print("right")
    setMotor(CH1, 25, FORWARD)
    # setMotor(CH2, 32, BACKWARD)
    sleep(0.07)


def turnLeft():
    print("turn left")
    setMotor(CH1, 35, BACKWARD)
    setMotor(CH2, 42, FORWARD)
    sleep(1.5)


def turnRight():
    print("turn right")
    setMotor(CH1, 30, FORWARD)
    setMotor(CH2, 42, BACKWARD)
    sleep(1.6)


def stop(time):
    print("stop")
    setMotor(CH1, 80, STOP)
    setMotor(CH2, 80, STOP)
    sleep(time)


def goHome(time):
    print("forward")
    setMotor(CH1, 30, FORWARD)
    setMotor(CH2, 30, FORWARD)
    sleep(time)


GPIO.setmode(GPIO.BCM)

pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

#GPIO.cleanup()
