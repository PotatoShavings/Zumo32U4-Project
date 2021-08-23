import serial 
from serial.serialutil import SerialException
import cv2 as cv
from time import time
from math import pi

ser = serial.Serial("COM5", 115200) #opens serial port

try: ser.open() #if the port is already open send a message
except SerialException:
    print("Port already opened")

#Time of flight sensor
ToF = 0

#Gyro
Gyro = 0 #raw gyro data
gyroOffset = 0
angle = 0.0
previousTime = 0

#Wheel encoders
Encoders = [0, 0] #encoder, left and right values are left and right wheels respectively
previousEncoders = [0, 0] #previous encoder measurements
hasTurned = False #if the robot has turned

#Proximity sensors
Prox = [0, 0] #proximity, left and right values are left and right sensors respectively

#Sensors in general
sensor, data = "", "" #the current reading from serial

#Finds gyroOffset
ser.write("a".encode("UTF-8")) 
for i in range(100):
    value = 0
    while (True):
        try:
            data = ser.readline().decode("UTF-8").strip("\r\n")
            value = int(data)
            if (ord(data[0]) > 57): continue #if it's not a number or the minus sign
            if (value > 200 or value < -200): continue #sometimes it gives a really high/low value (+- 20,000)
            break
        except ValueError:
            print("ValueError: " + ser.readline().decode("UTF-8").strip("\r\n"))
            pass
    gyroOffset += value
ser.write("b".encode("UTF-8")) #tells the arduino we're done figuring out the gyroOffset
gyroOffset /= 100
print("gyroOffset " + str(gyroOffset))

#Camera
video = cv.VideoCapture(0)
print("Camera has been opened")
#top left corner is (0, 0), top right is (640, 0), bottom left is (0, 480), bottom right is (640, 480)
ball = [0, 0, 0] #location of the ball [x, y, radius] (in pixels)
previousBall = [0, 0, 0] #previous location of the ball
onScreen = False #if the ball has moved 
inCenter = False
lastOnScreen = False

#Motors
previousMotor = [-1, -1]
def motor(left, right): #sends motor readings to serial
    if (left == previousMotor[0] or right == previousMotor[1]): return #prevents multiple messages with the same speeds to be sent
    ser.write((str(left) + ' ' + str(right)).encode())
    previousMotor[0] = left
    previousMotor[1] = right

#Errors


# hz = 0
# previousX = time()
while (True): #similar to loop() in the arduino program
    # if (time() - previousX >= 1): #Hz calculator
    #     print(hz)
    #     hz = 0
    #     previousX = time()

    #for gyro
    previousTime = time()

    try: 
        sensor, data = ser.readline().decode("UTF-8").strip("\r\n").split(" ") #converts serial messages into two strings, the first being the type of sensor and second being the data
    except ValueError:
        print("ValueError: " + ser.readline().decode("UTF-8").strip("\r\n"))
        continue
    data = data.split(",") #if the data has two outputs, split them
    #converts strings into numbers and puts them into their sensor's variable
    if (sensor == "ToF"):
        ToF = int(data[0])
    elif (sensor == "Gyro"):
        Gyro = int(data[0])
    elif (sensor == "Encoders"):
        Encoders = [int(data[0]), int(data[1])]
    else: #(sensor == 'p'):
        Prox = [int(data[0]), int(data[1])]
    print(data)

    #ToF error
    if (ToF == -1):
        print("ToF sensor out of range")

    #finds hasTurned
    if (previousEncoders[0] != Encoders[0] or previousEncoders[1] != Encoders[1]):
        hasTurned = True
    else:
        hasTurned = False
    previousEncoders = Encoders.copy()
    # print(Encoders)
    #finds angle
    if (hasTurned): angle += (Gyro - gyroOffset) * 0.07 * (time() - previousTime) #prevents the gyro from changing if the robot isn't turning
    # print("angle " + str(angle))
    #OpenCV
    on, frame = video.read() #boolean on, frame is the current frame in the camera
    # frame = cv.GaussianBlur(frame, (7, 7), 1) #blurs the image while keeping edges
    frameH = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #changes colorspace
    part1 = cv.inRange(frameH, (170, 175, 0), (179, 255, 255)) #for all pixels, make it white if it's within the color range (HSV), otherwise make it black
    part2 = cv.inRange(frameH, (0, 175, 0), (10, 255, 255))
    mask = cv.bitwise_or(part1, part2)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv.contourArea(contour) #area of contour
        if area > 1000: #if it's large enough
            perimeter = cv.arcLength(contour, True) #perimeter of contour
            circularity = 4 * pi * (area / perimeter ** 2) #range of 0 - 1, higher = more circular; 1 is a perfect circle, 0 is a line, a square is around 0.78
            if circularity < 0.3:
                break #most random detections have a circularity between 0.1 and 0.2

            # current = cv.approxPolyDP(contour, 1, True) #turns a jagged contour into a more polygonal contour
            (x, y), r = cv.minEnclosingCircle(cv.approxPolyDP(contour, 1, True)) #the smallest circle that can enclose the contour
            cv.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2) #circumference
            cv.circle(frame, (int(x), int(y)), 1, (255, 0, 0), 3) #center
            ball[0] = x
            ball[1] = y
            ball[2] = r

    # print(str(ball[0]) + " " + str(ball[1]) + " " + str(ball[2]))
    onScreen = ball[0] != previousBall[0] or ball[1] != previousBall[1] or ball[2] != previousBall[2] #if the ball is on the screen - if the coordinates of the ball are exactly the same (it always fluctuates when on screen), false
    if (not onScreen and lastOnScreen): #prevents 1-tick changes of onScreen from true to false, even if the ball is on the screen
        lastOnScreen = False
        onScreen = True

    inCenter = (ball[0] >= 290 and ball[0] <= 350 and onScreen)
    previousBall = ball.copy()
    # print(str(ball[0]) + " " + str(ball[1]) + " " + str(ball[2]) + " " + str(onScreen))

    cv.imshow("mask", mask)
    cv.imshow("frame", frame)
    if cv.waitKey(1) == ord("x"):
        cv.destroyAllWindows()
        break

    
    #Motors
    if (not onScreen):
       motor(100, -100) #turns right
        # print("right")
    elif (onScreen and not inCenter):
        if (ball[0] < 290):
           motor(-100, 100) #turns left
            # print("left")
        elif (ball[0] > 350):
           motor(100, -100) #turns right
            # print("right")
    else: #inCenter = true
       motor(100, 100)
        # print("forward")
    
    #Hz calculator
    # hz += 1
    