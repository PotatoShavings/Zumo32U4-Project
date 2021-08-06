import serial 
from serial.serialutil import SerialException
import cv2 as cv
import numpy as np

ser = serial.Serial("COM3", 9600) #opens serial port
try: ser.open() #if the port is already open send a message
except SerialException:
    print("Port already opened")

t = 0 #time of flight sensor
g = 0 #gyro yaw
e = [0, 0] #encoder, left and right values are left and right wheels respectively
p = [0, 0] #proximity, left and right values are left and right sensors respectively
sensor, data = '', "" #the current reading from serial

video = cv.VideoCapture(0) #the camera
ball = [0, 0, 0] #location of the ball [x, y, radius], x will equal -1 if there is no ball
while (True): #similar to loop() in the arduino program
    try: 
        sensor, data = ser.readline().decode("UTF-8").strip("\r\n").split(" ") #converts serial messages into two strings, the first being the type of sensor and second being the data
    except ValueError:
        print("value error ")
        print(ser.readline().decode("UTF-8").strip("\r\n").split(" "))
        while (True): pass
    data = data.split(",") #if the data has two outputs, split them
    #converts strings into numbers and puts them into their sensor's variable
    if (sensor == 't'):
        t = int(data[0])
    elif (sensor == 'g'):
        g = float(data[0])
    elif (sensor == 'e'):
        e = [int(data[0]), int(data[1])]
    else: #(sensor == 'p'):
        p = [int(data[0]), int(data[1])]
    

    #OpenCV
    on, frame = video.read() #boolean on, frame is the current frame in the camera
    frame = cv.GaussianBlur(frame, (7, 7), 1) #blurs the image while keeping edges
    frameH = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #changes colorspace
    mask = cv.inRange(frameH, (-10, 200, 200), (10, 255, 255)) #for all pixels, make it white if it's within the color range (HSV), otherwise make it black
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv.contourArea(contour) > 1000:
            # current = cv.approxPolyDP(contour, 1, True) #turns a jagged contour into a more polygonal contour
            (x, y), r = cv.minEnclosingCircle(cv.approxPolyDP(contour, 1, True)) #the smallest circle that can enclose the contour
            cv.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 3) #circumference
            cv.circle(frame, (int(x), int(y)), 1, (255, 0, 0), 3) #center
            ball[0] = x
            ball[1] = y
            ball[2] = r
        else:
            ball[0] = -1
    
    cv.imshow("mask", mask)
    cv.imshow("frame", frame)
    if cv.waitKey(1) == ord("x"):
        cv.destroyAllWindows()
        break
    