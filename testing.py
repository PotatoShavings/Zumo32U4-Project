import serial 
from serial.serialutil import SerialException
import time
import cv2 as cv
from math import pi
from multiprocessing import Process, Queue

ser = serial.Serial("COM3", 115200) #opens serial port

try:
    ser.open() #if the port is already open send a message
except SerialException:
    print("Port already opened")

currentLine = None
class Serial: #General Serial management
    def rawNext(): #next serial line
        while True:
            try:
                return ser.readline().decode("UTF-8").strip("\r\n")
            except ValueError: #sometimes the data corrupts
                print("ValueError")
    
    def next(): #for sensor data only
        next = [0, 0] #[sensor type, data from sensor]
        while True:
            try:
                #data is sent in the form of "<sensor type> <data point 1>(,<optional nth data point>...)"
                rawData = Serial.rawNext().split(" ")
                next[0] = rawData[0]
                if rawData[0] == "ToF" or rawData[0] == "Gyro": #one data point
                    next[1] = int(rawData[1])
                else: #two+ data points
                    next[1] = [int(x) for x in rawData[1].split(",")]
                break
            except IndexError: #sometimes the data corrupts
                print("IndexError")
        global currentLine
        currentLine = next

class Video:
    ball = [0, 0, 0] #location of the ball [x, y, radius] (in pixels)
    previousBall = [0, 0, 0] #previous location of the ball
    onScreen = False #if the ball has moved 
    inCenter = False
    lastOnScreen = False #the previous measurement of onScreen
    oneTick = 0

    def update(self, onScreen, ball):
        pass

    def main(self, queue): #main loop, second process
        print("camera main")
        video = cv.VideoCapture(0)
        print("Camera has been opened")
        video.set(cv.CAP_PROP_BUFFERSIZE, 1) # reduces delay
        hz = hzMonitor("Video")

        while True:
            on, frame = video.read() #boolean on, frame is the current frame in the camera
            # frame = image.array
            frame = cv.flip(frame, -1)
            # frame = cv.GaussianBlur(frame, (7, 7), 1) #blurs the image while keeping edges
            frameH = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #changes colorspace
            part1 = cv.inRange(frameH, (160, 150, 50), (179, 255, 255)) #for all pixels, make it white if it's within the color range (HSV), otherwise make it black
            part2 = cv.inRange(frameH, (0, 150, 50), (5, 255, 255))
            mask = cv.bitwise_or(part1, part2)
            contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            screen = False
            for contour in contours:
                area = cv.contourArea(contour) #area of contour
                if area > 500: #if it's large enough
                    perimeter = cv.arcLength(contour, True) #perimeter of contour
                    circularity = 4 * pi * (area / perimeter ** 2) #range of 0 - 1, higher = more circular; 1 is a perfect circle, 0 is a line, a square is around 0.78
                    if circularity < 0.3:
                        break #most random detections have a circularity between 0.1 and 0.2

                    # current = cv.approxPolyDP(contour, 1, True) #turns a jagged contour into a more polygonal contour
                    screen = True
                    (x, y), r = cv.minEnclosingCircle(cv.approxPolyDP(contour, 1, True)) #the smallest circle that can enclose the contour
                    cv.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2) #circumference
                    # cv.circle(frame, (int(x), int(y)), 1, (255, 0, 0), 3) #center
                    queue.put([True, x, y, r])
                    # self.ball = [True, x, y, r]
                    break
            if not screen: queue.put([False, -1, -1, -1])
            # if not screen: self.ball = [False, -1, -1, -1]

            # cv.imwrite("test.jpg", frame) #worse version of cv.imshow(), since you can't use it
            # cv.imwrite("test1.jpg", mask)
            cv.imshow("frame", frame)
            cv.imshow("mask", mask)
            if cv.waitKey(1) == ord("x"):
                cv.destroyAllWindows()
                break
            
            hz.main()

class Motor:
    previousMotor = [-1, -1] #last sent motor data from the python program
    pause = 0
    startTime = 0

    def motor(self, left, right): #sends motor readings to serial, time is the amount of time to do it
        # print(previousMotor)
        if left == self.previousMotor[0] and right == self.previousMotor[1]: return #prevents multiple messages with the same speeds to be sent
        if self.pause != 0:
            if time.time() - self.startTime > self.pause:
                self.pause = 0
            else:
                return
                
        ser.write((" " + str(left) + " " + str(right)).encode())
        self.previousMotor[0] = left
        self.previousMotor[1] = right

    def pause(self, pauseLength):
        self.startTime = time.time()
        self.pause = pauseLength
        


class Sensor(Serial): #Parent of all sensors
    def __init__(self, type = ""):
        self.type = type

    def update(self): #puts Serial.next() into value
        if currentLine[0] == self.type:
            self.value = currentLine[1]

class Gyroscope(Sensor):
    gyroOffset = 0
    gyroAngle = 0

    def __init__(self, value = 0):
        super().__init__("Gyro")
        self.value = value
    
    def update(self):
        if currentLine[0] == self.type:
            currentTime = time.time()
            self.value = currentLine[1]
            self.gyroAngle = (self.value - self.gyroOffset) * 0.07 * (time.time() - currentTime)
            
    def findOffset(self):
        ser.write("a".encode())
        for i in range(100):
            while True: #so if this continues you still divide gyroOffset by the correct amount
                try:
                    line = Serial.rawNext()
                    if ord(line[0]) > 57: continue #if the first character isn't a number or -

                    value = int(line)
                    if value > 200 or value < -200: continue #sometimes it gives a random high number (+-2000)

                    self.gyroOffset += value
                    break
                except (ValueError, IndexError):
                    print("gyroOffset Error")
        ser.write("a".encode())
        self.gyroOffset /= 100
        print("gyroOffset: " + str(self.gyroOffset))
    

class TimeOfFlight(Sensor):
    def __init__(self, value = 0):
        super().__init__("ToF")
        self.value = value

    def update(self):
        if currentLine[0] == self.type:
            self.value = currentLine[1]
            if self.value == -1: print("ToF out of range")

class Encoders(Sensor):
    previous = [0, 0] #last measurement from the wheels

    def __init__(self, value = [0, 0]):
        super().__init__("Encoders")
        self.value = value

    def update(self):
        if currentLine[0] == self.type:
            self.previousEncoders = self.value
            self.value = currentLine[1]

    def hasTurned(self):
        return self.previous[0] != self.value[0] or self.previous[1] != self.value[1]
    
    def encoderAngle(self):
        return (((((2 * pi * 0.825) / 909.7) * (self.value[0] - self.value[1])) / 3.825) * 180) / pi

class Proximity(Sensor):
    def __init__(self, value = [0, 0]):
        super().__init__("Prox")
        self.value = value

class hzMonitor:
    hz = 0
    startTime = time.time()

    def __init__(self, name = ""):
        if name != "":
            self.name = name + " "
        else:
            self.name = name

    def main(self):
        currentTime = time.time()

        self.hz += 1
        if currentTime - self.startTime > 1:
            print(self.name + "Hz: " + str(self.hz))
            self.hz = 0
            self.startTime = currentTime

class Processing:
    def start(self):
        self.video = Video()
        self.queue = Queue(maxsize = 1)
        proc = Process(target = self.video.main, args = (self.queue,))
        proc.start()
        print("Video Process Started")
    
    def getQueue(self): 
        if not self.queue.empty():
            current = self.queue.get() #[onScreen, ball position]
            onScreen = current[0]
            current.pop(0) #removes item at 0
            self.video.update(onScreen, current)

if __name__ == "__main__":
    proc = Processing()
    gyro = Gyroscope()
    ToF = TimeOfFlight()
    encoders = Encoders()
    prox = Proximity()
    hz = hzMonitor("Main")
    def sensorUpdate():
        Serial.next()
        gyro.update()
        ToF.update()
        encoders.update()
        prox.update()

    gyro.findOffset()
    proc.start()
    while True: #main loop
        sensorUpdate()
        proc.getQueue()
        
        hz.main()
    