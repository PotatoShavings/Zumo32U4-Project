import serial 
from serial.serialutil import SerialException
import time
import cv2 as cv
from math import pi
from math import sqrt
from multiprocessing import Process, Queue

currentLine = None
class Serial: #General Serial management
    def open(self):
        self.ser = serial.Serial("COM5", 250000) #opens serial port
        # self.ser = serial.Serial("/dev/ttyACM0", 250000) #opens serial port

        try:
            self.ser.open() #if the port is already open send a message
        except SerialException:
            print("Port already opened")
        
        

    def rawNext(self): #next serial line
        while True:
            try:
                return self.ser.readline().decode("UTF-8").strip("\r\n")
            except ValueError: #sometimes the data corrupts
                print("ValueError")

    def next(self): #for sensor data only
        next = [0, 0, 0] #[sensor type, data from sensor, time data was taken]
        while True:
            try:
                #data is sent in the form of "<sensor type> <data point 1>(,<optional nth data point>...)"
                rawData = self.rawNext().split(" ")
                next[0] = rawData[0]
                if rawData[0] == "Gyro": #one data point
                    next[1] = int(rawData[1])
                else: #two+ data points
                    next[1] = [int(x) for x in rawData[1].split(",")] 
                next[2] = time.time()
                break
            except IndexError: #sometimes the data corrupts
                print("IndexError")
        global currentLine
        currentLine = next
        
    def write(self, str):
        self.ser.write(str.encode())
        
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
            # frame = cv.flip(frame, -1)
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
    def __init__(self, gyro, encoders, ser): #gyro and encoder objects from the main loop
        self.gyro = gyro
        self.encoders = encoders
        self.ser = ser

        self.isRunning = False
        self.lastSpeed = [0, 0] #for multiple calls of setSpeed() with different left and right values
        self.startAngle = 0
        self.current = 0
        self.goal = 0
        
    
    # previousMotor = [-1, -1] #last sent motor data from the python program

    # sensor = "" #type of sensor, "time", "distance", "angle"
    # start = 0 #start of pause
    # target = 0 #target value before pause is lifted

    def setSpeed(self, left, right):
        if self.isRunning and self.lastSpeed == [left, right]: return

        self.isRunning = True
        self.lastSpeed = [left, right]
        print("speed " + str(left) + " " + str(right))
        self.ser.write(" " + str(left) + " " + str(right))

    def stop(self):
        self.setSpeed(0, 0)
        self.isRunning = False

    def update(self): #updates the current angle
        if not self.isRunning: return
    
        self.current = gyro.getAngle()
        if self.startAngle < 0:
            if self.current - self.startAngle <= self.goal:
                self.stop()
                self.isRunning = False

    def turn(self, angle): #sets values to isRunning and startAngle
        if self.isRunning: return
        
        self.isRunning = True
        self.startAngle = gyro.getAngle()
        self.goal = angle
        if angle > 0: #needs to turn right
            self.setSpeed(100, -100)
        else: self.setSpeed(-100, 100) #left


    #motor returns whether the function printed anything to serial, true if they did, false if not
    def motor(self, left, right, ser, sensor = "", target = 0): #sends motor readings to serial
        # print(previousMotor)

        if left == self.previousMotor[0] and right == self.previousMotor[1] and sensor == "": return False #prevents multiple messages with the same speeds to be sent
        if self.target == 0: ser.write(" " + str(left) + " " + str(right)) #only prints it on the first iteration
        
        self.sensor = sensor
        self.target = target

        #if there is a target in place: sort by type, then set start and return
        #then, if the current value - start > target, set target to 0 and write to serial new motor values
        if (self.target != 0):
            if self.sensor == "time":
                if self.start == 0:
                    self.start = time.time()
                    return False
                print(time.time() - self.start)
                print("t")
                if time.time() - self.start > self.target:
                    self.target = 0
                    self.start = 0
                    self.sensor = ""
                else: return False

            elif self.sensor == "distance":
                # self.encoders.update()
                if self.start == 0:
                    self.start = self.encoders.value                    
                    return False
                print(Encoders.distance(self.start, self.encoders.value))
                print("d")
                if Encoders.distance(self.start, self.encoders.value) > self.target:
                    self.target = 0
                    self.start = 0
                    self.sensor = ""
                else: return False

            elif self.sensor == "angle":
                # self.gyro.update()
                current = self.gyro.getAngle()
                if self.start == 0:
                    self.start = current
                    return False
                print(current - self.start)
                print("a")
                if current - self.start > self.target:
                    self.target = 0
                    self.start = 0
                    self.sensor = ""
                else: return False
            print("b")
            ser.write(" 0 0")
                
        self.previousMotor[0] = left
        self.previousMotor[1] = right
        return True
        


class Sensor: #Parent of all sensors
    def __init__(self, sensor = ""):
        self.type = sensor

    def update(self): #puts Serial.next() into value
        if currentLine[0] == self.type:
            self.value = currentLine[1]

class Gyroscope(Sensor):
    gyroOffset = 0
    gyroAngle = 0
    currentTime = time.time()

    def __init__(self, value = 0):
        super().__init__("Gyro")
        self.value = value
    
    def update(self):
        if currentLine[0] == self.type:
            if currentLine[1] > 5000 or currentLine[1] < -5000: return #sometimes there's a random spike
            self.gyroAngle += (self.value - self.gyroOffset) * 0.07 * (time.time() - self.currentTime)
            self.currentTime = time.time()
            self.value = currentLine[1]
            
    def getAngle(self):
        # print(self.gyroAngle)
        return self.gyroAngle


    def findOffset(self, ser):
        ser.write("a")
        for i in range(100):
            while True: #so if this continues you still divide gyroOffset by the correct amount
                try:
                    line = ser.rawNext()
                    if ord(line[0]) > 57: continue #if the first character isn't a number or -

                    value = int(line)
                    if value > 200 or value < -200: continue #sometimes it gives a random high number (+-2000)

                    self.gyroOffset += value
                    break
                except (ValueError, IndexError):
                    print("gyroOffset Error")
        ser.write("a")
        self.gyroOffset /= 100
        print("gyroOffset: " + str(self.gyroOffset))

class TimeOfFlight(Sensor):
    def __init__(self, value = 0):
        super().__init__("ToF")
        self.value = value

    # def update(self):
    #     if currentLine[0] == self.type:
    #         self.value = currentLine[1]
    #         if self.value == -1: print("ToF out of range")

    def distance(self, ser):
        ser.write("c")
        x = 0
        for i in range(10):
            while True:
                try:
                    x += int(ser.rawNext())
                    break
                except ValueError:
                    pass
        x /= 10
        ser.write("c")
        print(x)
        return x

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

    def distance(start, end): #returns the distance between the two points
        # return sqrt(abs((end[0] - start[0]) ** 2 - (end[1] - start[1]) ** 2)) * 0.0057 #pythagorean theorem (sqrt(a^2 + b^2) = c), each encoder tick is 0.0057 inches 
        return (abs(end[0] - start[0]) + abs(end[1] - start[1])) / 2 * 0.0057 #0.0057 inches per tick
        #averages distance of both wheels, then multiplies by 0.0057

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

class VideoProc:
    def __init__(self, motor):
        self.onScreen = False
        self.motor = motor
        self.ball = [0, 0, 0] #x, y, radius (the camera is 640x480)

    def start(self):
        self.video = Video()
        self.queue = Queue(maxsize = 1)
        proc = Process(target = self.video.main, args = (self.queue,))
        proc.start()
        print("Video Process Started")
    
    def getQueue(self): 
        if not self.queue.empty():
            current = self.queue.get() #[onScreen, ball position]
            self.onScreen = current[0]
            current.pop(0) #removes item at 0
            self.ball = current

    def centerBall(self): #makes the robot look at the ball; returns false if not finished, true if it is
        if not self.onScreen: #if it's not on the screen
            motor.setSpeed(100, -100) #turn right
        elif self.ball[0] >= 300 and self.ball[0] <= 340: #if it's in the center of the screen
            motor.stop()
            print("center")
            return True
        else: #if it's on the screen but not in the center
            location = (self.ball[0] - 320)/320 #instead of going from 0 to 640, it goes from -1 to 1, with 0 being the center
            speed = round(abs(location * 100) + 100) #the farther away, the faster it turns (200 speed at -1 and 1, 100 at 0)
            if location > 0: #right
                motor.setSpeed(speed, -speed)
            else: motor.setSpeed(-speed, speed) #left
        return False


if __name__ == "__main__":
    ser = Serial()
    ser.open()
    ser.write("b") #resets encoders

    gyro = Gyroscope()
    ToF = TimeOfFlight()
    encoders = Encoders()
    prox = Proximity()
    motor = Motor(gyro, encoders, ser)
    proc = VideoProc(motor)
    hz = hzMonitor("Main")
    
    def sensorUpdate():
        ser.next()
        gyro.update()
        ToF.update()
        encoders.update()
        prox.update()

    gyro.findOffset(ser) #the program can only have 1 Serial object since otherwise you'd have to open the serial port multiple times

    proc.start()
    x = 0
    while True: #main loop
        sensorUpdate()
        proc.getQueue()

        #This will contain the logic of the robot
        proc.centerBall()
        
        hz.main()
    
    
# things to work on:
# remove currentLine, make it variable in ser object and make it parameter to <sensor>.update()
# add all functionalities from original code
# drive in square
# calculate for one wheel being bigger on robot
# never declare variables in classes without "self." otherwise it creates global variables