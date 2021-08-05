import serial 
from serial.serialutil import SerialException

ser = serial.Serial("COM3", 9600) #opens serial port
try: ser.open() #if the port is already open send a message
except SerialException:
    print("Port already opened")

t = 0 #time of flight sensor
g = 0 #gyro yaw
e = [0, 0] #encoder, left and right values are left and right wheels respectively
p = [0, 0] #proximity, left and right values are left and right sensors respectively

sensor, data = '', "" #the current reading from serial

while (True): #similar to loop() in the arduino program
    sensor, data = ser.readline().decode("UTF-8").strip("\r\n").split(" ") #converts serial messages into two strings, the first being the type of sensor and second being the data
    data = data.split(",") #if the data has two outputs, split them
    
    #converts strings into numbers and puts them into their sensor's variable
    if (sensor == 't'):
        t = int(data[0])
    elif (sensor == 'g'):
        g = float(data[0])
    elif (sensor == 'e'):
        e = [int(data[0]), int(data[1])]
    elif (sensor == 'p'):
        p = [int(data[0]), int(data[1])]