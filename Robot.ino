#include <Zumo32U4.h> //robot brand
#include <Wire.h> //used for startup
#include "Adafruit_VL53L0X.h" //time-of-flight sensor


/* 
 * Notes:
 * The program only reads data from the sensors and prints it to serial right now
 * I haven't included the ultrasonic sensor to this program since it's pretty inaccurate, and I haven't had the time to add the accelerometer (and also since the code looks complicated)
 * I'm planning on making the program control the motors by reading from serial
 * I couldn't find any docs on the time of flight sensor, so I'm just going off an example program (which means I don't know what any of its functions do, so I'm just interpreting using their names)
 * The gyro currently only reads yaw, since we don't really use roll and pitch (since there ground will be flat)
 * I wish Serial had printf ;-;
 * I haven't tested the brigtness/sensitivity system for the prox sensor, so for now, 6 means it's around 1 foot away or less, and any lower values means it's farther away
 * 
 * To differentiate between data sent by different sensors, there's a character (listed below) denoting which sensor sent it. A negative value as the data would mean the measurement was faulty in some way
 * If there are multiple args sent by the sensor, they are separated by a comma instead of a space (makes it easier to use split())
 * <char> <data> Examples: "g 20.5", "t -1", "e 50,100" etc.
 * t = time of flight sensor
 * g = gyro (only yaw - z-axis)
 * e = wheel encoders (first value is left, second is right)
 * p = proximity sensors (first value is left, second is right)
 */


Zumo32U4Motors motors;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Zumo32U4IMU imu;
Zumo32U4Encoders encoders; //wheel encoders
Zumo32U4ProximitySensors prox;

//gyro
float angle = 0;
float gyroZOffset = 0;

//encoders
static int16_t lastLeft = 0;
static int16_t lastRight = 0;
boolean hasTurned = false;

//general time stuff
static uint16_t lastMicro = 0;
static uint8_t lastMilli = 0;

//motors
int movement = -1;

void setup() { //called once at the beginning
  //starting up the robot and opening serial port
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {} //waits until the serial port is available (Serial is a boolean I guess)

  //if the time of flight sensor didn't boot, send a message through serial and stop the program
  if (!lox.begin()) {
    Serial.println("t -2");
    exit(0);
  }

  //starting imu
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  
  //finds the zero offset for the gyro
  for (int i = 0; i < 1000; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro(); //reads the gyro and puts the measurement into vector<int16_t> g = {0, 0, 0} (x, y, z)

    gyroZOffset += imu.g.z;
  }
  gyroZOffset /= 1000;

  //proximity setup
  prox.initFrontSensor();
}

void loop() { //does this constantly after setup()
  if (!((uint8_t) (millis() - lastMilli) >= 100)) return; //lets all the sensors read data at the same time and print to Serial accordingly instead of at varying rates
  lastMilli = millis();
  
  //time of flight
  VL53L0X_RangingMeasurementData_t measure;

  //if a measurement is in range, print it to serial, otherwise it's out of range
  lox.rangingTest(&measure, false); //sends if the measurement is in range or not (and distance I think) to "measure"; set the 2nd arg to true to get "debug data printout" (from the example program)
  if (measure.RangeStatus != 4) { //assuming this means "if the measurement is in range" (4 means it isn't)
    Serial.print("t ");
    Serial.println(measure.RangeMilliMeter);
  } else Serial.println("t -1");


  //encoders
  int16_t left = encoders.getCountsLeft();
  int16_t right = encoders.getCountsRight();
  
  if (left != lastLeft || right != lastRight) { //if the robot hasn't stayed still print to Serial
    Serial.print("e ");
    Serial.print(left);
    Serial.print(",");
    Serial.println(right);
  }
  hasTurned = (left % right != lastLeft % lastRight); //hasTurned = if the robot hasn't stayed still OR if the robot hasn't moved straight forwards/backwards (Will be true whenever the angle of the robot changes)
  lastLeft = left;
  lastRight = right;
  


  //gyro
  uint16_t currentMicro = micros();
  uint16_t difference = currentMicro - lastMicro;
  lastMicro = currentMicro;

  imu.readGyro();
  //only changes the angle when the wheels turn (when the robot isn't still, moving straight forwards, or moving straight backwards)
  if (hasTurned) angle += (((float) imu.g.z - gyroZOffset) * 70 * difference / 1000000000) * 4; //0.07 dps/digit
  Serial.print("g ");
  Serial.println(angle);


  //proximity
  prox.read(); //sends out pulses from the sensors and (I think) stores the values back into prox; retrieve the data with the getter functions
  Serial.print("p ");
  Serial.print(prox.countsFrontWithRightLeds());
  Serial.print(",");
  Serial.println(prox.countsFrontWithRightLeds());

  //follow a red ball
  if (Serial.available() > 0) {
    movement = Serial.read() - 48; //Serial.read() returns the number in ASCII decimal
    switch(movement) {
      case 0: //turn left
        motors.setLeftSpeed(-100);
        motors.setRightSpeed(100);
        break;
      case 1: //turn right
        motors.setLeftSpeed(100);
        motors.setRightSpeed(-100);
        break;
      case 2: //move forward
        motors.setLeftSpeed(150);
        motors.setRightSpeed(150);
        break;
    }
  }
}
