
#include <Zumo32U4.h> //robot brand
#include <Wire.h> //used for startup
#include "Adafruit_VL53L0X.h" //time-of-flight sensor


/*
 * Gets raw data from sensors and sends it over serial
 * Data format:
 * "<sensor name> <data value 1>(,<data value 2>) (if applicable)
 * 
 * Examples:
 * Encoders 25,-100
 * Prox 6,5
 * Gyro 4000
 * ToF
 */


Zumo32U4Motors motors;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Zumo32U4IMU imu;
Zumo32U4Encoders encoders; //wheel encoders
Zumo32U4ProximitySensors prox;


void setup() { //called once at the beginning
  //starting up the robot and opening serial port
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {} //waits until the serial port is available (Serial is a boolean I guess)
  Serial.println("started");
  //if the time of flight sensor didn't boot, send a message through serial and stop the program
  if (!lox.begin()) {
    Serial.println("ToF -2");
    exit(0);
  }

  //imu setup
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  //proximity sensor setup
  prox.initFrontSensor();
}

//uint16_t lastMilli = 0;
VL53L0X_RangingMeasurementData_t measure; //for ToF
int movement = -1; //for motor control

//gyroOffset calculation
boolean notDoneOffset = false;

//motor values
int left = 0;
int right = 0;
boolean reset = false;

//hz checker
//int x = 0;
//uint16_t lastMillis = millis();

void loop() { //does this constantly after setup()
//  if ((uint16_t) millis() - lastMillis >= 1000) { //hz checker
//    Serial.print("------------------------------- ");
//    Serial.println(x);
//    lastMillis = millis();
//    x = 0;
//  }

  //stores any input into variable reading
  
  /*
   * a = sends gyro until another character is recieved
   * b = resets all encoder values
   * c = sends ToF until another character is recieved
   */
  if (Serial.available() == 1) {
    if (Serial.read() == 98) {
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    } else notDoneOffset = false;
  }
  if (notDoneOffset == false) {
    while(Serial.available() == 0) {
      imu.readGyro();
      Serial.println(imu.g.z);
    }
    Serial.read();
  }
  if (!notDoneOffset) notDoneOffset = true;
  
  //time of flight
  //if a measurement is in range, print it to serial, otherwise it's out of range
  lox.rangingTest(&measure, false); //sends if the measurement is in range or not (and distance I think) to "measure"; set the 2nd arg to true to get "debug data printout" (from the example program)
  if (measure.RangeStatus != 4) { //assuming this means "if the measurement is in range" (4 means it isn't)
    Serial.print("ToF ");
    Serial.println(measure.RangeMilliMeter);
  } else Serial.println("ToF -1");


  //encoders
  Serial.print("Encoders ");
  Serial.print(encoders.getCountsLeft());
  Serial.print(",");
  Serial.println(encoders.getCountsRight());

  //gyro
  imu.readGyro();
  
  Serial.print("Gyro ");
  Serial.println(imu.g.z);


  //proximity
  prox.read(); //sends out pulses from the sensors and (I think) stores the values back into prox; retrieve the data with the getter functions
  Serial.print("Prox ");
  Serial.print(prox.countsFrontWithLeftLeds());
  Serial.print(",");
  Serial.println(prox.countsFrontWithRightLeds());

  //follow a red ball
  if (Serial.available() > 1) {
    left = readMotor();
    right = readMotor();
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
  }
  //  x++; //hz checker
}

//There's a space in front of the first value, so that if the python program is faster than the arduino, it will allow each message from the python to be parsed individually, instead of be bunched together
//For example, if I sent two "10 10" to the arduino, it would result in "10 1010 10" which would cause the right motor to go at a 1010 speed. But, if I sent two " 10 10", it would result in both motors maintaining that speed
int readMotor() {
  Serial.read(); //spaces
  int first = Serial.read();
  
  boolean isNegative = first == 45;
  String current = "";
  if (!isNegative) current = String(first - 48);
  
  while (true) {
    int digit = Serial.peek();
    if (digit == 32 || digit == -1) break; //if the character is a space or if there's no more data
    Serial.read();
    current.concat(String(digit - 48)); 
  }
  if (isNegative) return current.toInt() * -1; else return current.toInt();
}
