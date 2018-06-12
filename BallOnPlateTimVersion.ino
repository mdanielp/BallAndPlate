/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

double SERVOMIN = 170; // this is the 'minimum' pulse length count (out of 4096)
double SERVOMAX = 300; // this is the 'maximum' pulse length count (out of 4096)
double SERVOMID = 235; // this is the 'middle' pulse length count (out of 4096)

//Offsets for leveling the plate: Not all servos were at the same angle.
#define XNOFFSET  -18 
#define YPOFFSET  -12

// X and Y are the ball location on the touchscreen
int x,y = 0;


// Servo motor pin numbers 
uint8_t X_Pos_servo = 1;
uint8_t X_Neg_servo = 0;
uint8_t Y_Pos_servo = 2;
uint8_t Y_Neg_servo = 3;

// X_angle is a user input from the Serial Monitor
float X_angle;

// The pulselength variables control the angle of the servos. The servos have a deadband of 2-4 microseconds.
int x_pulselength;
int y_pulselength;
int x_opposite_pulselength;
int y_opposite_pulselength;

// These are our PID tuning gains. I turned off the derivative for now. It adds jitter, so we need to filter it.
float xKp = 0.13; // 0.15 was good starting point                                                      
float xKi = 0.005;  // 0.005                                                      
float xKd = 0.025;   // 0.03

float yKp = 0.13;                                                       
float yKi = 0.005;                                                      
float yKd = 0.025;

// PID parameters
double xSetpoint, xInput, xOutput; //for X
double ySetpoint, yInput, yOutput; //for Y

// These are our PID's for X and Y
PID xPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, REVERSE);
PID yPID(&yInput, &yOutput, &ySetpoint,yKp,yKi,yKd, REVERSE);

// PID sampling time
int Ts = 50; 



int readY()
{
  // Get the raw X position from the touchscreen by applying a voltage potential 
  // from one side to the opposite side
  int xr = 0;
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(0, INPUT);     //  Sense pin
  digitalWrite(6, LOW);  // set pin 6 to GND
  digitalWrite(7, LOW);  // set pin 7 to GND
  digitalWrite(8, HIGH); // set pin 8 to 5V
  digitalWrite(9, HIGH); // set pin 9 to 5V
  delay(5);
  xr = analogRead(0);
  return xr;
}

int readX()
{
  int yr = 0;
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(0, INPUT);      // Sense pin
  digitalWrite(6, LOW);   // set pin 6 to GND
  digitalWrite(7, HIGH);  // set pin 7 to 5V
  digitalWrite(8, LOW);   // set pin 8 to GND
  digitalWrite(9, HIGH);  // set pin 9 to 5V
  delay(5);
  yr = analogRead(0);
  return yr;
}


void setup() {
  Serial.begin(9600);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
  X_angle = 0;

  // Start each servo at the middle of the range of motion
  pwm.setPWM(Y_Pos_servo, 0, SERVOMID + YPOFFSET); // Offset to help with leveling
  pwm.setPWM(Y_Neg_servo, 0, SERVOMID);
  pwm.setPWM(X_Pos_servo, 0, SERVOMID);
  pwm.setPWM(X_Neg_servo, 0, SERVOMID + XNOFFSET); // Offset to help with leveling


  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-15, 15);
  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-15, 15);
  // TIME SAMPLE
  yPID.SetSampleTime(Ts); 
  xPID.SetSampleTime(Ts);  

  // Drive the ball to 0,0. (The origin (0,0) is where we want the ball to stay)
  xSetpoint = 0;
  ySetpoint = 0;
  
}


void loop() {

  // Get the location of the ball (raw data)
  x = readX();
  y = readY();
  
  // Convert the raw x and y data to millimeters (mm) by mapping the raw data to the touchscreen dimensions
  // These values will be the inputs to our PID's
  xInput = map(y,264,730,-123,123); // Map these according to our touchscreen dimensions
  yInput = map(x,262,732,-93,93); // 271.27 x 205.74 or 248.87 x 187.40  

  xInput = xInput - 4;
  yInput = yInput + 6;

  // Compute the output angle/pulselength using the PID's
  xPID.Compute();  //action control X compute
  yPID.Compute();  //action control  Y compute

  x_pulselength = map(xOutput, -15.0, 15.0, SERVOMIN, SERVOMAX);
  y_pulselength = map(yOutput, -15.0, 15.0, SERVOMIN, SERVOMAX);
  x_opposite_pulselength = map(x_pulselength, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN);
  y_opposite_pulselength = map(y_pulselength, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN);


  
  pwm.setPWM(X_Pos_servo, 0, x_pulselength);
  pwm.setPWM(X_Neg_servo, 0, x_opposite_pulselength + XNOFFSET);
  pwm.setPWM(Y_Pos_servo, 0, y_pulselength + YPOFFSET); // Offset to help with leveling
  pwm.setPWM(Y_Neg_servo, 0, y_opposite_pulselength);

  
  Serial.print(" xInput = ");
  Serial.print(xInput);
  Serial.print("         yInput = ");
  Serial.print(yInput);
  Serial.print("\n");

  Serial.print(" xOutput = ");
  Serial.print(xOutput);
  Serial.print("         yOutput = ");
  Serial.print(yOutput);
  Serial.print("\n");
  //delay(1000);
}
