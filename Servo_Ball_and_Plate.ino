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
#define SERVOMIN  170 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  300 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMID  235 // this is the 'middle' pulse length count (out of 4096)

//Offsets for leveling the plate: Not all servos were at the same angle.
#define XNOFFSET  -12 
#define YPOFFSET  -8


// Servo motor pin numbers 
uint8_t X_Pos_servo = 2;
uint8_t X_Neg_servo = 0;
uint8_t Y_Pos_servo = 3;
uint8_t Y_Neg_servo = 1;

// X_angle is a user input from the Serial Monitor
float X_angle;

// The pulselength variables control the angle of the servos. The servos have a deadband of 2-4 microseconds.
int pulselength;
int opposite_pulselength;



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
  
}


void loop() {

  if (Serial.available() > 0) 
  {
    
    // get the desired X angle from the Serial Monitor input
    X_angle = Serial.parseFloat();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(X_angle);
    
  }

  // Map the X angle to a corresponding pulse length
  pulselength = map(X_angle, -15.0, 15.0, SERVOMIN, SERVOMAX);

  // Map the opposite X servo's pulse length - this allows the servos to move in opposite directions simultaneously. 
  opposite_pulselength = map(pulselength, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN);

  // Checking values:
  Serial.print("Pulslength: ");
  Serial.println(pulselength);  
  
  Serial.print("Opposite Pulslength: ");
  Serial.println(opposite_pulselength);

  // Move the X axis servos:
  pwm.setPWM(X_Pos_servo, 0, pulselength);
  pwm.setPWM(X_Neg_servo, 0, opposite_pulselength + XNOFFSET);

  delay(500);
  
}
