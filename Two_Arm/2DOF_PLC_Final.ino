/*************************************************** 
3 Degrees of Freedom (3 Motors) System Code
 ****************************************************/

#include <Wire.h>
#include <PID_v1.h>
//#include "RunningMedian.h"
#include <Adafruit_PWMServoDriver.h>

double SERVOMIN = 250; // this is the 'minimum' pulse length count (out of 1000)
double SERVOMAX = 750; // this is the 'maximum' pulse length count (out of 1000)
double SERVOMID = (SERVOMIN + SERVOMAX) / 2; // this is the 'middle' pulse length count (out of 1000)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


float x_plc_in = A10;
float y_plc_in = A11;

//Offsets for leveling the plate: Not all servos were at the same angle.
#define XNOFFSET  -32
#define YPOFFSET  -48

#define X_POSITION  10
#define Y_POSITION  11

// X and Y are the ball location on the touchscreen
double x,y = 0;

// These are our Serial Servo Motor ID tags
#define ID1   1
#define ID2   2


// These are our PID tuning gains. I turned off the derivative for now. It adds jitter, so we need to filter it.
float xKp = .6; // 0.6 was good starting point                                                     
float xKi = 0.00;  // 0.005                                                    
float xKd = 0.08;   // 0.08                        

float yKp = 0.6;                                                       
float yKi = 0.00;                                                      
float yKd = 0.08;


// Small tuning parameters for when the error is small
float sxKp = 0.035;                                                      
float sxKi = 0.002;                                                       
float sxKd = 0.012;   

float syKp = 0.035;                                                       
float syKi = 0.002;                                                      
float syKd = 0.012;

// PID parameters
double xInput, xOutput, x_angle, xOutput_filtered; //for X
double yInput, yOutput, y_angle, yOutput_filtered; //for Y

double xRaw, yRaw;





#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

//#define LOBOT_DEBUG 1  /*Debug ：print debug value*/

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  Serial.write(buf, 10);
}

void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  Serial.write(buf, 6);
}

void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  Serial.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  Serial.write(buf, 10);
}


void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  Serial.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  Serial.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}


int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (Serial.available()) {
    rxBuf = Serial.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}


int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (Serial.available())
    Serial.read();

  Serial.write(buf, 6);

  while (!Serial.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}


int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (Serial.available())
    SerialX.read();

  Serial.write(buf, 6);

  while (!Serial.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}




int readX()
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

int readY()
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
  //Serial.begin(9600);

  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  pwm.begin();
  
  pwm.setPWMFreq(1000);  // Analog servos run at ~60 Hz updates
  
  //delay(10);
  
  x_angle = 0;

  pinMode(X_POSITION, OUTPUT);
  pinMode(Y_POSITION, OUTPUT);

  //analogReadResolution(12);

 // analogWriteResolution(12);
  
}


void loop() {

  // Get the location of the ball (raw data)
  xRaw = readX();
  yRaw = readY();
  
  // Convert the raw x and y data to millimeters (mm) by mapping the raw data to the touchscreen dimensions
  // These values will be the inputs to our PID's
  x = map(xRaw,264,730,0,4096); // Map these according to our touchscreen dimensions ---- 264,730
  y = map(yRaw,262,732,0,4096); // 271.27 x 205.74 or 248.87 x 187.40  ---- 262, 732
  
  pwm.setPWM(X_POSITION, 0, x);
  pwm.setPWM(Y_POSITION, 0, y); // Offset to help with leveling

  
  //analogWrite(X_POSITION, xRaw);
  //analogWrite(Y_POSITION, yRaw);


  int plc_angle_x = analogRead(x_plc_in);
  int plc_angle_y = analogRead(y_plc_in);
  x_angle = map(plc_angle_x, 140, 900, SERVOMIN, SERVOMAX);
  y_angle = map(plc_angle_y, 230, 810, SERVOMAX, SERVOMIN);

  LobotSerialServoMove(Serial, ID1, x_angle + XNOFFSET, 0);
  LobotSerialServoMove(Serial, ID2, y_angle - YPOFFSET, 0);

  //LobotSerialServoMove(Serial, ID1, x_angle, 0);
  //LobotSerialServoMove(Serial, ID2, y_angle, 0);
  
  Serial.print(" plcx = ");
  Serial.print(plc_angle_x);
  Serial.print("         plcy = ");
  Serial.print(plc_angle_y);
  Serial.print("\n");


  

}
