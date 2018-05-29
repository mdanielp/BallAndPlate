/**********************************************
 * Touchscreen test code
 * 
 * Author: Tyler Sutton
 *********************************************/

int x,y = 0;

void setup() {
  Serial.begin(9600);
}

int readX()
{
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

void loop() { 
  
    x = readX();
    y = readY();

//  newY = map(y,75,840,0,650) // Map these according to our touchscreen dimensions
//  int newX = map(x,100,780,0,500); // 271.27 x 205.74 or 248.87 x 187.40
//
//  if((x<0)||(y<0))
//  {
//    newX = 0;
//    // newY = 0;
//  }

  Serial.print(" x = ");
  Serial.print(x);
  Serial.print("         y = ");
  Serial.print(y);
  Serial.print("\n");
  delay(50);
}













