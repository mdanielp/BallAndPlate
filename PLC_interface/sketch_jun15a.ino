int ledPinY = 10;      // LED connected to digital pin 9
int ledPinX = 11;      // LED connected to digital pin 9
int analogPinY = 0;   // potentiometer connected to analog pin 3
int analogPinX = 1;   // potentiometer connected to analog pin 3

int val = 0;         // variable to store the read value

void setup()
{
  pinMode(ledPinY, OUTPUT);   // sets the pin as output
  pinMode(ledPinX, OUTPUT);   // sets the pin as output
  pinMode(analogPinY, INPUT);   // sets the pin as output
  pinMode(analogPinX, INPUT);   // sets the pin as output
  Serial.begin(9600);
}

void loop()
{
  int valY = analogRead(analogPinY);   // read the input pin
  int valX = analogRead(analogPinX);   // read the input pin
  
  
  analogWriteResolution(12);
  analogWrite(ledPinY, 4000);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  analogWrite(ledPinX, 1000);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  Serial.print("\n Y");
  Serial.print(valY);   
  Serial.print("\n X");
  Serial.print(valX);
  
}

/*
Uses a for loop to print numbers in various formats.
*/
/*void setup() {
  Serial.begin(115200);      // open the serial port at 9600 bps
}

void loop() {
  // print labels
  Serial.print("NO FORMAT");  // prints a label
  Serial.print("\t");      // prints a tab

  Serial.print("DEC");
  Serial.print("\t");

  Serial.print("HEX");
  Serial.print("\t");

  Serial.print("OCT");
  Serial.print("\t");

  Serial.print("BIN");
  Serial.println();        // carriage return after the last label

  for (int x = 0; x < 64; x++) { // only part of the ASCII chart, change to suit
    // print it out in many formats:
    Serial.print(x);       // print as an ASCII-encoded decimal - same as "DEC"
    Serial.print("\t\t");  // prints two tabs to accomodate the label lenght

    Serial.print(x, DEC);  // print as an ASCII-encoded decimal
    Serial.print("\t");    // prints a tab

    Serial.print(x, HEX);  // print as an ASCII-encoded hexadecimal
    Serial.print("\t");    // prints a tab

    Serial.print(x, OCT);  // print as an ASCII-encoded octal
    Serial.print("\t");    // prints a tab

    Serial.println(x, BIN);  // print as an ASCII-encoded binary
    // then adds the carriage return with "println"
    delay(200);            // delay 200 milliseconds
  }
  Serial.println();        // prints another carriage return
}
*/
