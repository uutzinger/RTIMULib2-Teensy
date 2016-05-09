/*
 * Make some funny LED flashing
*/

const int firstLED  =  0; // green
const int secondLED =  1; // red
const int thirdLED  =  2; // blue
const int forthLED  =  3; // green
const int fifthLED  =  4; // not used
const int ledPin    = 13; // built in LED from teensy

void setup()   {                
  pinMode(firstLED, OUTPUT);
  pinMode(secondLED,OUTPUT);
  pinMode(thirdLED, OUTPUT);
  pinMode(forthLED, OUTPUT);
  pinMode(fifthLED, OUTPUT);
  pinMode(ledPin,   OUTPUT);
}

void loop()                     
{
    digitalWrite(firstLED,  LOW);
    digitalWrite(secondLED, LOW);
    digitalWrite(thirdLED,  LOW);
    digitalWrite(forthLED,  LOW);
    digitalWrite(fifthLED,  LOW);
    digitalWrite(ledPin,    LOW);
    delay(500);
    digitalWrite(firstLED,  HIGH);
    digitalWrite(secondLED, HIGH);
    digitalWrite(thirdLED,  HIGH);
    digitalWrite(forthLED,  HIGH);
    digitalWrite(fifthLED,  HIGH);
    digitalWrite(ledPin,    HIGH);
    delay(500);
}

