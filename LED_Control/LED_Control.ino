/*
 * Make some funny LED flashing
*/

const int firstLED  =  1; // red
const int secondLED =  0; // blue
const int thirdLED  =  2; // green
const int forthLED  =  3; // green
const int fifthLED  =  4; // unused
const int ledPin    = 13;
const int maxBrightness = 1024;

unsigned long currenttime;
unsigned long deltaT;
unsigned long offtimered;
unsigned long offtimegreen;
unsigned long offtimeblue;
int stategreen = 1;         // 1 ramp up, 3 stay up, 2 ramp down
int stategreencount = 0;
int brightnessgreen = 0; // how bright the LED is
int brightnessred =    0;      // how bright the LED is
int brightnessblue =   0;     // how bright the LED is
int fadeAmountred   =  0;   //
int fadeAmountgreen =  4;   //
int fadeAmountblue  =  0;   // no change in blue, will stay off


int fadeAmount = 1;    // how many points to fade the LED by
const int interval=10000; // 10ms

void setup()   {                
  pinMode(firstLED, OUTPUT);
  pinMode(secondLED, OUTPUT);
  pinMode(thirdLED, OUTPUT);
  pinMode(forthLED, OUTPUT);
  pinMode(fifthLED, OUTPUT);
  pinMode(ledPin, OUTPUT);  
  digitalWrite(firstLED, LOW);
  digitalWrite(secondLED, LOW);
  digitalWrite(thirdLED, LOW);
  digitalWrite(forthLED, LOW);
  digitalWrite(fifthLED, LOW);
  digitalWrite(ledPin, LOW);
  currenttime = micros();
  //Serial.begin(9600);
}

void loop()   //Its lit.                  
{
  //* Make some kickass light flashing show
  deltaT = micros() - currenttime;
  if ( deltaT >= offtimegreen) {
       digitalWrite(thirdLED, HIGH);
       digitalWrite(forthLED, HIGH);
       digitalWrite(ledPin, HIGH);
  }
  
  if (deltaT > 1000) {
    
    if (stategreen == 1) {
      brightnessgreen = brightnessgreen + fadeAmountgreen;
      if (brightnessgreen >= maxBrightness) {
        stategreen = 3;
      }
    }
    if (stategreen == 2) {
      brightnessgreen = brightnessgreen - fadeAmountgreen;
      if (brightnessgreen <= 0) {
        stategreen = 1;
      }
    }
    if (stategreen == 3) {
      stategreencount= stategreencount + 1;
      if (stategreencount >= 500) {
        stategreen = 2;
        stategreencount = 0;
      }
    }
    
    offtimegreen = interval - ((brightnessgreen * interval) / maxBrightness);
    // offtimered   = interval - ((brightnessred   * interval) / maxBrightness);
    // offtimeblue  = interval - ((brightnessblue  * interval) / maxBrightness);
    
    //digitalWrite(firstLED,  LOW);
    //digitalWrite(secondLED, LOW);
    digitalWrite(thirdLED,  LOW);
    digitalWrite(forthLED,  LOW);
    //digitalWrite(fifthLED,  LOW);
    digitalWrite(ledPin,    LOW);
    //Serial.print(brightnessred);
    //Serial.print(' ');
    //Serial.println(brightnessgreen);
    
    currenttime = micros(); 
  }
}

