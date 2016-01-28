/**
Visualize a cube which will assumes the orientation described
in a quaternion coming from the serial port.

INSTRUCTIONS: 
This program has to be run when you have the FreeIMU_serial
program running on your Arduino and the Arduino connected to your PC.
Remember to set the serialPort variable below to point to the name the
Arduino serial port has in your system. You can get the port using the
Arduino IDE from Tools->Serial Port: the selected entry is what you have
to use as serialPort variable.


Copyright (C) 2011-2012 Fabio Varesano - http://www.varesano.net/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


import processing.serial.*;
import processing.opengl.*;

Serial myPort;  // Create object from Serial class

final String serialPort = "COM6"; // replace this with your serial port. On windows you will need something like "COM1".
int BAUDRATE = 115200;

float [] q       = new float [4]; // quaternion  
float [] eul     = new float [3]; // pitch, roll, yaw
float [] heul    = new float [3]; // homing pitch, roll, yaw 
float heading    = 0.0;
float [] acc     = new float [3]; // acceleration
float [] gyr     = new float [3]; // gyration
float [] mag     = new float [3]; // compass
float [] a       = new float [3]; // residual acceleration
float [] wa      = new float [3]; // residula acceleration in world coordinate system
float [] wv      = new float [3]; // velocity
float [] wp      = new float [3]; // position
int  dt          = 0;             // sample interval 
int tim          = 0;             // sample interval 
int tim_previous = 0;             // sample interval 
int mo           = 0;             // motion

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 1000, VIEW_SIZE_Y = 600;

final int burst = 32;
int count = 0;

// Processing: x to left, z to observer, y down
// Sensor: x to left, z down, y to observer
// xp=xsens (40 wide)
// yp=zsens (10 high)
// zp=ysens (60 longh)

PImage Top, TopInv, Bottom, BottomInv, Right, RightInv, Left, LeftInv, Front, FrontInv, Back, BackInv;

final int box_x2=20;
final int box_y2=5;
final int box_z2=30;

final float PI = 3.1415926535;
final float RAD_TO_DEGREE = (180.0f/PI);

void myDelay(int time) { // millis 
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

void setup() 
{
  size(VIEW_SIZE_X, VIEW_SIZE_Y, OPENGL);
  println("Opening Serial Port..");
  myPort = new Serial(this, serialPort, BAUDRATE);
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
  
  println("Waiting IMU..");
  // myDelay(2000);
  println("Clear serial port");
  myPort.clear();
  println("Buffer serial port");
  myPort.bufferUntil('\n');

  //load cube images
  textureMode(NORMAL);
  Top = loadImage("Top.png");
  TopInv = loadImage("TopInv.png");
  Bottom = loadImage("Bottom.png");
  BottomInv = loadImage("BottomInv.png");
  
  heul[2]=0;
  heul[1]=0;
  heul[0]=0;
  
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];
  
  if(inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }
      
  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}

void serialEvent(Serial p) {
  if(p.available() >= 62) {                    // 6 * 9 char per float (8 char per float plus comma)
    // 34B048C0,6D8104BB,3735CDBE,F86EF7BB,9C02533D,85002141,4E20,0
    String inputString = p.readStringUntil('\n');
    // print(inputString);
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      // print(inputStringArr.length);
      if(inputStringArr.length == 9) {           // 8 elements min
        eul[0]  = decodeFloat(inputStringArr[0]);
        eul[1]  = decodeFloat(inputStringArr[1]);
        eul[2]  = decodeFloat(inputStringArr[2]);
        a[0]    = decodeFloat(inputStringArr[3]);
        a[1]    = decodeFloat(inputStringArr[4]);
        a[2]    = decodeFloat(inputStringArr[5]);
        tim     = unhex(inputStringArr[6]);
        mo      = Integer.parseInt(inputStringArr[7]);
      }
      else if(inputStringArr.length >= 31) {    
        acc[0]  = decodeFloat(inputStringArr[0]);
        acc[1]  = decodeFloat(inputStringArr[1]);
        acc[2]  = decodeFloat(inputStringArr[2]);
        gyr[0]  = decodeFloat(inputStringArr[3]);
        gyr[1]  = decodeFloat(inputStringArr[4]);
        gyr[2]  = decodeFloat(inputStringArr[5]);
        mag[0]  = decodeFloat(inputStringArr[6]);
        mag[1]  = decodeFloat(inputStringArr[7]);
        mag[2]  = decodeFloat(inputStringArr[8]);
        q[0]    = decodeFloat(inputStringArr[9]);
        q[1]    = decodeFloat(inputStringArr[10]);
        q[2]    = decodeFloat(inputStringArr[11]);
        q[3]    = decodeFloat(inputStringArr[12]);
        eul[0]  = decodeFloat(inputStringArr[13]);
        eul[1]  = decodeFloat(inputStringArr[14]);
        eul[2]  = decodeFloat(inputStringArr[15]);
        a[0]    = decodeFloat(inputStringArr[16]);
        a[1]    = decodeFloat(inputStringArr[17]);
        a[2]    = decodeFloat(inputStringArr[18]);
        heading = decodeFloat(inputStringArr[19]);
        wa[0]    = decodeFloat(inputStringArr[20]);
        wa[1]    = decodeFloat(inputStringArr[21]);
        wa[2]    = decodeFloat(inputStringArr[22]);
        wv[0]    = decodeFloat(inputStringArr[23]);
        wv[1]    = decodeFloat(inputStringArr[24]);
        wv[2]    = decodeFloat(inputStringArr[25]);
        wp[0]    = decodeFloat(inputStringArr[26]);
        wp[1]    = decodeFloat(inputStringArr[27]);
        wp[2]    = decodeFloat(inputStringArr[28]);
        tim     = unhex(inputStringArr[29]);
        mo      = Integer.parseInt(inputStringArr[30]);
      }
      dt = tim - tim_previous;
      tim_previous = tim;
    }
  }
}


void buildBoxShape() {
  //box(60, 10, 40);
  //noStroke();
  
  //Z+ (to the drawing area)
  beginShape(QUADS);
  fill(#00ff00);
  vertex(-box_x2, -box_y2, box_z2);
  vertex(box_x2, -box_y2, box_z2);
  vertex(box_x2, box_y2, box_z2);
  vertex(-box_x2, box_y2, box_z2);
  endShape();

  //Z-
  beginShape(QUADS);
  fill(#0000ff);
  vertex(-box_x2, -box_y2, -box_z2);
  vertex(box_x2, -box_y2, -box_z2);
  vertex(box_x2, box_y2, -box_z2);
  vertex(-box_x2, box_y2, -box_z2);
  endShape();
  
  //X-
  beginShape(QUADS);
  fill(#ff0000);
  vertex(-box_x2, -box_y2, -box_z2);
  vertex(-box_x2, -box_y2, box_z2);
  vertex(-box_x2, box_y2, box_z2);
  vertex(-box_x2, box_y2, -box_z2);
  endShape();
  
  //X+
  beginShape(QUADS);
  fill(#ffff00);
  vertex(box_x2, -box_y2, -box_z2);
  vertex(box_x2, -box_y2, box_z2);
  vertex(box_x2, box_y2, box_z2);
  vertex(box_x2, box_y2, -box_z2);
  endShape();
  
  //Y-
  beginShape(QUADS);
  //fill(#ff00ff);
  texture(Bottom);
  vertex(-box_x2, -box_y2, -box_z2, 1, 1);
  vertex(box_x2, -box_y2, -box_z2, 1, 0);
  vertex(box_x2, -box_y2, box_z2, 0, 0);
  vertex(-box_x2, -box_y2, box_z2, 0, 1);
  endShape();
  
  //Y+
  beginShape(QUADS);
  texture(Top);
  // fill(#00ffff);
  vertex(-box_x2, box_y2, -box_z2, 1, 1);
  vertex(box_x2, box_y2, -box_z2, 1, 0);
  vertex(box_x2, box_y2, box_z2, 0, 0);
  vertex(-box_x2, box_y2, box_z2, 0, 1);
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
    scale(5,5,5);
 
    // Goal is to have North pointing into monitor
    // World FreeIMU
    //  X points North
    //  Y points to West
    //  Z points to Sky 
    // World RTIMU (but use FreeIMU directions in RTIMU)
    //  X points North
    //  Y points to East
    //  Z points to Earth 
    // Processing
    //  X points to the right
    //  Y points down
    //  Z points forward to the monitor
    
    // World FreeIMU to Procesing:
    // Rotation phi(roll)    around X_w corresponds to rotation : +phi   around Z_m
    // Rotation theta(pitch) around Y_w corresponds to rotation : -theta around X_m
    // Rotation psi(yaw)     around Z_w corresponds to rotation : -psi   around Y_m
    
    // RTIMU EUL:[R,P,Y] to FIMU: EUL[Y,P,R]
    
    // FreeIMU:
    // a demonstration of the following is at 
    // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    // rotateZ(-Euler[2]); // phi - roll
    // rotateX(-Euler[1]); // theta - pitch
    // rotateY(-Euler[0]); // psi - yaw
    
    // Adapted:
    rotateZ(+(eul[0]-heul[0])); // phi - roll
    rotateX(+(eul[1]-heul[1])); // theta - pitch
    rotateY(-(eul[2]-heul[2])); // psi - yaw

    buildBoxShape();
    
  popMatrix();
}


void draw() { // Main Loop
  background(#000000);
  fill(#ffffff);
  
  textFont(font, 20);
  textAlign(LEFT, TOP);
  text("Tait- Bryan X:North Z:Earth\nRoll (phi)   : " + String.format("%+.1f",degrees(eul[0])) + "deg \nPitch (theta): " + String.format("%+.1f",degrees(eul[1])) + "deg \nYaw (psi)    : " + String.format("%+.1f",degrees(eul[2])) + "deg", 20, 20);
  text("Q   - " + String.format("%+.3f",sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]))   + "\n"     + String.format("%+.3f",q[0])                 + "\n"           + String.format("%+.3f",q[1])                 + "\n"           + String.format("%+.3f",q[2])   + "\n" + String.format("%+.3f",q[3]), 20, 120);
  text("Acc - " + String.format("%+.3f",sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])) + "\nX : " + String.format("%+.3f",acc[0])               + "m/s2 \nY : "  + String.format("%+.3f",acc[1])               + "m/s2 \nZ : "  + String.format("%+.3f",acc[2]) + "m/s2", 20, 220);
  text("Gyr - " + String.format("%+.3f",sqrt(gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2])*RAD_TO_DEGREE) + "\nX : " + String.format("%+.3f",gyr[0]*RAD_TO_DEGREE) + "deg/s \nY : " + String.format("%+.3f",gyr[1]*RAD_TO_DEGREE) + "deg/s \nZ : " + String.format("%+.3f",gyr[2]*RAD_TO_DEGREE) + "deg/s", 20, 300);
  text("Mag - " + String.format("%+.3f",sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2])) + "\nX : " + String.format("%+.3f",mag[0])               + "muT \nY : "   + String.format("%+.3f",mag[1])               + "muT \nZ : "   + String.format("%+.3f",mag[2]) + "muT", 20, 380);
  
  text("Residual - " + String.format("%+.3f",sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])) + "\nX : " + String.format("%+.3f",a[0]) + "m/s2 \nY : " +   String.format("%+.3f",a[1]) + "m/s2 \nZ : " +   String.format("%+.3f",a[2]) + "m/s2", 700, 20);
  text("Heading: " + String.format("%+.1f",heading*RAD_TO_DEGREE) + "deg" , 700, 100);
  text("dt: " + str( dt/1000) + "ms\n", 700, 120);
  String smo = str(mo);
  text("Motion: " + mo, 700, 140);

  text("Accel    - " + String.format("%+.3f",sqrt(wa[0]*wa[0]+wa[1]*wa[1]+wa[2]*wa[2])) + "m/s2\nX : " + String.format("%+.3f",wa[0]) + "m/s2 \nY : " +   String.format("%+.3f",wa[1]) + "m/s2 \nZ : " +   String.format("%+.3f",wa[2]) + "m/s2", 700, 180);
  text("Velocity - " + String.format("%+.3f",sqrt(wv[0]*wv[0]+wv[1]*wv[1]+wv[2]*wv[2])) + "m/s\nX : "  + String.format("%+.3f",wv[0]) + "m/s \nY : "  +   String.format("%+.3f",wv[1]) + "m/s \nZ : "  +   String.format("%+.3f",wv[2]) + "m/s", 700, 260);
  text("Position - " + String.format("%+.3f",sqrt(wp[0]*wp[0]+wp[1]*wp[1]+wp[2]*wp[2])) + "m\nX : "    + String.format("%+.3f",wp[0]) + "m \nY : "    +   String.format("%+.3f",wp[1]) + "m \nZ : "    +   String.format("%+.3f",wp[2]) + "m", 700, 340);

  //println(smo.length());
  //if (smo.length() == 6) {  text("Mot: " +     ((int) mo) +"\n", 700, 440);}
  //if (smo.length() == 5) {  text("Mot: 0" +    ((int) mo) +"\n", 700, 440);}
  //if (smo.length() == 4) {  text("Mot: 00" +   ((int) mo) +"\n", 700, 440);}
  //if (smo.length() == 3) {  text("Mot: 000" +  ((int) mo) +"\n", 700, 440);}

  drawCube();
}

void keyPressed() {
  if(key == 'm') {
    //turn off Magnetometer
    println("pressed m, turn off magnetometer");
    myPort.write("m");
  }
  else if(key == 'M') {
    println("pressed M, turn on magnetometer");
   //turn on Magnetometer
    myPort.write("M");
  }
  else if (key == 's') { 
    // turn off streaming
    println("pressed s, tunr off streaming");
    myPort.write("s");
  }
  else if (key == 'S') { 
    // turn on  streaming
    println("pressed S, turn on streaming");
    myPort.write("S");
  }
  else if (key == 'P') { 
    // reset position
    println("pressed P, zero position");
    myPort.write("P");
  }
  else if (key == 'p') { 
    // reset position
    println("pressed P, zero position");
    myPort.write("P");
  }
  else if (key == 'H') { 
    // reset position
    println("pressed H, adjusting pointing direction");
    heul[2]=eul[2];
    heul[1]=eul[1];
    heul[0]=eul[0];
  }
  else if (key == 'h') { 
    // reset position
    println("pressed h, disengage pointing direction");
    heul[2]=0;
    heul[1]=0;
    heul[0]=0;
  }
}

