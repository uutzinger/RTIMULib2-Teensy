////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  Copyright (c) 2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <SD.h>
#include <SPI.h>

#define SERIAL_PORT_SPEED    115200
#define SD_SELECT_PIN    10
#define INI_FILE_NAME    "RTIMULib.ini"

void setup()
{
    Serial.begin(SERIAL_PORT_SPEED);
    while (!Serial) {
        ; // wait for serial port to connect. 
    }
    pinMode(SD_SELECT_PIN, OUTPUT);
    
    if (!SD.begin(SD_SELECT_PIN)) {
         Serial.println("Failed to find SD card");
         return;   
    }
    
    if (!SD.exists(INI_FILE_NAME)) {
        Serial.println("RTIMULib.ini not found");
        return;
    }
    
    SD.remove(INI_FILE_NAME);
    
    if (!SD.exists(INI_FILE_NAME)) {
        Serial.println("RTIMULib.ini deleted successfully");
        return;
    }
    Serial.println("RTIMULib.ini was not deleted");
}

void loop()
{  
}

