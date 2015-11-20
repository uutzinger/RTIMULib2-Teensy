////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTTeensyLink
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


#include <string.h>
#include "RTTeensyLinkEEPROM.h"
#include <Arduino.h>

//  The global config structure

RTTEENSYLINK_EEPROM RTTeensyLinkConfig;

bool RTTeensyLinkEEPROMValid()
{
    RTTeensyLinkEEPROMRead();                              // see what it really is
    return (RTTeensyLinkConfig.sig0 == RTTEENSYLINKEEPROM_SIG0) && 
        (RTTeensyLinkConfig.sig1 == RTTEENSYLINKEEPROM_SIG1);
}

void RTTeensyLinkEEPROMDisplay()
{
    Serial.println();

    if ((RTTeensyLinkConfig.sig0 != RTTEENSYLINKEEPROM_SIG0) || 
        (RTTeensyLinkConfig.sig1 != RTTEENSYLINKEEPROM_SIG1)) {
        Serial.println("Invalid config");
        return;
    }
    Serial.print("Identity: ");
    Serial.println(RTTeensyLinkConfig.identity);

}

void RTTeensyLinkEEPROMDefault()
{
    RTTeensyLinkConfig.sig0 = RTTEENSYLINKEEPROM_SIG0;       // set to valid signature
    RTTeensyLinkConfig.sig1 = RTTEENSYLINKEEPROM_SIG1;                        
    strcpy(RTTeensyLinkConfig.identity, "RTTeensyLink_Teensy");

    RTTeensyLinkEEPROMWrite();
}

void RTTeensyLinkEEPROMRead()
{
    unsigned char *data;

    data = (unsigned char *)&RTTeensyLinkConfig;

    for (int i = 0; i < (int)sizeof(RTTEENSYLINK_EEPROM); i++)
        *data++ = EEPROM.read(i + RTTEENSYLINK_EEPROM_OFFSET);
}

void RTTeensyLinkEEPROMWrite()
{
    unsigned char *data;

    data = (unsigned char *)&RTTeensyLinkConfig;

    for (int i = 0; i < (int)sizeof(RTTEENSYLINK_EEPROM); i++)
        EEPROM.write(i + RTTEENSYLINK_EEPROM_OFFSET, *data++);
}
