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

#ifndef _RTTEENSYLINKEEPROM_H
#define _RTTEENSYLINKEEPROM_H

//----------------------------------------------------------
//  Target-specific includes
//
//
//  Teensyino HAL

#include <RTTeensyLinkDefs.h>
#include <EEPROM.h>

#define RTTEENSYLINK_EEPROM_OFFSET         256              // where the config starts in EEPROM

//  RTTEENSYLINKHAL_EEPROM is the target-specific structure used to
//  store configs in EEPROM

//  Signature bytes indicating valid config

#define RTTEENSYLINKEEPROM_SIG0                  0x38          
#define RTTEENSYLINKEEPROM_SIG1                  0xc1           

typedef struct
{
    unsigned char sig0;                                     // signature byte 0
    unsigned char sig1;                                     // signature byte 1
    char identity[RTTEENSYLINK_DATA_MAX_LEN];               // identity string
} RTTEENSYLINK_EEPROM;

//  The global config structure

extern RTTEENSYLINK_EEPROM RTTeensyLinkConfig;

//  RTTeensyLinkEEPROMValid() returns true if the EEPROM contains a valid configuration,
//  false otherwise.

    bool RTTeensyLinkEEPROMValid();                         // returns true if a valid config


//  RTTeensyLinkEEPROMDisplay() displays the current configuration

    void RTTeensyLinkEEPROMDisplay();                       // display the config


//  RTTeensyLinkEEPROMDefault() writes a default config to EEPROM

    void RTTeensyLinkEEPROMDefault();                       // write and load default settings


//  RTTeensyLinkEEPROMRead() loads the EEPROM config into the RTTeensyLinkConfig
//  global structure.

    void RTTeensyLinkEEPROMRead();                          // to load the config


//  RTTeensyLinkEEPROMWrite() writes the config in the RTTeensyLinkConfig
//  global structure back to EEPROM.

    void RTTeensyLinkEEPROMWrite();                          // to write the config


#endif // _RTTEENSYLINKEEPROM_H

