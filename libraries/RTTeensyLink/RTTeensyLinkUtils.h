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

#ifndef _RTTEENSYLINKUTILS_H
#define _RTTEENSYLINKUTILS_H

#include "RTTeensyLinkDefs.h"

//  Function defs

void RTTeensyLinkRXFrameInit(RTTEENSYLINK_RXFRAME *RXFrame, RTTEENSYLINK_FRAME *frameBuffer);	// initializes RTTEENSYLINK_RXFRAME for a new frame
bool RTTeensyLinkReassemble(RTTEENSYLINK_RXFRAME *RXFrame, unsigned char data);	// adds a byte to the reassembly, returns false if error

//  Checksum utilities

void RTTeensyLinkSetChecksum(RTTEENSYLINK_FRAME *frame);        // sets the checksum field prior to transmission
bool RTTeensyLinkCheckChecksum(RTTEENSYLINK_FRAME *frame);      // checks the checksum field after reception - returns true if ok, false if error

//  Type conversion utilities

long RTTeensyLinkConvertUC4ToLong(RTTEENSYLINK_UC4 uc4);        // converts a 4 byte array to a signed long
void RTTeensyLinkConvertLongToUC4(long val, RTTEENSYLINK_UC4 uc4);  // converts a long to a four byte array
int	 RTTeensyLinkConvertUC2ToInt(RTTEENSYLINK_UC2 uc2);         // converts a 2 byte array to a signed integer
unsigned int RTTeensyLinkConvertUC2ToUInt(RTTEENSYLINK_UC2 uc2);// converts a 2 byte array to an unsigned integer
void RTTeensyLinkConvertIntToUC2(int val, RTTEENSYLINK_UC2 uc2);// converts an integer to a two byte array
void RTTeensyLinkCopyUC2(RTTEENSYLINK_UC2 destUC2, RTTEENSYLINK_UC2 sourceUC2); // copies a UC2

#endif // _RTTEENSYLINKUTILS_H
