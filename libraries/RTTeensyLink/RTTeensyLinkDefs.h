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

#ifndef _RTTEENSYLINKDEFS_H
#define _RTTEENSYLINKDEFS_H

//	Some general purpose typedefs - used especially for transferring values greater than
//	8 bits across the link and avoids endian issues. Assumes processor has 32 bit ints!

typedef	unsigned char RTTEENSYLINK_UC2[2];                  // an array of two unsigned chars
typedef	unsigned char RTTEENSYLINK_UC4[4];                  // an array of four unsigned chars

//------------------------------------------------------------------------------------------------------
//
//	Frame level defs and structure

#define	RTTEENSYLINK_FRAME_MAX_LEN        64                // maximum possible length of a frame
#define	RTTEENSYLINK_FRAME_HEADER_LEN     4                 // 4 bytes in frame header (must correspond with the structure below!)
#define	RTTEENSYLINK_MESSAGE_HEADER_LEN   4                 // 4 bytes in message header (must correspond with the structure below!)
#define	RTTEENSYLINK_MESSAGE_MAX_LEN      (RTTEENSYLINK_FRAME_MAX_LEN - RTTEENSYLINK_FRAME_HEADER_LEN)    // max length of message
#define	RTTEENSYLINK_DATA_MAX_LEN         (RTTEENSYLINK_MESSAGE_MAX_LEN - RTTEENSYLINK_MESSAGE_HEADER_LEN)// max length of data field

#define	RTTEENSYLINK_MESSAGE_SYNC0        0xAA
#define	RTTEENSYLINK_MESSAGE_SYNC1        0x55

#define	RTTEENSYLINK_MY_ADDRESS           0                 // the subsystem address for local processing
#define	RTTEENSYLINK_BROADCAST_ADDRESS    0xffff            // the subsystem address for all subsystems
#define	RTTEENSYLINK_ADDRESSES            0x1000            // number of addresses

//  RTTEENSYLINK_MESSAGE is carried in the RTTEENSYLINK_FRAME
//
//  The messageAddress field allows subsystems to be daisy-chained. Valid addresses are 0 to 65534.
//  Address 65535 is a broadcast and goes to all subsystems.
//  Every message has the messageType and messageParam bytes but there can be from 0 to 56 bytes of data

typedef struct
{
    RTTEENSYLINK_UC2 messageAddress;                        // subsystem message address
    unsigned char messageType;                              // message type code
    unsigned char messageParam;                             // an optional parameter to the message type
    unsigned char data[RTTEENSYLINK_DATA_MAX_LEN];          // the actual data! Length is computed from messageLength.
} RTTEENSYLINK_MESSAGE;

//  RTTEENSYLINK_FRAME is the lowest level structure used across the RTTeensyLink

typedef struct
{
    unsigned char sync0;                                    // sync0 code
    unsigned char sync1;                                    // sync1 code
    unsigned char messageLength;                            // the length of the message in the message field - between 4 and 60 bytes
    unsigned char frameChecksum;                            // checksum for frame
    RTTEENSYLINK_MESSAGE message;                           // the actual message
} RTTEENSYLINK_FRAME;

//  RTTEENSYLINK_RXFRAME is a type that is used to reassemble a frame from a stream of bytes in conjunction with reassemble()

typedef struct
{
    RTTEENSYLINK_FRAME *frameBuffer;                        // the frame buffer pointer
    int length;                                             // current length of frame
    int bytesLeft;                                          // number of bytes needed to complete
    bool complete;                                          // true if frame is complete and correct (as far as checksum goes)
} RTTEENSYLINK_RXFRAME;

//  Message types

//  RTTEENSYLINK_MESSAGE_POLL
//
//  The host should poll the RTTeensyLink at every RTTEENSYLINK_POLL_INTERVAL.
//  The subsystem will respond by echoing the poll message as received.

#define	RTTEENSYLINK_MESSAGE_POLL         0                   // poll message

//  RTTEENSYLINK_MESSAGE_IDENTIFY
//
//  The host can send this message to request an identity string from the subsystem.
//  Only the messageType field is used in the request host -> subsystem. The subsystem
//  responds with an identity string in the data field.

#define	RTTEENSYLINK_MESSAGE_IDENTITY     1                   // identity message

//  RTTEENSYLINK_MESSAGE_DEBUG
//
//  This can be used to send a debug message up to the host. The data field contains a debug message

#define	RTTEENSYLINK_MESSAGE_DEBUG        2                   // debug message

//  RTTEENSYLINK_MESSAGE_INFO
//
//  This can be used to send an info message up to the host. The data field contains the message

#define	RTTEENSYLINK_MESSAGE_INFO         3                   // info message

//  RTTEENSYLINK_MESSAGE_ERROR
//
//  This code is returned by the subsystem if it received a message with an illegal message type
//  The first byte of the data is the error code. The rest of the data field depends on the error.

#define	RTTEENSYLINK_MESSAGE_ERROR        4                   // illegal message type response

//  RTTEENSYLINK_MESSAGE_ECHO
//
//  This message can be used to test link performance. The addressed subsystem just returns
//  the entire message to the host.

#define	RTTEENSYLINK_MESSAGE_ECHO         5                   // echo message

//  RTTEENSYLINK_MESSAGE_CUSTOM
//
//  This is the first message code that should be used for custom messages 16-255 are available.

#define	RTTEENSYLINK_MESSAGE_CUSTOM       16                  // start of custom messages

//  RTTeensyLink response codes

#define	RTTEENSYLINK_RESPONSE_OK                      0       // means things worked
#define	RTTEENSYLINK_RESPONSE_ILLEGAL_COMMAND         1       // not a supported message type, data[1] has offending type


#endif  // _RTTEENSYLINKDEFS_H
