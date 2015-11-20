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

#include "RTTeensyLink.h"
#include "RTTeensyLinkEEPROM.h"
#include "RTTeensyLinkUtils.h"
#include "Arduino.h"
#include <string.h>

RTTeensyLink::RTTeensyLink()
{
}

RTTeensyLink::~RTTeensyLink()
{
}

void RTTeensyLink::begin(const char *identitySuffix)
{
    if (!RTTeensyLinkEEPROMValid())
        RTTeensyLinkEEPROMDefault();

    m_identitySuffix = identitySuffix;

    RTTeensyLinkRXFrameInit(&(m_RXFrame), &(m_RXFrameBuffer));
}


void RTTeensyLink::background()
{
    while (Serial.available()) {
        if (!RTTeensyLinkReassemble(&(m_RXFrame), Serial.read())) {
            sendDebugMessage("Reassembly error");
        } else {
            if (m_RXFrame.complete) {
                processReceivedMessage();
                RTTeensyLinkRXFrameInit(&(m_RXFrame), &(m_RXFrameBuffer));
            }
        }
    }
}

void RTTeensyLink::processReceivedMessage()
{
    RTTEENSYLINK_MESSAGE *message;                          // a pointer to the message part of the frame
    int identityLength;
    int suffixLength;

    message = &(m_RXFrameBuffer.message);                   // get the message pointer

    switch (message->messageType)
    {
        case RTTEENSYLINK_MESSAGE_POLL:
        case RTTEENSYLINK_MESSAGE_ECHO:
             RTTeensyLinkConvertIntToUC2(RTTEENSYLINK_MY_ADDRESS, message->messageAddress);
             sendFrame(&(m_RXFrameBuffer), m_RXFrameBuffer.messageLength);   // just send the frame back as received
             break;

        case RTTEENSYLINK_MESSAGE_IDENTITY:
             identityLength = strlen(RTTeensyLinkConfig.identity);
             suffixLength = strlen(m_identitySuffix);

             memcpy(message->data, RTTeensyLinkConfig.identity, identityLength + 1);    // copy in identity

             if ((identityLength + suffixLength) < RTTEENSYLINK_DATA_MAX_LEN - 1) {
                 memcpy(message->data + identityLength, m_identitySuffix, suffixLength + 1); // copy in suffix
             } else {
                 suffixLength = 0;
             }
             RTTeensyLinkConvertIntToUC2(RTTEENSYLINK_MY_ADDRESS, message->messageAddress);
             message->data[RTTEENSYLINK_DATA_MAX_LEN - 1] = 0;     // make sure zero terminated if it was truncated
             sendFrame(&(m_RXFrameBuffer), RTTEENSYLINK_MESSAGE_HEADER_LEN + identityLength + suffixLength + 1);
             break;

        default:
            if (message->messageType < RTTEENSYLINK_MESSAGE_CUSTOM) {	// illegal code
                message->data[0] = RTTEENSYLINK_RESPONSE_ILLEGAL_COMMAND;
                message->data[1] = message->messageType;        // this is the offending code
                message->messageType = RTTEENSYLINK_MESSAGE_ERROR;
                RTTeensyLinkConvertIntToUC2(RTTEENSYLINK_MY_ADDRESS, message->messageAddress);
                sendFrame(&(m_RXFrameBuffer), RTTEENSYLINK_MESSAGE_HEADER_LEN + 2);
                break;
            }
            processCustomMessage(message->messageType, message->messageParam, message->data,
                    m_RXFrameBuffer.messageLength - RTTEENSYLINK_MESSAGE_HEADER_LEN);	// see if anyone wants to process it
            break;
    }
}

void RTTeensyLink::sendDebugMessage(const char *debugMessage)
{
    RTTEENSYLINK_FRAME frame;
    int stringLength;

    stringLength = strlen(debugMessage);
    if (stringLength >= RTTEENSYLINK_DATA_MAX_LEN)
        stringLength = RTTEENSYLINK_DATA_MAX_LEN-1;
    memcpy(frame.message.data, debugMessage, stringLength);
    frame.message.data[stringLength] = 0;
    frame.message.messageType = RTTEENSYLINK_MESSAGE_DEBUG;
    RTTeensyLinkConvertIntToUC2(RTTEENSYLINK_MY_ADDRESS, frame.message.messageAddress);
    sendFrame(&frame, RTTEENSYLINK_MESSAGE_HEADER_LEN + stringLength + 1);
}

void RTTeensyLink::sendMessage(unsigned char messageType, unsigned char messageParam, unsigned char *data, int length)
{
    RTTEENSYLINK_FRAME frame;
  
    RTTeensyLinkConvertIntToUC2(RTTEENSYLINK_MY_ADDRESS, frame.message.messageAddress);
    frame.message.messageType = messageType;
    frame.message.messageParam = messageParam;

    if (length > RTTEENSYLINK_DATA_MAX_LEN)
        length = RTTEENSYLINK_DATA_MAX_LEN;
    memcpy(frame.message.data, data, length);

    sendFrame(&frame, length + RTTEENSYLINK_MESSAGE_HEADER_LEN);
}

void RTTeensyLink::sendFrame(RTTEENSYLINK_FRAME *frame, int length)
{
    frame->sync0 = RTTEENSYLINK_MESSAGE_SYNC0;
    frame->sync1 = RTTEENSYLINK_MESSAGE_SYNC1;
    frame->messageLength = length;                          // set length
    RTTeensyLinkSetChecksum(frame);                           // compute checksum
    Serial.write((unsigned char *)frame, frame->messageLength + RTTEENSYLINK_FRAME_HEADER_LEN);
}

