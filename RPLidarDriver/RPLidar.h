/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 * 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include "Arduino.h"
#include "inc/rptypes.h"
#include "inc/rplidar_cmd.h"

struct RPLidarMeasurement
{
    float distance;
    float angle;
    uint8_t quality;
    bool  startBit;
};

class RPLidar
{
public:
    enum {
        RPLIDAR_SERIAL_BAUDRATE = 115200,  
        RPLIDAR_DEFAULT_TIMEOUT = 500,
    };
    
    RPLidar();  
    ~RPLidar();

    // open the given serial interface and try to connect to the RPLIDAR
    bool begin(HardwareSerial &serialobj);

    // close the currently opened serial interface
    void end();
  
    // check whether the serial interface is opened
    bool isOpen(); 

    // ask the RPLIDAR for its health info
    u_result getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    
    // ask the RPLIDAR for its device info like the serial number
    u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);

    // stop the measurement operation
    u_result stop();

    // start the measurement operation
    u_result startScan(bool force = false, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);

    // wait for one sample point to arrive
    u_result waitPoint(_u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    
    // retrieve currently received sample point
    
    const RPLidarMeasurement & getCurrentPoint()
    {
        return _currentMeasurement;
    }

protected:
    u_result _sendCommand(_u8 cmd, const void * payload, size_t payloadsize);
    u_result _waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout);

protected:
    HardwareSerial * _bined_serialdev;  
    RPLidarMeasurement _currentMeasurement;
};
