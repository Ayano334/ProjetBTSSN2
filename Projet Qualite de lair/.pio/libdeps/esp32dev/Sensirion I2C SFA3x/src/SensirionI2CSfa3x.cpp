/*
 * THIS FILE IS AUTOMATICALLY GENERATED AND MUST NOT BE EDITED MANUALLY!
 *
 * I2C-Generator: 0.2.0
 * Yaml Version: 0.1.0
 * Template Version: 0.3.0
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SensirionI2CSfa3x.h"
#include "Arduino.h"
#include "SensirionCore.h"
#include <Wire.h>

#define SFA3X_I2C_ADDRESS 0x5D

SensirionI2CSfa3x::SensirionI2CSfa3x() {
}

void SensirionI2CSfa3x::begin(TwoWire& i2cBus) {
    _i2cBus = &i2cBus;
}

uint16_t SensirionI2CSfa3x::startContinuousMeasurement() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x06);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SFA3X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(1);
    return error;
}

uint16_t SensirionI2CSfa3x::stopMeasurement() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0x104);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SFA3X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(50);
    return error;
}

uint16_t SensirionI2CSfa3x::readMeasuredValues(int16_t& hcho, int16_t& humidity,
                                               int16_t& temperature) {
    uint16_t error;
    uint8_t buffer[9];
    SensirionI2CTxFrame txFrame(buffer, 9);

    error = txFrame.addCommand(0x327);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SFA3X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(5);

    SensirionI2CRxFrame rxFrame(buffer, 9);
    error = SensirionI2CCommunication::receiveFrame(SFA3X_I2C_ADDRESS, 9,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(hcho);
    error |= rxFrame.getInt16(humidity);
    error |= rxFrame.getInt16(temperature);
    return error;
}

uint16_t SensirionI2CSfa3x::getDeviceMarking(uint8_t deviceMarking[],
                                             uint8_t deviceMarkingSize) {
    uint16_t error;
    uint8_t buffer[48];
    SensirionI2CTxFrame txFrame(buffer, 48);

    error = txFrame.addCommand(0xD060);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SFA3X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(2);

    SensirionI2CRxFrame rxFrame(buffer, 48);
    error = SensirionI2CCommunication::receiveFrame(SFA3X_I2C_ADDRESS, 48,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(deviceMarking, deviceMarkingSize);
    return error;
}

uint16_t SensirionI2CSfa3x::deviceReset() {
    uint16_t error;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame(buffer, 2);

    error = txFrame.addCommand(0xD304);
    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SFA3X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(100);
    return error;
}