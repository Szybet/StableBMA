#pragma once

/* MIT License
 *
 * Copyright (c) 2020 Lewis He
 * Copyright (c) 2022 for StableBMA GuruSR
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * StableBMA is a fork of:
 * bma.cpp - Arduino library for Bosch BMA423 accelerometer software library.
 * Created by Lewis He on July 27, 2020.
 * github:https://github.com/lewisxhe/BMA423_Library
 */

#include <Arduino.h>

#include "bma423.h"
#include "bma456.h"

#include "bma530.h"
#include "bma530_features.h"

enum
{
    DIRECTION_TOP_EDGE = 0,
    DIRECTION_BOTTOM_EDGE = 1,
    DIRECTION_LEFT_EDGE = 2,
    DIRECTION_RIGHT_EDGE = 3,
    DIRECTION_DISP_UP = 4,
    DIRECTION_DISP_DOWN = 5
};

struct Accel
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct AccelF
{
    float x;
    float y;
    float z;
};

class StableBMA
{

public:
    StableBMA();
    ~StableBMA();

    bool begin4(uint8_t atchyVersion, uint8_t address, uint16_t whichBma, bma4_com_fptr_t readCallBlack, bma4_com_fptr_t writeCallBlack);
    bool begin5(uint8_t atchyVersion, uint8_t address, uint16_t whichBma, bma5_read_fptr_t readCallBlack, bma5_write_fptr_t writeCallBlack);

    void softReset();
    void shutDown();
    void wakeUp();
    // True if acc ok
    bool selfTest();

    uint8_t getDirection(); // Tt is orientated to show the proper higher edge on your Watchy.
    bool IsUp();            // Returns True if your Watchy is in the Tilt position (with flexible room).

    bool setAccelConfig(bma4_accel_config &cfg);
    bool getAccelConfig(bma4_accel_config &cfg);
    bool getAccel(Accel *acc);
    bool getAccelMPSS(AccelF *acc); // Meters per second square, only bma530
    bool getAccelEnable();
    bool disableAccel();
    bool enableAccel();

    bool setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap);
    bool getINT();
    uint8_t getIRQMASK();
    bool disableIRQ(uint16_t int_map = BMA423_STEP_CNTR_INT);
    bool enableIRQ(uint16_t int_map = BMA423_STEP_CNTR_INT);
    bool isStepCounter();
    bool isDoubleClick();
    bool isTilt();
    bool isActivity();
    bool isAnyNoMotion();
    // bool didBMAWakeUp(uint64_t hwWakeup); // Allows you to tell via wakeupBit, if the BMA woke the Watchy, if it did, it reads the reason so you can use the above 4 functions.
    bool resetStepCounter();
    uint32_t getCounter();

    float readTemperature(bool Metric = true);
    float readTemperatureF();

    uint16_t getErrorCode();
    uint16_t getStatus();
    uint32_t getSensorTime();

    const char *getActivity();
    bool setRemapAxes(bma423_axes_remap *remap_data);
    bool setRemapAxes(bma456_axes_remap *remap_data);
    bool setRemapAxes(bma530_feat_axis *remap_data);

    bool enableFeature(uint8_t feature, uint8_t enable);
    bool enableStepCountInterrupt(bool en = true);
    bool enableTiltInterrupt(bool en = true);
    bool enableWakeupInterrupt(bool en = true);
    bool enableAnyNoMotionInterrupt(bool en = true);
    bool enableActivityInterrupt(bool en = true);
    bool defaultConfig(bool LowPower = true);
    bool enableDoubleClickWake(bool en = true); // Enables/Disables DoubleClick and the Wake Interrupt
    bool enableTiltWake(bool en = true);        // Enables/Disables Tilt and the Wake Interrupt
    bma4_dev _devFptr4;
    bma5_dev _devFptr5;
    bool damagedAcc;
    bool enableStepCount();

private:
    bma4_com_fptr_t _readRegisterFptr4;
    bma4_com_fptr_t _writeRegisterFptr4;
    bma5_read_fptr_t _readRegisterFptr5;
    bma5_write_fptr_t _writeRegisterFptr5;

    uint8_t _address;
    uint8_t _atchyVersion;
    uint16_t _IRQ_MASK;
    uint16_t _whichBma;
    bool isBma423();
    bool isBma456();
    bool isBma530();

    bool bma5Error(int8_t rslt);

    bool defaultConfig4(bool LowPower = true);
    bool defaultConfig5(bool LowPower = true);
};
