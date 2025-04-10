#include "StableBMA.h"

/* Forked from bma.cpp by GuruSR (https://www.github.com/GuruSR/StableBMA) 
 * This fork is to improve Watchy functionality based on board version
 * (via RTCType).
 *
 * Version 1.0, February  6, 2022 - Initial changes for Watchy usage.
 * Version 1.1, February  8, 2022 - Fixed readTemperatureF to show F properly.
 * Version 1.2, July     19, 2022 - Fixed readTemperatureF to include errors.
 *                                  License Update.
 * Version 1.3, December 31, 2023 - Added orientation for V3.0 and cleaned up
 *                                  temperature code.
 * Version 1.4, February 24, 2024 - Added Low Power mode to the
 *                                  defaultConfig().
 * Version 1.5, July      9, 2024 - Fixed wrong getAccel assignments and added
 *                                  conditionBMA.
 * Version 1.6, July     30, 2024 - Fixed V3 orientation, missing return and
 *                                  int level.
 *
 * MIT License
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

#define DEBUGPORT Serial
#ifdef DEBUGPORT
#define DEBUG(...)      DEBUGPORT.printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif
#include <Arduino.h>
// BMA For Tilt/DTap
RTC_DATA_ATTR uint8_t BMA_INT1_PIN;
RTC_DATA_ATTR uint8_t BMA_INT2_PIN;
RTC_DATA_ATTR uint32_t BMA_INT1_MASK;
RTC_DATA_ATTR uint32_t BMA_INT2_MASK;
RTC_DATA_ATTR bool usingHIGHINT;

// BMA condition
RTC_DATA_ATTR uint8_t conditionBMA;

#define BMA_423 423
#define BMA_456 456

StableBMA::StableBMA()
{
    __readRegisterFptr = nullptr;
    __writeRegisterFptr = nullptr;
    __delayCallBlackFptr = nullptr;
    __init = false;
}

StableBMA::~StableBMA() {}

bool StableBMA::begin(bma4_com_fptr_t readCallBlack, bma4_com_fptr_t writeCallBlack, bma4_delay_fptr_t delayCallBlack, uint8_t atchyVersion, uint8_t address, bool usesHIGHINT, uint8_t BMA_INT1_PIN, uint8_t BMA_INT2_PIN, uint16_t whichBma)
{
    if (__init ||
        readCallBlack == nullptr ||
        writeCallBlack == nullptr ||
        delayCallBlack == nullptr ||
        atchyVersion == 0)
    {
        DEBUG("StableBMA: Arguments are wrong");
        return true;
    }

    BMA_INT1_PIN = BMA_INT1_PIN;
    BMA_INT2_PIN = BMA_INT2_PIN;
    BMA_INT1_MASK = (1 << BMA_INT1_PIN);
    BMA_INT2_MASK = (1 << BMA_INT2_PIN);
    usingHIGHINT = usesHIGHINT;
    __whichBma = whichBma;

    __readRegisterFptr = readCallBlack;
    __writeRegisterFptr = writeCallBlack;
    __delayCallBlackFptr = delayCallBlack;
    __atchyVersion = atchyVersion;

    __devFptr.dev_addr = address;
    __devFptr.interface = BMA4_I2C_INTERFACE;
    __devFptr.bus_read = readCallBlack;
    __devFptr.bus_write = writeCallBlack;
    __devFptr.delay = delayCallBlack;
    __devFptr.read_write_len = 8;
    __devFptr.resolution = 12;
    if (__whichBma == BMA_423)
    {
        __devFptr.feature_len = BMA423_FEATURE_SIZE;
    }
    else if (__whichBma == BMA_456)
    {
        __devFptr.feature_len = BMA456_FEATURE_SIZE;
    }

    StableBMA::softReset();

    __delayCallBlackFptr(20);

    if (__whichBma == BMA_423)
    {
        if (bma423_init(&__devFptr) != BMA4_OK)
        {
            DEBUG("BMA423 FAIL\n");
            return false;
        }

        if (bma423_write_config_file(&__devFptr) != BMA4_OK)
        {
            DEBUG("BMA423 Write Config FAIL\n");
            return false;
        }
    }
    else if (__whichBma == BMA_456)
    {
        if (bma456_init(&__devFptr) != BMA4_OK)
        {
            DEBUG("BMA456 FAIL\n");
            return false;
        }

        if (bma456_write_config_file(&__devFptr) != BMA4_OK)
        {
            DEBUG("BMA456 Write Config FAIL\n");
            return false;
        }
    }

    __init = true;

    return __init;
}

void StableBMA::softReset()
{
    uint8_t reg = BMA4_RESET_ADDR;
    __writeRegisterFptr(BMA4_I2C_ADDR_PRIMARY, BMA4_RESET_SET_MASK, &reg, 1);
}

void StableBMA::shutDown()
{
    bma4_set_advance_power_save(BMA4_DISABLE, &__devFptr);
}

void StableBMA::wakeUp()
{
    bma4_set_advance_power_save(BMA4_ENABLE, &__devFptr);
}

uint16_t StableBMA::getErrorCode()
{
    struct bma4_err_reg err;
    uint16_t rslt = bma4_get_error_status(&err, &__devFptr);
    return rslt;
}

uint16_t StableBMA::getStatus()
{
    uint8_t status;
    bma4_get_status(&status, &__devFptr);
    return status;
}

uint32_t StableBMA::getSensorTime()
{
    uint32_t ms;
    bma4_get_sensor_time(&ms, &__devFptr);
    return ms;
}

bool StableBMA::selfTest()
{
    return (BMA4_OK == bma4_selftest_config(BMA4_ACCEL_SELFTEST_ENABLE_MSK, &__devFptr));
}

uint8_t StableBMA::getDirection()
{
    Accel acc;
    if (!StableBMA::getAccel(acc))
        return 0;
    uint16_t absX = abs(acc.x);
    uint16_t absY = abs(acc.y);
    uint16_t absZ = abs(acc.z);

    if ((absZ > absX) && (absZ > absY))
    {
        return ((acc.z > 0) ? DIRECTION_DISP_DOWN : DIRECTION_DISP_UP);
    }
    else if ((absY > absX) && (absY > absZ))
    {
        return ((acc.y > 0) ? DIRECTION_LEFT_EDGE : DIRECTION_RIGHT_EDGE);
    }
    else
    {
        return ((acc.x < 0) ? DIRECTION_TOP_EDGE : DIRECTION_BOTTOM_EDGE);
    }
}

bool StableBMA::IsUp()
{
    Accel acc;
    bool b;
    if (conditionBMA & 1)
        return false; // Broken Accelerometer
    if (!StableBMA::getAccel(acc))
        return false;
    return (acc.x <= 0 && acc.x >= -700 && acc.y >= -300 && acc.y <= 300 && acc.z <= -750 && acc.z >= -1070);
}

float StableBMA::readTemperature(bool Metric)
{
    int32_t data = 0;
    bma4_get_temperature(&data, BMA4_DEG, &__devFptr);
    float temp = (float)data / (float)BMA4_SCALE_TEMP;
    if (((data - 23) / BMA4_SCALE_TEMP) == 0x80)
        return 0;
    return (Metric ? temp : (temp * 1.8 + 32.0));
}

float StableBMA::readTemperatureF()
{
    return StableBMA::readTemperature(false);
}

bool StableBMA::getAccel(Accel &acc)
{
    memset(&acc, 0, sizeof(acc));
    if (conditionBMA & 1)
        return false;
    if (bma4_read_accel_xyz(&acc, &__devFptr) != BMA4_OK)
    {
        conditionBMA |= 1;
        return false;
    }
    if (__atchyVersion != 1)
    {
        acc.x = -acc.x;
        acc.y = -acc.y;
    }
    return true;
}

bool StableBMA::getAccelEnable()
{
    uint8_t en;
    bma4_get_accel_enable(&en, &__devFptr);
    return (en & BMA4_ACCEL_ENABLE_POS) == BMA4_ACCEL_ENABLE_POS;
}

bool StableBMA::disableAccel()
{
    return StableBMA::enableAccel(false);
}

bool StableBMA::enableAccel(bool en)
{
    return (BMA4_OK == bma4_set_accel_enable(en ? BMA4_ENABLE : BMA4_DISABLE, &__devFptr));
}

bool StableBMA::setAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_set_accel_config(&cfg, &__devFptr));
}

bool StableBMA::getAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_get_accel_config(&cfg, &__devFptr));
}

bool StableBMA::setRemapAxes(struct bma423_axes_remap *remap_data)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_set_remap_axes(remap_data, &__devFptr));
    } else {
        DEBUG("Wrong struct for wrong acc");
    }
    return false;
}

bool StableBMA::setRemapAxes(struct bma456_axes_remap *remap_data)
{
    if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_set_remap_axes(remap_data, &__devFptr));
    } else {
        DEBUG("Wrong struct for wrong acc");
    }
    return false;
}

bool StableBMA::resetStepCounter()
{
    if(__whichBma == BMA_423) {
        return BMA4_OK == bma423_reset_step_counter(&__devFptr);
    } else if(__whichBma == BMA_456) {
        return BMA4_OK == bma456_reset_step_counter(&__devFptr);
    }
    return false;
}

uint32_t StableBMA::getCounter()
{
    uint32_t stepCount;

    if(__whichBma == BMA_423) {
        if (bma423_step_counter_output(&stepCount, &__devFptr) == BMA4_OK)
        {
            return stepCount;
        }
    } else if(__whichBma == BMA_456) {
        if (bma456_step_counter_output(&stepCount, &__devFptr) == BMA4_OK)
        {
            return stepCount;
        }
    }

    DEBUG("Failed to get step count");
    return 0;
}

bool StableBMA::setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap)
{
    return BMA4_OK == bma4_set_int_pin_config(&config, pinMap, &__devFptr);
}

bool StableBMA::getINT()
{
    if(__whichBma == BMA_423) {
        return bma423_read_int_status(&__IRQ_MASK, &__devFptr) == BMA4_OK;
    } else if(__whichBma == BMA_456) {
        return bma456_read_int_status(&__IRQ_MASK, &__devFptr) == BMA4_OK;
    }
    return false;
}

uint8_t StableBMA::getIRQMASK()
{
    return __IRQ_MASK;
}

bool StableBMA::disableIRQ(uint16_t int_map)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &__devFptr));
    }
    return false;
}

bool StableBMA::enableIRQ(uint16_t int_map)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &__devFptr));
    }
    return false;
}

bool StableBMA::enableFeature(uint8_t feature, uint8_t enable)
{   
    if(__whichBma == BMA_423) {
        if ((feature & BMA423_STEP_CNTR) == BMA423_STEP_CNTR)
        {
            bma423_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &__devFptr);
        }
        return (BMA4_OK == bma423_feature_enable(feature, enable, &__devFptr));
    } else if(__whichBma == BMA_456) {
        if ((feature & BMA456_STEP_CNTR) == BMA456_STEP_CNTR)
        {
            bma456_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &__devFptr);
        }
        return (BMA4_OK == bma456_feature_enable(feature, enable, &__devFptr));
    }
    return false;
}

bool StableBMA::isStepCounter()
{
    if(__whichBma == BMA_423) {
        return (bool)(BMA423_STEP_CNTR_INT & __IRQ_MASK);
    } else if(__whichBma == BMA_456) {
        return (bool)(BMA456_STEP_CNTR_INT & __IRQ_MASK);
    }
    return false;
}

bool StableBMA::isDoubleClick()
{
    if(__whichBma == BMA_423) {
        return (bool)(BMA423_WAKEUP_INT & __IRQ_MASK);
    } else if(__whichBma == BMA_456) {
        return (bool)(BMA456_WAKEUP_INT & __IRQ_MASK);
    }
    return false;
}

bool StableBMA::isTilt()
{
    if(__whichBma == BMA_423) {
        return (bool)(BMA423_TILT_INT & __IRQ_MASK);
    } else if(__whichBma == BMA_456) {
        DEBUG("Not sure");
        return (bool)(BMA456_WRIST_TILT_INT & __IRQ_MASK); // Not sure
    }
    return false;
}

bool StableBMA::isActivity()
{
    if(__whichBma == BMA_423) {
        return (bool)(BMA423_ACTIVITY_INT & __IRQ_MASK);
    } else if(__whichBma == BMA_456) {
        return (bool)(BMA456_ACTIVITY_INT & __IRQ_MASK);
    }
    return false;
}

bool StableBMA::isAnyNoMotion()
{
    if(__whichBma == BMA_423) {
        return (bool)(BMA423_ANY_NO_MOTION_INT & __IRQ_MASK);
    } else if(__whichBma == BMA_456) {
        return (bool)(BMA456_ANY_NO_MOTION_INT & __IRQ_MASK);
    }
    return false;
}

bool StableBMA::didBMAWakeUp(uint64_t hwWakeup)
{
    bool B = (hwWakeup & BMA_INT1_MASK);
    if (!B)
        return B;
    if (StableBMA::getINT())
        return B;
    return false;
}

bool StableBMA::enableStepCountInterrupt(bool en)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_STEP_CNTR_INT, en, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, en, &__devFptr));
    }
    return false;
}

bool StableBMA::enableTiltInterrupt(bool en)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_TILT_INT, en, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_WRIST_TILT_INT, en, &__devFptr));
    }
    return false;
}

bool StableBMA::enableWakeupInterrupt(bool en)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_WAKEUP_INT, en, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_WAKEUP_INT, en, &__devFptr));
    }
    return false;
}

bool StableBMA::enableAnyNoMotionInterrupt(bool en)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ANY_NO_MOTION_INT, en, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ANY_NO_MOTION_INT, en, &__devFptr));
    }
    return false;
}

bool StableBMA::enableActivityInterrupt(bool en)
{
    if(__whichBma == BMA_423) {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ACTIVITY_INT, en, &__devFptr));
    } else if(__whichBma == BMA_456) {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ACTIVITY_INT, en, &__devFptr));
    }
    return false;
}

const char *StableBMA::getActivity()
{
    uint8_t activity;
    if(__whichBma == BMA_423) {
        bma423_activity_output(&activity, &__devFptr);
        if (activity & BMA423_USER_STATIONARY)
        {
            return "BMA423_USER_STATIONARY";
        }
        else if (activity & BMA423_USER_WALKING)
        {
            return "BMA423_USER_WALKING";
        }
        else if (activity & BMA423_USER_RUNNING)
        {
            return "BMA423_USER_RUNNING";
        }
        else if (activity & BMA423_STATE_INVALID)
        {
            return "BMA423_STATE_INVALID";
        }
    } else if(__whichBma == BMA_456) {
        bma456_activity_output(&activity, &__devFptr);
        if (activity & BMA456_USER_STATIONARY)
        {
            return "BMA456_USER_STATIONARY";
        }
        else if (activity & BMA456_USER_WALKING)
        {
            return "BMA456_USER_WALKING";
        }
        else if (activity & BMA456_USER_RUNNING)
        {
            return "BMA456_USER_RUNNING";
        }
        else if (activity & BMA456_STATE_INVALID)
        {
            return "BMA456_STATE_INVALID";
        }
    }
    return "None";
}

uint32_t StableBMA::WakeMask()
{
    return BMA_INT1_MASK;
}

bool StableBMA::defaultConfig(bool LowPower)
{
    struct bma4_int_pin_config config;
    Acfg cfg;
    config.edge_ctrl = BMA4_LEVEL_TRIGGER;
    config.lvl = (usingHIGHINT ? BMA4_ACTIVE_HIGH : BMA4_ACTIVE_LOW);
    config.od = BMA4_PUSH_PULL;
    config.output_en = BMA4_OUTPUT_ENABLE;
    config.input_en = BMA4_INPUT_DISABLE;
    if(LowPower == true) {
        cfg.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
    } else {
        cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    }
    cfg.range = BMA4_ACCEL_RANGE_2G;
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    cfg.perf_mode = (LowPower ? BMA4_CIC_AVG_MODE : BMA4_CONTINUOUS_MODE); // Testing for Low Power done by Michal Szczepaniak

    if (StableBMA::setAccelConfig(cfg))
    {
        if (StableBMA::enableAccel())
        {
            if (bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &__devFptr) != BMA4_OK)
            {
                DEBUG("BMA423 DEF CFG FAIL\n");
                return false;
            }
            enableDoubleClickWake(false);
            enableTiltWake(false);
            enableActivityInterrupt(false);
            enableAnyNoMotionInterrupt(false);
            enableWakeupInterrupt(false);
            enableTiltInterrupt(false);
            enableStepCountInterrupt(false);
            if(__whichBma == BMA_423) {
                struct bma423_axes_remap remap_data;
                remap_data.x_axis = 1;
                remap_data.x_axis_sign = (__atchyVersion == 1 || __atchyVersion == 3 ? 1 : 0);
                remap_data.y_axis = 0;
                remap_data.y_axis_sign = (__atchyVersion == 1 ? 1 : 0);
                remap_data.z_axis = 2;
                remap_data.z_axis_sign = 1;
                return StableBMA::setRemapAxes(&remap_data);    
            } else if(__whichBma == BMA_456) {
                struct bma456_axes_remap remap_data;
                remap_data.x_axis = 1;
                remap_data.x_axis_sign = (__atchyVersion == 1 || __atchyVersion == 3 ? 1 : 0);
                remap_data.y_axis = 0;
                remap_data.y_axis_sign = (__atchyVersion == 1 ? 1 : 0);
                remap_data.z_axis = 2;
                remap_data.z_axis_sign = 1;
                return StableBMA::setRemapAxes(&remap_data);    
            }
        }
    }
    return false;
}

bool StableBMA::enableDoubleClickWake(bool en)
{
    if(__whichBma == BMA_423) {
        if (StableBMA::enableFeature(BMA423_WAKEUP, en))
            return StableBMA::enableWakeupInterrupt(en);
    } else if(__whichBma == BMA_456) {
        if (StableBMA::enableFeature(BMA456_WAKEUP, en))
            return StableBMA::enableWakeupInterrupt(en);
    }
    return false;
}

bool StableBMA::enableTiltWake(bool en)
{
    if(__whichBma == BMA_423) {
        if (StableBMA::enableFeature(BMA423_TILT, en))
            return StableBMA::enableTiltInterrupt(en);
    } else if(__whichBma == BMA_456) {
        if (StableBMA::enableFeature(BMA456_WRIST_TILT_INT, en))
            return StableBMA::enableTiltInterrupt(en);
    }
    return false;
}
