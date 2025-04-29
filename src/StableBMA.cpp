#include "StableBMA.h"

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

#define DEBUG_LOGS 0
#if DEBUG_LOGS
#define DEBUGPORT Serial
#endif

#ifdef DEBUGPORT
#define DEBUG(format, ...) DEBUGPORT.printf(format "\n", ##__VA_ARGS__)
#else
#define DEBUG(...)
#endif

#define BMA_423 423
#define BMA_456 456
#define BMA_530 530

StableBMA::StableBMA() {}

StableBMA::~StableBMA() {}

bool StableBMA::begin4(uint8_t atchyVersion, uint8_t address, uint16_t whichBma, bma4_com_fptr_t readCallBlack, bma4_com_fptr_t writeCallBlack)
{
    if (readCallBlack == nullptr ||
        writeCallBlack == nullptr ||
        atchyVersion == 0)
    {
        DEBUG("StableBMA: Arguments are wrong");
        return false;
    }

    _whichBma = whichBma;

    if (!(isBma423() || isBma456()))
    {
        DEBUG("Wrong function for this bma!");
        assert(false);
    }

    _atchyVersion = atchyVersion;

    _devFptr4.dev_addr = address;
    _address = address;

    _devFptr4.interface = BMA4_I2C_INTERFACE;

    _readRegisterFptr4 = readCallBlack;
    _writeRegisterFptr4 = writeCallBlack;
    _devFptr4.bus_read = readCallBlack;
    _devFptr4.bus_write = writeCallBlack;

    _devFptr4.delay = vTaskDelay;
    _devFptr4.read_write_len = 8;
    _devFptr4.resolution = 12;
    if (isBma423())
    {
        _devFptr4.feature_len = BMA423_FEATURE_SIZE;
    }
    else if (isBma456())
    {
        _devFptr4.feature_len = BMA456_FEATURE_SIZE;
    }

    StableBMA::softReset();

    vTaskDelay(20);

    if (isBma423())
    {
        if (bma423_init(&_devFptr4) != BMA4_OK)
        {
            DEBUG("BMA423 FAIL");
            return false;
        }

        if (bma423_write_config_file(&_devFptr4) != BMA4_OK)
        {
            DEBUG("BMA423 Write Config FAIL");
            return false;
        }
        return true;
    }
    else if (isBma456())
    {
        if (bma456_init(&_devFptr4) != BMA4_OK)
        {
            DEBUG("BMA456 FAIL");
            return false;
        }

        if (bma456_write_config_file(&_devFptr4) != BMA4_OK)
        {
            DEBUG("BMA456 Write Config FAIL");
            return false;
        }
        return true;
    }

    return false;
}

void bma5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    ets_delay_us(period);
}

bool StableBMA::begin5(uint8_t atchyVersion, uint8_t address, uint16_t whichBma, bma5_read_fptr_t readCallBlack, bma5_write_fptr_t writeCallBlack)
{
    if (readCallBlack == nullptr ||
        writeCallBlack == nullptr ||
        atchyVersion == 0)
    {
        DEBUG("StableBMA: Arguments are wrong");
        return false;
    }

    _whichBma = whichBma;

    if (!(isBma530()))
    {
        DEBUG("Wrong function for this bma!");
        assert(false);
    }

    _atchyVersion = atchyVersion;

    _devFptr5.bus_read = readCallBlack;
    _devFptr5.bus_write = writeCallBlack;
    _devFptr5.intf = BMA5_I2C_INTF;
    _devFptr5.context = bma5_context::BMA5_WEARABLE;
    _devFptr5.delay_us = bma5_delay_us;
    _devFptr5.chip_id = BMA530_CHIP_ID;
    _address = address;
    _devFptr5.intf_ptr = &_address;

    if (bma5Error(bma530_init(&_devFptr5)))
    {
        DEBUG("Failed to init bma5");
        return false;
    }

    DEBUG("Inited bma530 successfully");

    return true;
}

void StableBMA::softReset()
{
    if (isBma423() || isBma456())
    {
        uint8_t reg = BMA4_RESET_ADDR;
        _writeRegisterFptr4(BMA4_I2C_ADDR_PRIMARY, BMA4_RESET_SET_MASK, &reg, 1);
    }

    if (isBma530())
    {
        bma530_soft_reset(&_devFptr5);
    }
}

void StableBMA::shutDown()
{
    if (isBma423() || isBma456())
    {
        bma4_set_advance_power_save(BMA4_DISABLE, &_devFptr4);
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
}

void StableBMA::wakeUp()
{
    if (isBma423() || isBma456())
    {
        bma4_set_advance_power_save(BMA4_ENABLE, &_devFptr4);
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
}

uint16_t StableBMA::getErrorCode()
{
    if (isBma423() || isBma456())
    {
        struct bma4_err_reg err;
        uint16_t rslt = bma4_get_error_status(&err, &_devFptr4);
        return rslt;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return 0;
}

uint16_t StableBMA::getStatus()
{
    if (isBma423() || isBma456())
    {
        uint8_t status;
        bma4_get_status(&status, &_devFptr4);
        return status;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return 0;
}

uint32_t StableBMA::getSensorTime()
{
    if (isBma423() || isBma456())
    {
        uint32_t ms;
        bma4_get_sensor_time(&ms, &_devFptr4);
        return ms;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return 0;
}

bool StableBMA::selfTest()
{
    // Not implemented, for now it simply freezes the bma
    return true;

    if (isBma423() || isBma456())
    {
        if (damagedAcc == true)
        {
            return false;
        }
        damagedAcc = !(BMA4_OK == bma4_selftest_config(BMA4_ACCEL_SELFTEST_ENABLE_MSK, &_devFptr4));
        return !damagedAcc;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
}

uint8_t StableBMA::getDirection()
{
    Accel acc;
    if (!StableBMA::getAccel(&acc))
    {
        return false;
    }
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
    if (damagedAcc == true)
    {
        return false;
    }
    if (!StableBMA::getAccel(&acc))
    {
        return false;
    }
    return (acc.x <= 0 && acc.x >= -700 && acc.y >= -300 && acc.y <= 300 && acc.z <= -750 && acc.z >= -1070);
}

float StableBMA::readTemperature(bool Metric)
{
    if (isBma423() || isBma456())
    {
        int32_t data = 0;
        bma4_get_temperature(&data, BMA4_DEG, &_devFptr4);
        float temp = (float)data / (float)BMA4_SCALE_TEMP;
        if (((data - 23) / BMA4_SCALE_TEMP) == 0x80)
            return 0;
        return (Metric ? temp : (temp * 1.8 + 32.0));
    }
    if (isBma530())
    {
        DEBUG("To my knowledge, bma530 doesn't have it");
    }
    return 0.0;
}

float StableBMA::readTemperatureF()
{
    return StableBMA::readTemperature(false);
}

#define GRAVITY_EARTH (9.80665f)
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

bool StableBMA::getAccel(Accel *acc)
{
    if (damagedAcc == true)
    {
        return false;
    }

    if (isBma423() || isBma456())
    {
        memset(acc, 0, sizeof(acc));
        bma4_accel acc4 = {0};
        if (bma4_read_accel_xyz(&acc4, &_devFptr4) != BMA4_OK)
        {
            damagedAcc = true;
            DEBUG("Acc is damaged?");
            return false;
        }
        acc->x = acc4.x;
        acc->y = acc4.y;
        acc->z = acc4.z;

        if (_atchyVersion != 1)
        {
            acc->x = -acc->x;
            acc->y = -acc->y;
        }
    }

    if (isBma530())
    {
        bma5_sensor_status status;
        if (bma5Error(bma5_get_sensor_status(&status, &_devFptr5)))
        {
            DEBUG("Failed to bma5_get_sensor_status");
            return false;
        }
        // We do not clean the old data until new is here
        if (status.acc_data_rdy)
        {
            bma5_accel sens_data;
            if (bma5Error(bma5_get_acc(&sens_data, &_devFptr5)))
            {
                DEBUG("Failed to bma5_get_acc");
                return false;
            }
            memset(acc, 0, sizeof(acc));
            acc->x = sens_data.x;
            acc->y = sens_data.y;
            acc->z = sens_data.z;
        }
    }

    return true;
}

bool StableBMA::getAccelMPSS(AccelF *acc)
{
    if (isBma530())
    {
        Accel accR;
        if (getAccel(&accR) == false)
        {
            DEBUG("Failed to get regular acc");
            return false;
        }
        acc->x = lsb_to_ms2(accR.x, (float)2, BMA5_16_BIT_RESOLUTION);
        acc->y = lsb_to_ms2(accR.y, (float)2, BMA5_16_BIT_RESOLUTION);
        acc->z = lsb_to_ms2(accR.z, (float)2, BMA5_16_BIT_RESOLUTION);
        return true;
    }
    DEBUG("Not supported on this bma");
    return false;
}

bool StableBMA::getAccelEnable()
{
    if (isBma423() || isBma456())
    {
        uint8_t en;
        bma4_get_accel_enable(&en, &_devFptr4);
        return (en & BMA4_ACCEL_ENABLE_POS) == BMA4_ACCEL_ENABLE_POS;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::disableAccel()
{
    if (isBma423() || isBma456())
    {
        return (BMA4_OK == bma4_set_accel_enable(false ? BMA4_ENABLE : BMA4_DISABLE, &_devFptr4));
    }

    if (isBma530())
    {
        uint8_t sensor_ctrl = BMA5_SENSOR_CTRL_DISABLE;

        if (bma5Error(bma5_set_acc_conf_0(sensor_ctrl, &_devFptr5)))
        {
            DEBUG("Failed to disable accel for bma530");
            return false;
        }
        return true;
    }
    return false;
}

bool StableBMA::enableAccel()
{
    if (isBma423() || isBma456())
    {
        return (BMA4_OK == bma4_set_accel_enable(true ? BMA4_ENABLE : BMA4_DISABLE, &_devFptr4));
    }
    if (isBma530())
    {
        uint8_t sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;

        if (bma5Error(bma5_set_acc_conf_0(sensor_ctrl, &_devFptr5)))
        {
            DEBUG("Failed to enable accel for bma530");
            return false;
        }
        return true;
    }
    return false;
}

bool StableBMA::setAccelConfig(bma4_accel_config &cfg)
{
    if (isBma423() || isBma456())
    {
        return (BMA4_OK == bma4_set_accel_config(&cfg, &_devFptr4));
    }
    if (isBma530())
    {
        DEBUG("Wrong BMA!");
    }
    return false;
}

bool StableBMA::getAccelConfig(bma4_accel_config &cfg)
{
    if (isBma423() || isBma456())
    {
        return (BMA4_OK == bma4_get_accel_config(&cfg, &_devFptr4));
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::setRemapAxes(bma423_axes_remap *remap_data)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_set_remap_axes(remap_data, &_devFptr4));
    }
    else
    {
        DEBUG("Wrong struct for wrong acc");
    }
    return false;
}

bool StableBMA::setRemapAxes(bma456_axes_remap *remap_data)
{
    if (isBma456())
    {
        return (BMA4_OK == bma456_set_remap_axes(remap_data, &_devFptr4));
    }
    else
    {
        DEBUG("Wrong struct for wrong acc");
    }
    return false;
}

bool StableBMA::setRemapAxes(bma530_feat_axis *remap_data)
{
    if (isBma530())
    {
        /*
        bma530_feat_axis conf;
        if(bma5Error(bma530_get_feature_axis_config(&conf, &_devFptr5))) {
            DEBUG("Failed to bma530_get_feature_axis_config");
            return false;
        }
        */

        if (bma5Error(bma530_set_feature_axis_config(remap_data, &_devFptr5)))
        {
            DEBUG("Failed to bma530_set_feature_axis_config");
            return false;
        }

        return true;
    }
    else
    {
        DEBUG("Wrong struct for wrong acc");
    }
    return false;
}

bool StableBMA::resetStepCounter()
{
    if (isBma423())
    {
        return BMA4_OK == bma423_reset_step_counter(&_devFptr4);
    }
    if (isBma456())
    {
        return BMA4_OK == bma456_reset_step_counter(&_devFptr4);
    }
    if (isBma530())
    {
        // Place holder but should just work and who cares
        DEBUG("Reset step counter for bma530");
        return enableStepCount();
    }

    return false;
}

uint32_t StableBMA::getCounter()
{
    uint32_t stepCount;

    if (isBma423())
    {
        if (bma423_step_counter_output(&stepCount, &_devFptr4) == BMA4_OK)
        {
            return stepCount;
        }
    }
    if (isBma456())
    {
        if (bma456_step_counter_output(&stepCount, &_devFptr4) == BMA4_OK)
        {
            return stepCount;
        }
    }

    if (isBma530())
    {
        bma530_feat_eng_feat_out feat_out = {0};
        if (bma5Error(bma530_get_feat_eng_feature_out(&feat_out, &_devFptr5)))
        {
            DEBUG("Failed to bma530_get_feat_eng_feature_out");
            return 0;
        }

        stepCount = (long unsigned int)feat_out.step_cntr_out;
        return stepCount;
    }

    DEBUG("Failed to get step count");
    return 0;
}

bool StableBMA::setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap)
{
    if (isBma423() || isBma456())
    {
        return BMA4_OK == bma4_set_int_pin_config(&config, pinMap, &_devFptr4);
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::getINT()
{
    if (isBma423())
    {
        return bma423_read_int_status(&_IRQ_MASK, &_devFptr4) == BMA4_OK;
    }
    if (isBma456())
    {
        return bma456_read_int_status(&_IRQ_MASK, &_devFptr4) == BMA4_OK;
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

uint8_t StableBMA::getIRQMASK()
{
    return _IRQ_MASK;
}

bool StableBMA::disableIRQ(uint16_t int_map)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &_devFptr4));
    }
    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &_devFptr4));
    }
    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableIRQ(uint16_t int_map)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableFeature(uint8_t feature, uint8_t enable)
{
    if (isBma423())
    {
        if ((feature & BMA423_STEP_CNTR) == BMA423_STEP_CNTR)
        {
            bma423_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &_devFptr4);
        }
        return (BMA4_OK == bma423_feature_enable(feature, enable, &_devFptr4));
    }

    if (isBma456())
    {
        if ((feature & BMA456_STEP_CNTR) == BMA456_STEP_CNTR)
        {
            bma456_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &_devFptr4);
        }
        return (BMA4_OK == bma456_feature_enable(feature, enable, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }

    return false;
}

bool StableBMA::isStepCounter()
{
    if (isBma423())
    {
        return (bool)(BMA423_STEP_CNTR_INT & _IRQ_MASK);
    }

    if (isBma456())
    {
        return (bool)(BMA456_STEP_CNTR_INT & _IRQ_MASK);
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::isDoubleClick()
{
    if (isBma423())
    {
        return (bool)(BMA423_WAKEUP_INT & _IRQ_MASK);
    }

    if (isBma456())
    {
        return (bool)(BMA456_WAKEUP_INT & _IRQ_MASK);
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::isTilt()
{
    if (isBma423())
    {
        return (bool)(BMA423_TILT_INT & _IRQ_MASK);
    }

    if (isBma456())
    {
        DEBUG("Not sure");
        return (bool)(BMA456_WRIST_TILT_INT & _IRQ_MASK); // Not sure
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::isActivity()
{
    if (isBma423())
    {
        return (bool)(BMA423_ACTIVITY_INT & _IRQ_MASK);
    }

    if (isBma456())
    {
        return (bool)(BMA456_ACTIVITY_INT & _IRQ_MASK);
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::isAnyNoMotion()
{
    if (isBma423())
    {
        return (bool)(BMA423_ANY_NO_MOTION_INT & _IRQ_MASK);
    }

    if (isBma456())
    {
        return (bool)(BMA456_ANY_NO_MOTION_INT & _IRQ_MASK);
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableStepCountInterrupt(bool en)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_STEP_CNTR_INT, en, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, en, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableTiltInterrupt(bool en)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_TILT_INT, en, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_WRIST_TILT_INT, en, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableWakeupInterrupt(bool en)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_WAKEUP_INT, en, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_WAKEUP_INT, en, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableAnyNoMotionInterrupt(bool en)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ANY_NO_MOTION_INT, en, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ANY_NO_MOTION_INT, en, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableActivityInterrupt(bool en)
{
    if (isBma423())
    {
        return (BMA4_OK == bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_ACTIVITY_INT, en, &_devFptr4));
    }

    if (isBma456())
    {
        return (BMA4_OK == bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ACTIVITY_INT, en, &_devFptr4));
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

const char *StableBMA::getActivity()
{
    uint8_t activity;
    if (isBma423())
    {
        bma423_activity_output(&activity, &_devFptr4);
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
    }

    if (isBma456())
    {
        bma456_activity_output(&activity, &_devFptr4);
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

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return "None";
}

bool StableBMA::defaultConfig4(bool LowPower)
{
    struct bma4_int_pin_config config;
    bma4_accel_config cfg;
    config.edge_ctrl = BMA4_LEVEL_TRIGGER;
    // No support yet for interrupts!
    // config.lvl = (usingHIGHINT ? BMA4_ACTIVE_HIGH : BMA4_ACTIVE_LOW);
    config.od = BMA4_PUSH_PULL;
    config.output_en = BMA4_OUTPUT_ENABLE;
    config.input_en = BMA4_INPUT_DISABLE;
    if (LowPower == true)
    {
        cfg.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
    }
    else
    {
        cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    }
    cfg.range = BMA4_ACCEL_RANGE_2G;
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    cfg.perf_mode = (LowPower ? BMA4_CIC_AVG_MODE : BMA4_CONTINUOUS_MODE); // Testing for Low Power done by Michal Szczepaniak

    if (StableBMA::setAccelConfig(cfg))
    {
        if (StableBMA::enableAccel())
        {
            if (bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &_devFptr4) != BMA4_OK)
            {
                DEBUG("BMA423 DEF CFG FAIL");
                return false;
            }
            enableDoubleClickWake(false);
            enableTiltWake(false);
            enableActivityInterrupt(false);
            enableAnyNoMotionInterrupt(false);
            enableWakeupInterrupt(false);
            enableTiltInterrupt(false);
            enableStepCountInterrupt(false);
            if (_whichBma == BMA_423)
            {
                struct bma423_axes_remap remap_data;
                remap_data.x_axis = 1;
                remap_data.x_axis_sign = (_atchyVersion == 1 || _atchyVersion == 3 ? 1 : 0);
                remap_data.y_axis = 0;
                remap_data.y_axis_sign = (_atchyVersion == 1 ? 1 : 0);
                remap_data.z_axis = 2;
                remap_data.z_axis_sign = 1;
                return StableBMA::setRemapAxes(&remap_data);
            }
            else if (_whichBma == BMA_456)
            {
                struct bma456_axes_remap remap_data;
                remap_data.x_axis = 1;
                remap_data.x_axis_sign = (_atchyVersion == 1 || _atchyVersion == 3 ? 1 : 0);
                remap_data.y_axis = 0;
                remap_data.y_axis_sign = (_atchyVersion == 1 ? 1 : 0);
                remap_data.z_axis = 2;
                remap_data.z_axis_sign = 1;
                return StableBMA::setRemapAxes(&remap_data);
            }
        }
    }
    return false;
}

bool StableBMA::defaultConfig5(bool LowPower)
{
    bma5_acc_conf acc_cfg;
    if (bma5Error(bma5_get_acc_conf(&acc_cfg, &_devFptr5)))
    {
        DEBUG("Failed to get acc conf");
        return false;
    }

    // LowPower not implemented (I think it's on default now)
    acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_25;
    acc_cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;
    acc_cfg.power_mode = BMA5_POWER_MODE_HPM;

    acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_2G;
    acc_cfg.acc_iir_ro = BMA5_ACC_IIR_RO_DB_40;
    acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_POWER;
    acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED;

    if (bma5Error(bma5_set_acc_conf(&acc_cfg, &_devFptr5)))
    {
        DEBUG("Failed to set acc conf");
        return false;
    }

    /*
    bma530_feat_axis remap_data;
    remap_data.feat_axis_ex = BMA530_FEAT_AXIS_EX_ZXY;
    remap_data.feat_x_inv = BMA530_FEAT_X_INV_INVERT;
    remap_data.feat_y_inv = BMA530_FEAT_Y_INV_INVERT;
    remap_data.feat_z_inv = BMA530_FEAT_Z_INV_INVERT;

    if (bma5Error(setRemapAxes(&remap_data)))
    {
        DEBUG("Failed to remap axes");
        return false;
    }
    */

    DEBUG("Succ for def conf bma5");
    return true;
}

bool StableBMA::defaultConfig(bool LowPower)
{
    if (isBma423() || isBma456())
    {
        return defaultConfig4();
    }
    else if (isBma530())
    {
        return defaultConfig5();
    }
    return false;
}

bool StableBMA::enableDoubleClickWake(bool en)
{
    if (isBma423())
    {
        if (StableBMA::enableFeature(BMA423_WAKEUP, en))
        {
            return StableBMA::enableWakeupInterrupt(en);
        }
    }

    if (isBma456())
    {
        if (StableBMA::enableFeature(BMA456_WAKEUP, en))
        {
            return StableBMA::enableWakeupInterrupt(en);
        }
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::enableTiltWake(bool en)
{
    if (isBma423())
    {
        if (StableBMA::enableFeature(BMA423_TILT, en))
        {
            return StableBMA::enableTiltInterrupt(en);
        }
    }

    if (isBma456())
    {
        if (StableBMA::enableFeature(BMA456_WRIST_TILT_INT, en))
        {
            return StableBMA::enableTiltInterrupt(en);
        }
    }

    if (isBma530())
    {
        DEBUG("Not implemented!");
    }
    return false;
}

bool StableBMA::isBma423()
{
    if (_whichBma == BMA_423)
    {
        return true;
    }
    return false;
}

bool StableBMA::isBma456()
{
    if (_whichBma == BMA_456)
    {
        return true;
    }
    return false;
}

bool StableBMA::isBma530()
{
    if (_whichBma == BMA_530)
    {
        return true;
    }
    return false;
}

bool StableBMA::bma5Error(int8_t rslt)
{
    switch (rslt)
    {
    case BMA5_OK:
        return false;
        break;
    case BMA5_E_NULL_PTR:
        DEBUG("Error  [%d] : Null pointer", rslt);
        break;
    case BMA5_E_COM_FAIL:
        DEBUG("Error  [%d] : Communication failure", rslt);
        break;
    case BMA5_E_DEV_NOT_FOUND:
        DEBUG("Error  [%d] : Device not found", rslt);
        break;
    default:
        DEBUG("Error  [%d] : Unknown error code", rslt);
        break;
    }
    return true;
}

bool StableBMA::enableStepCount()
{
    if (isBma423())
    {
        if (enableFeature(BMA423_STEP_CNTR, true) == false)
        {
            DEBUG("enableFeature(BMA423_STEP_CNTR failed");
            return false;
        }
    }
    if (isBma456())
    {
        if (enableFeature(BMA456_STEP_CNTR, true) == false)
        {
            DEBUG("enableFeature(BMA456_STEP_CNTR failed");
            return false;
        }
    }

    if (isBma530())
    {
        // https://github.com/boschsensortec/BMA530_SensorAPI/blob/master/examples/step_counter/step_counter.c
        bma530_feat_eng_gpr_0 gpr_0 = {0};
        bma530_step_cntr conf = {0};
        uint8_t gpr_ctrl_host = BMA5_ENABLE;

        if (bma5Error(bma530_get_feat_eng_gpr_0(&gpr_0, &_devFptr5)))
        {
            DEBUG("Failed to bma530_get_feat_eng_gpr_0");
            return false;
        }
        gpr_0.step_en = BMA5_ENABLE;

        if (bma5Error(bma530_set_feat_eng_gpr_0(&gpr_0, &_devFptr5)))
        {
            DEBUG("Failed to bma530_set_feat_eng_gpr_0");
            return false;
        }

        if (bma5Error(bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &_devFptr5)))
        {
            DEBUG("Failed to bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL");
            return false;
        }

        if (bma5Error(bma530_get_default_step_counter_config(&conf, &_devFptr5)))
        {
            DEBUG("Failed to bma530_get_default_step_counter_config");
            return false;
        }

        conf.acc_mean_decay_coeff = 0xEAC8;
        conf.activity_detection_factor = 0x3;
        conf.activity_detection_thres = 0xF3C;
        conf.en_half_step = 0x0;
        conf.en_mcr_pp = 0x0;
        conf.en_step_dur_pp = 0x0;
        conf.envelope_down_decay_coeff = 0xD938;
        conf.envelope_down_thres = 0x84;
        conf.envelope_up_decay_coeff = 0xF1CC;
        conf.envelope_up_thres = 0x132;
        conf.filter_cascade_enabled = 0x1;
        conf.filter_coeff_a_1 = 0x41EF;
        conf.filter_coeff_a_2 = 0xE897;
        conf.filter_coeff_b_0 = 0x55F;
        conf.filter_coeff_b_1 = 0xABE;
        conf.filter_coeff_b_2 = 0x55F;
        conf.filter_coeff_scale_a = 0xE;
        conf.filter_coeff_scale_b = 0xE;
        conf.mcr_thres = 0x0;
        conf.peak_duration_min_running = 0xC;
        conf.peak_duration_min_walking = 0xC;
        conf.reset_counter = 0x0;
        conf.sc_en = 0x1;
        conf.sd_en = 0x1;
        conf.step_buffer_size = 0x7;
        conf.step_counter_increment = 0x100;
        conf.step_dur_mean_decay_coeff = 0xFD54;
        conf.step_dur_thres = 0x0;
        conf.step_duration_max = 0x4A;
        conf.step_duration_window = 0xA0;

        conf.watermark_level = 1;

        if (bma5Error(bma530_set_step_counter_config(&conf, &_devFptr5)))
        {
            DEBUG("Failed to bma530_set_step_counter_config");
            return false;
        }

        DEBUG("BMA530 step counter enabled");
    }

    return true;
}