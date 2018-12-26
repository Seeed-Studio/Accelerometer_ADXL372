/*
 * adxl372.h
 * Library for Grove - 3 Axis accelerometer ADXL372
 *
 * Copyright (c) 2018 seeed technology inc.
 * The MIT License (MIT)
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef __ADXL372_H__
#define __ADXL372_H__

#include <stdint.h>

#include "Arduino.h"
#include "Wire.h"


// ADXL372 Register Address
#define ADXL372_ADI_DEVID           0x00   // Analog Devices, Inc., accelerometer ID 
#define ADXL372_MST_DEVID           0x01   // Analog Devices MEMS device ID 
#define ADXL372_DEVID               0x02   // Device ID 
#define ADXL372_REVID               0x03   // product revision ID
#define ADXL372_STATUS_1            0x04   // Status register 1 
#define ADXL372_STATUS_2            0x05   // Status register 2 
#define ADXL372_FIFO_ENTRIES_2      0x06   // Valid data samples in the FIFO 
#define ADXL372_FIFO_ENTRIES_1      0x07   // Valid data samples in the FIFO 
#define ADXL372_X_DATA_H            0x08   // X-axis acceleration data [11:4] 
#define ADXL372_X_DATA_L            0x09   // X-axis acceleration data [3:0] | dummy LSBs 
#define ADXL372_Y_DATA_H            0x0A   // Y-axis acceleration data [11:4] 
#define ADXL372_Y_DATA_L            0x0B   // Y-axis acceleration data [3:0] | dummy LSBs 
#define ADXL372_Z_DATA_H            0x0C   // Z-axis acceleration data [11:4] 
#define ADXL372_Z_DATA_L            0x0D   // Z-axis acceleration data [3:0] | dummy LSBs 
#define ADXL372_X_MAXPEAK_H         0x15   // X-axis MaxPeak acceleration data [15:8] 
#define ADXL372_X_MAXPEAK_L         0x16   // X-axis MaxPeak acceleration data [7:0] 
#define ADXL372_Y_MAXPEAK_H         0x17   // X-axis MaxPeak acceleration data [15:8] 
#define ADXL372_Y_MAXPEAK_L         0x18   // X-axis MaxPeak acceleration data [7:0] 
#define ADXL372_Z_MAXPEAK_H         0x19   // X-axis MaxPeak acceleration data [15:8] 
#define ADXL372_Z_MAXPEAK_L         0x1A   // X-axis MaxPeak acceleration data [7:0] 
#define ADXL372_OFFSET_X            0x20   // X axis offset 
#define ADXL372_OFFSET_Y            0x21   // Y axis offset 
#define ADXL372_OFFSET_Z            0x22   // Z axis offset 
#define ADXL372_X_THRESH_ACT_H      0x23   // X axis Activity Threshold [15:8] 
#define ADXL372_X_THRESH_ACT_L      0x24   // X axis Activity Threshold [7:0] 
#define ADXL372_Y_THRESH_ACT_H      0x25   // Y axis Activity Threshold [15:8] 
#define ADXL372_Y_THRESH_ACT_L      0x26   // Y axis Activity Threshold [7:0] 
#define ADXL372_Z_THRESH_ACT_H      0x27   // Z axis Activity Threshold [15:8] 
#define ADXL372_Z_THRESH_ACT_L      0x28   // Z axis Activity Threshold [7:0] 
#define ADXL372_TIME_ACT            0x29   // Activity Time 
#define ADXL372_X_THRESH_INACT_H    0x2A   // X axis Inactivity Threshold [15:8] 
#define ADXL372_X_THRESH_INACT_L    0x2B   // X axis Inactivity Threshold [7:0] 
#define ADXL372_Y_THRESH_INACT_H    0x2C   // Y axis Inactivity Threshold [15:8] 
#define ADXL372_Y_THRESH_INACT_L    0x2D   // Y axis Inactivity Threshold [7:0] 
#define ADXL372_Z_THRESH_INACT_H    0x2E   // Z axis Inactivity Threshold [15:8] 
#define ADXL372_Z_THRESH_INACT_L    0x2F   // Z axis Inactivity Threshold [7:0] 
#define ADXL372_TIME_INACT_H        0x30   // Inactivity Time [15:8] 
#define ADXL372_TIME_INACT_L        0x31   // Inactivity Time [7:0] 
#define ADXL372_X_THRESH_ACT2_H     0x32   // X axis Activity2 Threshold [15:8] 
#define ADXL372_X_THRESH_ACT2_L     0x33   // X axis Activity2 Threshold [7:0] 
#define ADXL372_Y_THRESH_ACT2_H     0x34   // Y axis Activity2 Threshold [15:8] 
#define ADXL372_Y_THRESH_ACT2_L     0x35   // Y axis Activity2 Threshold [7:0] 
#define ADXL372_Z_THRESH_ACT2_H     0x36   // Z axis Activity2 Threshold [15:8] 
#define ADXL372_Z_THRESH_ACT2_L     0x37   // Z axis Activity2 Threshold [7:0] 
#define ADXL372_HPF                 0x38   // High Pass Filter 
#define ADXL372_FIFO_SAMPLES        0x39   // FIFO Samples 
#define ADXL372_FIFO_CTL            0x3A   // FIFO Control 
#define ADXL372_INT1_MAP            0x3B   // Interrupt 1 mapping control 
#define ADXL372_INT2_MAP            0x3C   // Interrupt 2 mapping control 
#define ADXL372_TIMING              0x3D   // Timing 
#define ADXL372_MEASURE             0x3E   // Measure 
#define ADXL372_POWER_CTL           0x3F   // Power control 
#define ADXL372_SELF_TEST           0x40   // Self Test 
#define ADXL372_SRESET              0x41   // Reset 
#define ADXL372_FIFO_DATA           0x42   // FIFO Data 

#define ADXL372_ADI_DEVID_VAL       0xAD   // Analog Devices, Inc., accelerometer ID 
#define ADXL372_MST_DEVID_VAL       0x1D   // Analog Devices MEMS device ID 
#define ADXL372_DEVID_VAL           0xFA   // Device ID 
#define ADXL372_REVID_VAL           0x02   // product revision ID


#define MEASURE_AUTOSLEEP_MASK          0xBF
#define MEASURE_BANDWIDTH_MASK          0xF8
#define MEASURE_ACTPROC_MASK            0xCF
#define TIMING_ODR_MASK                 0x1F
#define TIMING_WUR_MASK                 0xE3
#define PWRCTRL_OPMODE_MASK             0xFC
#define PWRCTRL_INSTON_THRESH_MASK      0xDF
#define PWRCTRL_INSTON_THRESH_MASK      0xDF
#define PWRCTRL_FILTER_SETTLE_MASK      0xEF

#define MEASURE_AUTOSLEEP_POS           6
#define MEASURE_ACTPROC_POS             4
#define TIMING_ODR_POS                  5
#define TIMING_WUR_POS                  2
#define INSTAON_THRESH_POS              5
#define FIFO_CRL_SAMP8_POS              0
#define FIFO_CRL_MODE_POS               1
#define FIFO_CRL_FORMAT_POS             3
#define PWRCTRL_FILTER_SETTLE_POS       4

#define DATA_READY                      1
#define FIFO_READY                      2
#define FIFO_FULL                       4
#define FIFO_OVERRUN                    8


#define DEFAULT_ADXL372_ADDR           0x53

typedef enum {
    STANDBY_MODE = 0,
    WAKEUP_MODE,
    INSTANT_ON_MODE,
    MEASUREMENT_MODE
} operating_mode_t;

typedef enum {
    RATE_400 = 0,
    RATE_800,
    RATE_1600,
    RATE_3200,
    RATE_6400
} rate_t;

typedef enum {
    BW_200 = 0,
    BW_400,
    BW_800,
    BW_1600,
    BW_3200
} bandwidth_t;

typedef enum {
    MS52 = 0,
    MS104,
    MS208,
    MS512,
    MS2048,
    MS4096,
    MS8192,
    MS24576
} wakeup_time_t;


typedef enum {
    DEF = 0,
    LINKED,
    LOOPED
} ADXL372_ACT_PROC_MODE;


typedef enum {
    FIFO_XYZ = 0,
    FIFO_X,
    FIFO_Y,
    FIFO_XY,
    FIFO_Z,
    FIFO_XZ,
    FIFO_YZ,
    FIFO_XYZ_PEAK
} fifo_format_t;

typedef enum {
    BYPASSED = 0,
    STREAMED,
    TRIGGERED,
    OLDEST_SAVED
} fifo_mode_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} xyz_t; 


typedef enum
{
    CHANNEL_X = 0,
    CHANNEL_Y,
    CHANNEL_Z,
}channel_t;

typedef enum
{
    INT_NUM_1 = 0,
    INT_NUM_2,
}IntPin_t;


class ADXL372 {
public:
    ADXL372(uint8_t addr = DEFAULT_ADXL372_ADDR) {
        i2c_addr = addr;
    }

    int begin() {
        Wire.begin();
        return reset();
    }

    /* 
    * Set operating mode, low pass filter and high pass filter
    *
    * @param: mode  supported modes: STANDBY_MODE WAKEUP_MODE, INSTANT_ON_MODE, MEASUREMENT_MODE
    * @param: low_pass_filter   true - enable low pass anti-aliasing filter, otherwise disable the filter
    *                           disable it when oversampling a signal
    * @param: high_pass_filter  true - enable high pass filter, otherwise disable the filter
    *                           When enabled, the gravity may not be filtered out.
    */
    int power_ctrl(operating_mode_t mode, bool low_pass_filter = true, bool high_pass_filter = false) {
        uint8_t value = mode;
        if (!low_pass_filter) {
            value |= 1 << 3;
        }
        if (!high_pass_filter) {
            value |= 1 << 2;
        }
        return write(ADXL372_POWER_CTL, value);
    }

    /* 
    * Set sample rate (aka Ouput Data Rate, ODR) and wakeup time 
    *
    * @param: rate  supported sample rates (Hz): RATE_400, RATE_800, RATE_1600, RATE_3200, RATE_6400
    * @param: ms    wakeup time, supported values (ms): MS52, MS104, MS208, MS512, MS2048, MS4096, MS8192, MS24576     
    */
    int timing_ctrl(rate_t rate, wakeup_time_t ms=MS52) {
        uint8_t value = (rate << TIMING_ODR_POS) | (ms << TIMING_WUR_POS);
        return write(ADXL372_TIMING, value);
    }

    int measurement_ctrl(bandwidth_t bandwidth, bool low_noise=true, uint8_t linkloop=0, bool autosleep=false) {
        uint8_t value = bandwidth;
        if (low_noise) {
            value |= 1 << 3;
        }
        if (linkloop) {
            value |= linkloop << 4;
        }
        if (autosleep) {
            value |= 1 << 6;
        }

        return write(ADXL372_MEASURE, value);
    }

    int reset(void) {
        return write(ADXL372_SRESET, 0x52);
    }

    uint32_t id() {
        uint32_t id;
        read(0x0, &id, 4);

        return id;
    }

    uint32_t status() {
        uint16_t value;
        read(ADXL372_STATUS_1, &value, 2);

        return value;
    }

    int fifo_ctrl(fifo_mode_t mode, fifo_format_t format=FIFO_XYZ, uint16_t samples_to_trigger=0x80) {
        write(ADXL372_FIFO_SAMPLES, samples_to_trigger & 0xFF);
        write(ADXL372_FIFO_CTL, (samples_to_trigger & 1) | (mode << 1) | (format << 3));
        return 0;
    }

    int fifo_read(void *ptr, uint16_t size) {
        uint8_t *buf = (uint8_t *)ptr;
        while (size > 32) {
            read(ADXL372_FIFO_DATA, buf, 32);
            size -= 32;
            buf += 32;
        }

        if (size) {
            read(ADXL372_FIFO_DATA, buf, size);
        }
        return size;
    }

    int samples_in_fifo() {
        uint16_t data = 0;
        read(ADXL372_FIFO_ENTRIES_2, &data, 2);
        return ((data & 0x3) << 8) | ((data >> 8) & 0xFF);
    }

    xyz_t *format(void *ptr) {
        uint8_t *buf = (uint8_t *)ptr;

        // for (uint8_t i=0; i<6; i++) {
        //     Serial.print(buf[i], HEX);
        //     Serial.print(' ');
        // }
        // Serial.println();

        for (uint8_t i=0; i<3; i++) {
            ((uint16_t *)buf)[i] = (buf[2*i] << 8) | buf[2*i+1];
            ((int16_t *)buf)[i] = ((int16_t *)buf)[i] >> 4;
        }

        return (xyz_t *)buf;
    }

    int read(xyz_t *xyz) {
        read(ADXL372_X_DATA_H, xyz, 6);
        format(xyz);

        return 0;
    }



    int setActiveThreshold(channel_t chan,uint16_t thres,bool activity_axis,bool regfer_or_abs)
    {
        uint8_t value[2] = {0};
        //bit [5:15] map to threshold register.
        value[0] = thres >> 3;
        value[1] |= (thres & 0x7)<<5;  
        value[1] |= activity_axis ;
        value[1] |= regfer_or_abs << 1;
        return write(ADXL372_X_THRESH_ACT_H+chan*2,value,2);
        
        write(ADXL372_X_THRESH_ACT_H+chan*2,value,2);

        // Serial.println(value[0],HEX);
        // Serial.println(value[1],HEX);

        // delay(300);
        // read(ADXL372_X_THRESH_ACT_H+chan*2,&value,2);
        // Serial.print("read data ");
        // Serial.print(value[0],HEX);
        // Serial.print(value[1],HEX);
        // Serial.print("from ");
        // Serial.println(ADXL372_X_THRESH_ACT_H+chan*2,HEX);
    }


    int setInactiveThreshold(channel_t chan,uint16_t thres,bool inactivity_axis,bool regfer_or_abs)
    {
        uint8_t value[2] = {0};
        //bit [5:15] map to threshold register.
        value[0] = thres >> 3;
        value[1] |= (thres & 0x7)<<5;  
        value[1] |= inactivity_axis ;
        value[1] |= regfer_or_abs << 1;
        return write(ADXL372_X_THRESH_ACT_H+chan*2,value,2);

        
    }


    /*@param Only sustained motion for a specified time can trigger activity detection.*/
    /*@return write result.*/
    int setActiveTime(uint8_t count)
    {
        uint8_t value = 0;
        // return write(ADXL372_TIME_ACT, count);
        write(ADXL372_TIME_ACT, count);
        
    }

    /*@param Only sustained motion for a specified time can trigger inactivity detection.*/
    /*@return write result.*/
    int setInactiveTime(uint16_t count)
    {
        uint8_t value[2] = {0};
        value[0] = count >> 8;
        value[1] = (uint8_t)count;
        return write(ADXL372_TIME_INACT_H,value,2);
    }


    /**Interrupt register config. 
     * @param chan : There are two channels:INT1 & INT2.
     * @config: config data,more detail below:
     * bit0:data ready        Map data ready interrupt onto INTX.
     * bit1:fifo ready        Map FIFO_READY interrupt onto INTX.
     * bit2:fifo full         Map FIFO_FULL interrupt onto INTX
     * bit3:fifo overflow     Map FIFO_OVERRUN interrupt onto INTX
     * bit4:inactive          Map inactivity interrupt onto INTX
     * bit5:active            Map activity interrupt onto INTX.
     * bit6:awake             Map awake interrupt onto INTX.
     * bit7:active low        Configures INT1 for active low operation
     * @return return write result.
    */
    int setIntConfig(IntPin_t chan,uint8_t config)
    {
        return write(ADXL372_INT1_MAP+chan,config);
    }
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/

    int read(uint8_t addr) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(addr);
        Wire.endTransmission(false);
        Wire.requestFrom(i2c_addr, (uint8_t)1);

        while (!Wire.available()) {
            
        }
        
        return Wire.read();
    }

    int read(uint8_t addr, void *ptr, uint8_t size) {
        uint8_t *buf = (uint8_t *)ptr;
        Wire.beginTransmission(i2c_addr);
        Wire.write(addr);
        Wire.endTransmission(false);
        Wire.requestFrom(i2c_addr, size);

        for (uint8_t i=0; i<size; i++) {
            while (!Wire.available()) {
            
            }
            buf[i] = Wire.read();
        }
        
        return size;
    }

    int write(uint8_t addr, uint8_t value) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(addr);
        Wire.write(value);
        Wire.endTransmission();

        return 0;
    }

    int write(uint8_t addr, void *ptr, uint8_t size) {
        uint8_t *buf = (uint8_t *)ptr;
        Wire.beginTransmission(i2c_addr);
        Wire.write(addr);
        for (uint8_t i=0; i<size; i++) {
            Wire.write(buf[i]);
        }
        Wire.endTransmission();

        return size;
    }

private:
    uint8_t i2c_addr;
};


#endif // __ADXL372_H__

