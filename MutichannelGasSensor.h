/*
    MutichannelGasSensor.h
    2015 Copyright (c) Seeed Technology Inc.  All right reserved.

    Author: Jacky Zhang
    2015-3-17
    http://www.seeed.cc/
    modi by Jack, 2015-8

    V2 by Loovee
    2016-11-11

    The MIT License (MIT)

    Copyright (c) 2015 Seeed Technology Inc.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef __MUTICHANNELGASSENSOR_H__
#define __MUTICHANNELGASSENSOR_H__

#if defined (ARDUINO_SAMD_ZERO)
    #define _SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_SEEED_STM32F4)
    #define _SERIAL SerialUSB
#elif defined (ARDUINO_ARCH_SAMD)
    #define _SERIAL Serial
#elif defined (ARDUINO_ARCH_AVR)
    #define _SERIAL Serial
#else
    #warning "Architecture not matched"
    #define _SERIAL Serial
#endif


#define DEFAULT_I2C_ADDR    0x04

#define ADDR_IS_SET             0           // if this is the first time to run, if 1126, set 
#define ADDR_FACTORY_ADC_NH3    2
#define ADDR_FACTORY_ADC_CO     4
#define ADDR_FACTORY_ADC_NO2    6

#define ADDR_USER_ADC_HN3       8
#define ADDR_USER_ADC_CO        10
#define ADDR_USER_ADC_NO2       12
#define ADDR_IF_CALI            14          // IF USER HAD CALI

#define ADDR_I2C_ADDRESS        20

#define CH_VALUE_NH3            1
#define CH_VALUE_CO             2
#define CH_VALUE_NO2            3

#define CMD_ADC_RES0            1           // NH3
#define CMD_ADC_RES1            2           // CO
#define CMD_ADC_RES2            3           // NO2
#define CMD_ADC_RESALL          4           // ALL CHANNEL
#define CMD_CHANGE_I2C          5           // CHANGE I2C
#define CMD_READ_EEPROM         6           // READ EEPROM VALUE, RETURN UNSIGNED INT
#define CMD_SET_R0_ADC          7           // SET R0 ADC VALUE
#define CMD_GET_R0_ADC          8           // GET R0 ADC VALUE
#define CMD_GET_R0_ADC_FACTORY  9           // GET FACTORY R0 ADC VALUE
#define CMD_CONTROL_LED         10
#define CMD_CONTROL_PWR         11

#define CMD_ADC_RES0_DATUM      0x11        // NH3
#define CMD_ADC_RES1_DATUM      0x12        // CO
#define CMD_ADC_RES2_DATUM      0x13        // NO2


enum {CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH};

class MutichannelGasSensor {

  private:

    int __version;
    unsigned char dta_test[20];

    unsigned int readChAdcValue(int ch);


  public:

    uint8_t i2cAddress;     //I2C address of this MCU
    uint16_t res0[3];       //sensors res0  //not used in V2 sensors
    uint16_t res[3];        //sensors res   //not used in V2 sensors

    float ratio0, ratio1, ratio2;           //used for all sensors

    bool r0_inited;

    MutichannelGasSensor();

    unsigned int get_addr_dta(const uint8_t *addr_str, size_t len);
    unsigned int get_addr_dta(unsigned char addr_reg);
    unsigned int get_addr_dta(unsigned char addr_reg, unsigned char __dta);

    inline void write_i2c(unsigned char addr, const unsigned char* dta, unsigned char dta_len);

    void sendI2C(unsigned char dta);
    void sendI2C(const unsigned char *dta, unsigned char dta_len);


    int16_t readData(uint8_t cmd);
    int16_t readR0(void);
    int16_t readR(void);
    bool sampleSensor();
    float calcGas(int gas);

  public:

    void begin(int address);
    void begin();
    void changeI2cAddr(uint8_t newAddr);
    void powerOn(void);
    void powerOff(void);
    void doCalibrate(void);

    //get gas concentration, unit: ppm
    float measure_CO() {
        return pow(ratio1, -1.179) * 4.385;
    }
    float measure_NO2() {
        return pow(ratio2, 1.007) / 6.855;
    }
    float measure_NH3() {
        return pow(ratio0, -1.67) / 1.47;
    }
    float measure_C3H8() {
        return pow(ratio0, -2.518) * 570.164;
    }
    float measure_C4H10() {
        return pow(ratio0, -2.138) * 398.107;
    }
    float measure_CH4() {
        return pow(ratio1, -4.363) * 630.957;
    }
    float measure_H2() {
        return pow(ratio1, -1.8) * 0.73;
    }
    float measure_C2H5OH() {
        return pow(ratio1, -1.552) * 1.622;
    }

    float getR0(unsigned char ch);      // 0:CH3, 1:CO, 2:NO2
    float getRs(unsigned char ch);      // 0:CH3, 1:CO, 2:NO2

  public:

    void ledOn() {
        dta_test[0] = CMD_CONTROL_LED;
        dta_test[1] = 1;
        write_i2c(i2cAddress, dta_test, 2);
    }

    void ledOff() {
        dta_test[0] = CMD_CONTROL_LED;
        dta_test[1] = 0;
        write_i2c(i2cAddress, dta_test, 2);
    }

    void display_eeprom();
    void factory_setting();
    void change_i2c_address(unsigned char addr);
    unsigned char getVersion();
};

//extern MutichannelGasSensor gas;

#endif

/*********************************************************************************************************
    END FILE
*********************************************************************************************************/