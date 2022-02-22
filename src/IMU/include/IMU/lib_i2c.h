//
// Created by nico on 5/11/20.
//

#ifndef IMU_i2c_H
#define IMU_i2c_H

#include "raspi_batmon/registre.h"
#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <vector>
#include <string>
#include <sstream>
#include <ros/console.h>

class i2c {
public:
    batmon_driver(int adapter_nr, int addr);

    float getAres(uint8_t Ascale); 

    float getGres(uint8_t Gscale) ;

    void reset();

    uint8_t status();

    void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);

    void offsetBias(float * dest1, float * dest2);

    void readData(int16_t * destination);
     
    float _aRes, _gRes;

//    void setup(int adapter_nr);
private:
    int read_8bit_word(int reg);
    int write_8bit_word(int reg, int data);

//    int file;
    int file_num;
    int addr;
    

};

#endif //RASPI_BATMON_i2c_H
