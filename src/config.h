#ifndef _SRUAV_CONFIG_H
#define _SRUAV_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include "stdint.h"

class Config
{
public:

    struct config
    {
        int16_t servo_offset[4];
    } params;
    

    void load();

    void save();

private: 

    uint16_t address = 0x00;
 
};


#endif