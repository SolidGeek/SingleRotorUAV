#include "config.h"

void Config::load(){

    EEPROM.get(0, params);

}

void Config::save(){

    EEPROM.put(0, params);
}