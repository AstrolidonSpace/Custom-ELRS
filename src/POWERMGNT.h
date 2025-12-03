#pragma once

#include <Arduino.h>
#include "Definitions.h" // For PowerLevels_e and other global enums
#include "LoRaRadio.h"   // To access the global 'Radio' object for SetTxPower (our custom driver)

class POWERMGNT
{
public:
    POWERMGNT(); // Constructor
    void init();
    void setDefaultPower();
    PowerLevels_e setPower(PowerLevels_e Power);
    PowerLevels_e incPower();
    PowerLevels_e decPower();
    PowerLevels_e currPower();

private:
    static PowerLevels_e CurrentPower;
    // MaxPower and DefaultPowerEnum are #defined in Definitions.h or platformio.ini now
};