#include "InputManager.h"

InputManager::InputManager()
{
    // Constructor, nothing specific needed here if init() handles setup
}

void InputManager::init()
{
    // Already defined in main.cpp's setup, but good to have a dedicated init here for consistency.
    // For now, these are commented out to avoid redeclaration if main.cpp already sets them.
    // pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
    // pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
    // pinMode(ANALOG_CHANNEL_3_PIN, INPUT);
    // pinMode(ANALOG_CHANNEL_4_PIN, INPUT);

    // pinMode(DIGITAL_SWITCH_1_PIN, INPUT_PULLUP);
    // pinMode(DIGITAL_SWITCH_2_PIN, INPUT_PULLUP);
}

void InputManager::readInputs(uint16_t *channelData)
{
    // This function can be called to read and populate the channelData array.
    // However, in our main.cpp, the reading is done directly within SendRCdataToRF() for ICACHE_RAM_ATTR.
    // You could move the readAnalogChannels() and readDigitalSwitches() functions into this class
    // as public methods if you prefer a more object-oriented approach for them.
    // For current setup, the calls in main.cpp are sufficient.
}