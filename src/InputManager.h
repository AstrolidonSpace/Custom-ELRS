#pragma once

#include <Arduino.h>
#include "Definitions.h" // For pin definitions

class InputManager
{
public:
    InputManager();
    void init(); // Initialize pins
    void readInputs(uint16_t *channelData); // Read all inputs and populate the array

private:
    // No private members needed if directly reading from global pins
};