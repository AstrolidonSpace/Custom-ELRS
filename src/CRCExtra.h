#pragma once

#include <Arduino.h>

class GENERIC_CRC8
{
public:
    GENERIC_CRC8(uint8_t polynomial);
    uint8_t calc(const uint8_t *data, size_t length);

private:
    uint8_t _polynomial;
    uint8_t _crcTable[256];
    void generateTable();
};

// Define the CRC polynomial for ELRS. This is a standard CRC-8.
#ifndef ELRS_CRC_POLY
#define ELRS_CRC_POLY 0xD5 // This is the polynomial used by ExpressLRS
#endif