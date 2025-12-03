#include "CRCExtra.h"

GENERIC_CRC8::GENERIC_CRC8(uint8_t polynomial) : _polynomial(polynomial)
{
    generateTable();
}

void GENERIC_CRC8::generateTable()
{
    for (int i = 0; i < 256; ++i)
    {
        uint8_t crc = i;
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ _polynomial;
            }
            else
            {
                crc <<= 1;
            }
        }
        _crcTable[i] = crc;
    }
}

uint8_t GENERIC_CRC8::calc(const uint8_t *data, size_t length)
{
    uint8_t crc = 0; // Initial CRC value for ELRS
    for (size_t i = 0; i < length; ++i)
    {
        crc = _crcTable[crc ^ data[i]];
    }
    return crc;
}