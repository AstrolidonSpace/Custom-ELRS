#pragma once

#include <Arduino.h>
#include <SPI.h> // For SPI communication
#include "Definitions.h" // For pin definitions and other global constants

// Forward declarations for ISRs
void ICACHE_RAM_ATTR SX1280_DIO1_ISR();

class LoRaRadio
{
public:
    uint8_t RXdataBuffer[255]; // Max packet size for LoRa
    uint8_t TXdataBuffer[255];
    int8_t LastPacketRSSI;
    int8_t LastPacketSNR;
    uint32_t currFreq;

    // Callbacks for TX/RX completion
    void (*RXdoneCallback)();
    void (*TXdoneCallback)();

    LoRaRadio();
    void Begin();
    void Config(uint32_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint16_t preambleLen);
    void SetFrequency(uint32_t freq);
    void TXnb(uint8_t *data, uint8_t len); // Non-blocking Transmit
    void RXnb();                         // Non-blocking Receive
    void SetTxPower(int8_t power_idx);

    // SX1280 Specific methods (often private in full drivers, public for simplicity here)
    void SendOpcode(uint8_t opcode, uint8_t *data, uint8_t len);
    void ReadRegisters(uint16_t address, uint8_t *data, uint8_t len);
    void WriteRegisters(uint16_t address, uint8_t *data, uint8_t len);
    uint8_t ReadRegister(uint16_t address);
    void WriteRegister(uint16_t address, uint8_t value);
    uint8_t GetStatus();
    void SetStandby(uint8_t mode); // STDBY_RC or STDBY_XOSC
    void SetPacketType(uint8_t type); // PACKET_TYPE_LORA
    void SetRfFrequency(uint32_t frequency);
    void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void SetModulationParams(uint8_t packetType, uint32_t spreadingFactor, uint32_t bandwidth, uint8_t codingRate);
    void SetPacketParams(uint8_t packetType, uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIQ);
    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    uint16_t GetIrqStatus();
    void ClearIrqStatus(uint16_t irqMask);
    void SetTx(uint32_t timeout); // timeout in ms for TX, or 0 for infinite
    void SetRx(uint32_t timeout); // timeout in ms for RX, or 0 for infinite
    void GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer);
    void GetRssiInst(int8_t *rssi); // Get current RSSI
    void GetPacketStatus(int8_t *rssi, int8_t *snr); // Get RSSI/SNR of last packet

private:
    SPIClass *radioSPI;
    int _nssPin;
    int _busyPin;
    int _rstPin;
    int _dio1Pin; // DIO1 is often used for TX_DONE and RX_DONE interrupts
    int _dio2Pin; // Optional, if needed for other interrupts
    int _dio3Pin; // Optional, if needed for other interrupts

    void SPIwrite(uint8_t *data, uint8_t len);
    void SPIread(uint8_t *data, uint8_t len);
    void WaitOnBusy();
    void RadioReset();
};

extern LoRaRadio Radio; // Global instance