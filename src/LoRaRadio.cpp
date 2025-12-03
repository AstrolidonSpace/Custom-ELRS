#include "LoRaRadio.h"
#include <SPI.h>

// --- SX1280 Register Definitions (Simplified subset) ---
// These are essential opcodes and register addresses.
// A full list would be much larger from the SX1280 datasheet.
#define RADIO_SET_SLEEP                       0x84
#define RADIO_SET_STANDBY                     0x80
#define RADIO_SET_FS                          0xC1
#define RADIO_SET_TX                          0x83
#define RADIO_SET_RX                          0x82
#define RADIO_SET_RXDUTYCYCLE                 0x81
#define RADIO_SET_CAD                         0xC5
#define RADIO_SET_TXCONTINUOUSWAVE            0xD1
#define RADIO_SET_TXCONTINUOUSPRBL            0xD2
#define RADIO_SET_SETSWITCHMODE               0xD0
#define RADIO_SET_STOPRXTIMERONPREAMBLE       0x9F
#define RADIO_SET_STOPRXTIMERONSYNCWORD       0x9E
#define RADIO_SET_STOPRXTIMERONRSSI           0x9D

#define RADIO_GET_STATUS                      0xC0
#define RADIO_GET_RXBUFFERSTATUS              0x17
#define RADIO_GET_PACKETSTATUS                0x1D
#define RADIO_GET_RSSIINST                    0x1F
#define RADIO_GET_STATS                       0x21

#define RADIO_WRITE_REGISTER                  0x0D
#define RADIO_READ_REGISTER                   0x1C

#define RADIO_WRITE_BUFFER                    0x0E
#define RADIO_READ_BUFFER                     0x1E

#define RADIO_SET_DIOIRQPARAMS                0x08
#define RADIO_GET_IRQSTATUS                   0x18
#define RADIO_CLR_IRQSTATUS                   0x07

#define RADIO_SET_RFFREQUENCY                 0x86
#define RADIO_SET_PACKETTYPE                  0x8A
#define RADIO_SET_MODULATIONPARAMS            0x8B
#define RADIO_SET_PACKETPARAMS                0x8C
#define RADIO_SET_TXPARAMS                    0x8E
#define RADIO_SET_BUFFERBASEADDRESS           0x8F
#define RADIO_SET_LORA_SYMB_NUM_TIMEOUT       0xA8

// Packet Types
#define PACKET_TYPE_GFSK 0x00
#define PACKET_TYPE_LORA 0x01
#define PACKET_TYPE_RANGING 0x02

// Standby Modes
#define STDBY_RC   0x00 // Standby RC
#define STDBY_XOSC 0x01 // Standby XOSC

// IRQ Masks (a subset)
#define IRQ_TX_DONE             0x0001
#define IRQ_RX_DONE             0x0002
#define IRQ_CRC_ERROR           0x0020
#define IRQ_HEADER_ERROR        0x0010
#define IRQ_TIMEOUT             0x0004
#define IRQ_PREAMBLE_DETECTED   0x0008
#define IRQ_SYNCWORD_VALID      0x0008 // For specific packet types
#define IRQ_ALL                 0xFFFF

// LoRa Spreading Factors (SF)
#define LORA_SF5  0x05
#define LORA_SF6  0x06
#define LORA_SF7  0x07
#define LORA_SF8  0x08
#define LORA_SF9  0x09
#define LORA_SF10 0x0A
#define LORA_SF11 0x0B
#define LORA_SF12 0x0C

// LoRa Bandwidths
#define LORA_BW_200 0x0A // 203.125 kHz
#define LORA_BW_400 0x08 // 406.25 kHz
#define LORA_BW_800 0x06 // 812.5 kHz
#define LORA_BW_1600 0x04 // 1625 kHz
#define LORA_BW_3200 0x02 // 3250 kHz
#define LORA_BW_6400 0x00 // 6500 kHz

// Coding Rates
#define LORA_CR_4_5 0x01
#define LORA_CR_4_6 0x02
#define LORA_CR_4_7 0x03
#define LORA_CR_4_8 0x04

// TX Power Lookup Table (Simplified, real SX1280 has complex power steps)
// These values are internal to the SX1280's SetTxParams command.
// See SX1280 datasheet section 10.3.3 for actual power values vs. register settings.
// This is a placeholder, actual dBm values will vary.
const int8_t SX1280_TX_POWER_LOOKUP[] = {
    -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
}; // Maps roughly from -18dBm to +13dBm

// Global instance
LoRaRadio Radio;

// ISR for DIO1
void ICACHE_RAM_ATTR SX1280_DIO1_ISR()
{
    // Important: Inside ISRs, keep code minimal and fast.
    // We get IRQ status and clear it, then call the appropriate callback.
    uint16_t irqStatus = Radio.GetIrqStatus();
    Radio.ClearIrqStatus(irqStatus);

    if (irqStatus & IRQ_TX_DONE)
    {
        if (Radio.TXdoneCallback)
        {
            Radio.TXdoneCallback();
        }
    }
    if (irqStatus & IRQ_RX_DONE)
    {
        if (Radio.RXdoneCallback)
        {
            Radio.RXdoneCallback();
        }
    }
    // Handle other IRQs if necessary (e.g., CRC_ERROR, TIMEOUT)
    if (irqStatus & IRQ_CRC_ERROR) {
        Serial.println("SX1280: CRC Error!");
    }
    if (irqStatus & IRQ_TIMEOUT) {
        // This is important if RX or TX timeouts are used.
        // For non-blocking, we might rely more on the main loop's connection lost timeout.
    }
}


LoRaRadio::LoRaRadio()
{
    radioSPI = &SPI; // Use default SPI instance
    _nssPin = SX1280_NSS;
    _busyPin = SX1280_BUSY;
    _rstPin = SX1280_RST;
    _dio1Pin = SX1280_DIO1;
    // Other DIOs if needed for more complex interrupts
    RXdoneCallback = nullptr;
    TXdoneCallback = nullptr;
}

void LoRaRadio::SPIwrite(uint8_t *data, uint8_t len)
{
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(data, len);
    digitalWrite(_nssPin, HIGH);
}

void LoRaRadio::SPIread(uint8_t *data, uint8_t len)
{
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(data, len); // Send zeros to clock in data
    digitalWrite(_nssPin, HIGH);
}

void LoRaRadio::SendOpcode(uint8_t opcode, uint8_t *data, uint8_t len)
{
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(opcode);
    if (len > 0)
    {
        radioSPI->transfer(data, len);
    }
    digitalWrite(_nssPin, HIGH);
}

void LoRaRadio::ReadRegisters(uint16_t address, uint8_t *data, uint8_t len)
{
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_READ_REGISTER);
    radioSPI->transfer((uint8_t)(address >> 8)); // MSB
    radioSPI->transfer((uint8_t)(address & 0xFF)); // LSB
    radioSPI->transfer(0x00); // Dummy byte for SX1280 read
    radioSPI->transfer(data, len);
    digitalWrite(_nssPin, HIGH);
}

void LoRaRadio::WriteRegisters(uint16_t address, uint8_t *data, uint8_t len)
{
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_WRITE_REGISTER);
    radioSPI->transfer((uint8_t)(address >> 8)); // MSB
    radioSPI->transfer((uint8_t)(address & 0xFF)); // LSB
    radioSPI->transfer(data, len);
    digitalWrite(_nssPin, HIGH);
}

uint8_t LoRaRadio::ReadRegister(uint16_t address)
{
    uint8_t value;
    ReadRegisters(address, &value, 1);
    return value;
}

void LoRaRadio::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegisters(address, &value, 1);
}

uint8_t LoRaRadio::GetStatus()
{
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    uint8_t status = radioSPI->transfer(RADIO_GET_STATUS);
    digitalWrite(_nssPin, HIGH);
    return status;
}

void LoRaRadio::WaitOnBusy()
{
    while (digitalRead(_busyPin) == HIGH)
    {
        // Wait for busy pin to go low
    }
}

void LoRaRadio::RadioReset()
{
    digitalWrite(_rstPin, LOW);
    delay(20); // Short pulse
    digitalWrite(_rstPin, HIGH);
    delay(50); // Wait for chip to wake up
}

void LoRaRadio::SetStandby(uint8_t mode)
{
    SendOpcode(RADIO_SET_STANDBY, &mode, 1);
}

void LoRaRadio::SetPacketType(uint8_t type)
{
    SendOpcode(RADIO_SET_PACKETTYPE, &type, 1);
}

void LoRaRadio::SetRfFrequency(uint32_t frequency)
{
    // Frequency is in Hz, SX1280 expects in steps of F_XOSC / 2^16
    // F_XOSC = 32MHz, so step is 32,000,000 / 65536 = ~488.28125 Hz
    // To convert Hz to register value: freq_reg = freq_hz / (F_XOSC / 2^16) = freq_hz * 2^16 / F_XOSC
    // Or simpler: freq_reg = freq_hz / (32000000.0 / 65536.0)
    uint32_t freq_reg = (uint32_t)((double)frequency / (32000000.0 / 65536.0));
    uint8_t data[3];
    data[0] = (uint8_t)(freq_reg >> 16);
    data[1] = (uint8_t)(freq_reg >> 8);
    data[2] = (uint8_t)(freq_reg & 0xFF);
    SendOpcode(RADIO_SET_RFFREQUENCY, data, 3);
}

void LoRaRadio::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t data[2] = {txBaseAddress, rxBaseAddress};
    SendOpcode(RADIO_SET_BUFFERBASEADDRESS, data, 2);
}

void LoRaRadio::SetModulationParams(uint8_t packetType, uint32_t spreadingFactor, uint32_t bandwidth, uint8_t codingRate)
{
    // spreadingFactor: 0x05 to 0x0C for SF5 to SF12
    // bandwidth: LORA_BW_200 to LORA_BW_6400 (values like 0x0A, 0x08, etc.)
    // codingRate: LORA_CR_4_5 to LORA_CR_4_8
    uint8_t data[4] = {packetType, (uint8_t)spreadingFactor, (uint8_t)bandwidth, codingRate};
    SendOpcode(RADIO_SET_MODULATIONPARAMS, data, 4);
}

void LoRaRadio::SetPacketParams(uint8_t packetType, uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIQ)
{
    // preambleLength: for LoRa, 0x00 to 0xFF (up to 255 symbols). Usually 0x06 (6 symbols) for implicit.
    // headerType: 0x00 (Explicit) or 0x01 (Implicit)
    // crcType: 0x00 (no CRC), 0x01 (CRC on header), 0x02 (CRC on payload)
    // invertIQ: 0x00 (Standard), 0x01 (Inverted)
    uint8_t data[7] = {packetType, (uint8_t)(preambleLength >> 8), (uint8_t)(preambleLength & 0xFF),
                       headerType, payloadLength, crcType, invertIQ};
    SendOpcode(RADIO_SET_PACKETPARAMS, data, 7);
}

void LoRaRadio::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t data[8];
    data[0] = (uint8_t)(irqMask >> 8);
    data[1] = (uint8_t)(irqMask & 0xFF);
    data[2] = (uint8_t)(dio1Mask >> 8);
    data[3] = (uint8_t)(dio1Mask & 0xFF);
    data[4] = (uint8_t)(dio2Mask >> 8);
    data[5] = (uint8_t)(dio2Mask & 0xFF);
    data[6] = (uint8_t)(dio3Mask >> 8);
    data[7] = (uint8_t)(dio3Mask & 0xFF);
    SendOpcode(RADIO_SET_DIOIRQPARAMS, data, 8);
}

uint16_t LoRaRadio::GetIrqStatus()
{
    uint8_t data[2];
    SendOpcode(RADIO_GET_IRQSTATUS, nullptr, 0); // Need to get status bytes back
    WaitOnBusy(); // Status is returned after opcode, so need to wait for busy for next command
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_GET_IRQSTATUS); // Resend opcode to get the actual value back
    data[0] = radioSPI->transfer(0x00);
    data[1] = radioSPI->transfer(0x00);
    digitalWrite(_nssPin, HIGH);
    return ((uint16_t)data[0] << 8) | data[1];
}

void LoRaRadio::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t data[2];
    data[0] = (uint8_t)(irqMask >> 8);
    data[1] = (uint8_t)(irqMask & 0xFF);
    SendOpcode(RADIO_CLR_IRQSTATUS, data, 2);
}

void LoRaRadio::SetTx(uint32_t timeout)
{
    // Timeout in units of 16 us
    uint32_t timeout_reg = timeout / 16;
    if (timeout == 0) timeout_reg = 0x00FFFFFF; // Infinite timeout
    uint8_t data[3] = {(uint8_t)(timeout_reg >> 16), (uint8_t)(timeout_reg >> 8), (uint8_t)(timeout_reg & 0xFF)};
    SendOpcode(RADIO_SET_TX, data, 3);
}

void LoRaRadio::SetRx(uint32_t timeout)
{
    // Timeout in units of 16 us
    uint32_t timeout_reg = timeout / 16;
    if (timeout == 0) timeout_reg = 0x00FFFFFF; // Infinite timeout
    uint8_t data[3] = {(uint8_t)(timeout_reg >> 16), (uint8_t)(timeout_reg >> 8), (uint8_t)(timeout_reg & 0xFF)};
    SendOpcode(RADIO_SET_RX, data, 3);
}

void LoRaRadio::GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_GET_RXBUFFERSTATUS);
    uint8_t data[2];
    data[0] = radioSPI->transfer(0x00); // Payload Length
    data[1] = radioSPI->transfer(0x00); // Start Buffer Pointer
    digitalWrite(_nssPin, HIGH);
    *payloadLength = data[0];
    *rxStartBufferPointer = data[1];
}

void LoRaRadio::GetPacketStatus(int8_t *rssi, int8_t *snr)
{
    WaitOnBusy();
    uint8_t data[3];
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_GET_PACKETSTATUS);
    data[0] = radioSPI->transfer(0x00); // RSSI
    data[1] = radioSPI->transfer(0x00); // SNR
    data[2] = radioSPI->transfer(0x00); // Packet Error Rate
    digitalWrite(_nssPin, HIGH);

    *rssi = -data[0] / 2; // RSSI value is typically negative and needs scaling
    *snr = data[1] / 4;   // SNR value needs scaling
}

void LoRaRadio::GetRssiInst(int8_t *rssi)
{
    WaitOnBusy();
    uint8_t data[1];
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_GET_RSSIINST);
    data[0] = radioSPI->transfer(0x00); // RSSI value
    digitalWrite(_nssPin, HIGH);
    *rssi = -data[0] / 2;
}

void LoRaRadio::Begin()
{
    // Pin setup
    pinMode(_nssPin, OUTPUT);
    pinMode(_rstPin, OUTPUT);
    pinMode(_busyPin, INPUT);
    pinMode(_dio1Pin, INPUT_PULLUP); // DIO1 as input for interrupts

    digitalWrite(_nssPin, HIGH); // De-select radio
    RadioReset(); // Perform a hardware reset

    // Initialize SPI
    radioSPI->begin(SX1280_SCK, SX1280_MISO, SX1280_MOSI, _nssPin);
    radioSPI->setBitOrder(MSBFIRST);
    radioSPI->setDataMode(SPI_MODE0);
    radioSPI->setFrequency(8000000); // 8MHz SPI clock speed

    // Attach interrupt for DIO1 (TX_DONE, RX_DONE)
    attachInterrupt(digitalPinToInterrupt(_dio1Pin), SX1280_DIO1_ISR, RISING);

    // Initial configuration commands (basic setup for LoRa)
    SetStandby(STDBY_XOSC); // Start with crystal oscillator
    SetPacketType(PACKET_TYPE_LORA); // Set LoRa packet type

    // Set buffer base addresses (TX and RX buffers start at 0)
    SetBufferBaseAddress(0x00, 0x00);

    // Configure DIO1 for TX_DONE and RX_DONE interrupts
    SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_HEADER_ERROR | IRQ_TIMEOUT,
                    IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_HEADER_ERROR | IRQ_TIMEOUT, // Map to DIO1
                    0x0000, 0x0000); // DIO2, DIO3 masks (not used here)

    ClearIrqStatus(IRQ_ALL); // Clear any pending IRQs
    Serial.println("SX1280: Radio initialized.");
}

void LoRaRadio::Config(uint32_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint16_t preambleLen)
{
    SetStandby(STDBY_XOSC);
    ClearIrqStatus(IRQ_ALL);

    // Set modulation parameters
    SetModulationParams(PACKET_TYPE_LORA, sf, bw, cr);

    // Set packet parameters (headerType 0x00 = Explicit, 0x01 = Implicit)
    // For ELRS, payloadLength is typically fixed (8 bytes for RC, variable for others)
    // CRC type is usually 0x02 (on payload), InvertIQ 0x00 (standard)
    SetPacketParams(PACKET_TYPE_LORA, preambleLen, 0x00, 8, 0x02, 0x00); // Assuming 8-byte RC data, Explicit header, CRC on payload

    SetRfFrequency(freq);
    currFreq = freq;
    Serial.print("SX1280: Configured BW="); Serial.print(bw);
    Serial.print(" SF="); Serial.print(sf);
    Serial.print(" CR="); Serial.print(cr);
    Serial.print(" Freq="); Serial.println(freq);
}

void LoRaRadio::SetFrequency(uint32_t freq)
{
    SetStandby(STDBY_XOSC); // Ensure in standby before changing frequency
    SetRfFrequency(freq);
    currFreq = freq;
}

void LoRaRadio::SetTxPower(int8_t power_idx)
{
    // SX1280 has SetTxParams(rampTime, power).
    // power: 0x03 (-18dBm) to 0x1F (+13dBm).
    // rampTime: 0x00 to 0x06 (e.g., 0x06 is 200us)
    uint8_t rampTime = 0x04; // 80us ramp time (common)
    uint8_t txPowerValue = 0x1F; // Max power for now (+13dBm)

    // A more accurate mapping would involve the SX1280_TX_POWER_LOOKUP.
    // For simplicity, let's just set the highest common power if power_idx corresponds to it.
    // If power_idx is PWR_100mW, PWR_250mW, etc., you'd map it to the nearest actual SX1280 power level.
    // This example just sets max power regardless of input, you'll need a proper mapping.
    // Example: map desired dBm to 0x03-0x1F range.

    if (power_idx >= PWR_100mW) { // Example threshold
        txPowerValue = 0x1F; // Max 13dBm for SX1280
    } else {
        txPowerValue = 0x0B; // Example: 0 dBm
    }

    uint8_t data[2] = {rampTime, txPowerValue};
    SendOpcode(RADIO_SET_TXPARAMS, data, 2);
    Serial.print("SX1280: TX Power set to value "); Serial.println(txPowerValue);
}


void LoRaRadio::TXnb(uint8_t *data, uint8_t len)
{
    SetStandby(STDBY_XOSC); // Ensure in standby
    ClearIrqStatus(IRQ_ALL); // Clear all IRQs before TX

    // Write payload to TX buffer
    WaitOnBusy();
    digitalWrite(_nssPin, LOW);
    radioSPI->transfer(RADIO_WRITE_BUFFER);
    radioSPI->transfer(0x00); // Buffer offset (start at 0)
    radioSPI->transfer(data, len); // Write data
    digitalWrite(_nssPin, HIGH);

    // Set TX mode (with infinite timeout, TX_DONE interrupt will handle next step)
    SetTx(0); // 0 = infinite timeout, relies on IRQ_TX_DONE
}

void LoRaRadio::RXnb()
{
    SetStandby(STDBY_XOSC); // Ensure in standby
    ClearIrqStatus(IRQ_ALL); // Clear all IRQs before RX

    // Set RX mode (with infinite timeout, RX_DONE interrupt will handle next step)
    SetRx(0); // 0 = infinite timeout, relies on IRQ_RX_DONE
}