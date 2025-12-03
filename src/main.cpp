#include <Arduino.h>
#include "Definitions.h" // Our custom definitions and global structs
#include "LoRaRadio.h"   // Our custom SX1280 driver
#include "common.h"      // Original ELRS common definitions
#include "FIFO.h"        // Assuming FIFO.h/cpp is in src
#include "utils.h"       // Assuming utils.h/cpp is in src
#include "FHSS.h"        // Assuming FHSS.h/cpp is in src
//#include "LED.h"         // Assuming LED.h/cpp is in src (for updateLEDs)
#include "targets.h"     // Original ELRS targets (might define some pins)
#include "POWERMGNT.h"   // Original ELRS power management
#include "msp.h"         // Original ELRS MSP (for VTX config, etc.)
#include "msptypes.h"    // Original ELRS MSP types
//#include <OTA.h>         // Original ELRS OTA (for WebUpdate)
#include "elrs_eeprom.h" // Original ELRS EEPROM handling
#include "config.h"      // Original ELRS configuration saving
#include "hwTimer.h"     // Original ELRS hardware timer
#include "LQCALC.h"      // Original ELRS Link Quality calculation
#include "LowPassFilter.h" // Original ELRS Low Pass Filter
#include "CRCExtra.h"

#ifdef PLATFORM_ESP32
//#include "ESP32_WebUpdate.h" // For WiFi web update
#include "esp_mac.h"  // For esp_read_mac
#include <esp_system.h>
#endif

//// GLOBAL INSTANCES ////
hwTimer hwTimer;
GENERIC_CRC8 ota_crc(ELRS_CRC_POLY);
POWERMGNT POWERMGNT;
MSP msp; // For MSP processing (e.g., VTX config)
ELRS_EEPROM eeprom;
Config config;

// Global variables (from Definitions.h externs or common.h externs)
connectionState_e connectionState = disconnected;
extern uint8_t DeviceAddr; // Will be set based on UID
uint8_t baseMac[6]; // For ESP32 MAC address, to generate DeviceAddr/UID if not defined
volatile uint8_t NonceTX = 0;
// RC Channel Data (Analog & Digital Inputs)
uint16_t ChannelData[8]; // Channels 0-3 for analog, 4-5 for digital switches (0-2047 for 11-bit)

// Telemetry
uint32_t LastTLMpacketRecvMillis = 0;
LQCALC LQCALC;
LPF LPD_DownlinkLQ(1);

// MSP Data Handling (for VTX config packets, etc.)
uint32_t MSPPacketLastSent = 0; // time in ms when the last MSP packet was sent
uint32_t MSPPacketSendCount = 0; // number of times to send MSP packet
mspPacket_t MSPPacket;

// Sync Packet
uint32_t SyncPacketLastSent = 0;

// Update Parameters
bool webUpdateMode = false;
bool bindMode = false;
volatile bool UpdateParamReq = false; // Triggered internally or by a button for param updates
#define SERIAL_TELEMETRY_UPDATE_INTERVAL 1000 // How often to print telemetry to serial
uint32_t LastTelemetryPrinted = 0;

uint32_t PacketLastSentMicros = 0; // Last time an RF packet was sent

bool WaitRXresponse = false; // Flag to indicate we are waiting for a telemetry response
bool WaitEepromCommit = false; // Flag to trigger EEPROM commit in main loop

// Function Prototypes
void ICACHE_RAM_ATTR TimerCallbackISR();
void ICACHE_RAM_ATTR ProcessTLMpacket();
void ICACHE_RAM_ATTR ReadAnalogChannels();
void ICACHE_RAM_ATTR ReadDigitalSwitches();
void ICACHE_RAM_ATTR GenerateSyncPacketData();
void ICACHE_RAM_ATTR GenerateRCDataPacket_11bit();
void ICACHE_RAM_ATTR GenerateMSPData();
void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index);
void ICACHE_RAM_ATTR HandleFHSS();
void ICACHE_RAM_ATTR HandleTLM();
void ICACHE_RAM_ATTR SendRCdataToRF();
void HandleUpdateParameter();
void ProcessMSPPacket(mspPacket_t *packet);
void OnRFModePacket(mspPacket_t *packet);
void OnTxPowerPacket(mspPacket_t *packet);
void OnTLMRatePacket(mspPacket_t *packet);


// --- Custom Input Reading Functions ---
void ICACHE_RAM_ATTR ReadAnalogChannels()
{
    // Read analog values (0-4095 for ESP32 ADC) and map to 0-2047 for 11-bit ELRS channels
    ChannelData[0] = map(analogRead(ANALOG_CHANNEL_1_PIN), 0, 4095, 0, 2047);
    ChannelData[1] = map(analogRead(ANALOG_CHANNEL_2_PIN), 0, 4095, 0, 2047);
    ChannelData[2] = map(analogRead(ANALOG_CHANNEL_3_PIN), 0, 4095, 0, 2047);
    ChannelData[3] = map(analogRead(ANALOG_CHANNEL_4_PIN), 0, 4095, 0, 2047);
}

void ICACHE_RAM_ATTR TimerCallbackISR()
{
    SendRCdataToRF();
    PacketLastSentMicros = micros();
}

void ICACHE_RAM_ATTR ReadDigitalSwitches()
{
    // Read digital switch states. Assuming HIGH = ON (2047), LOW = OFF (0).
    // Uses INPUT_PULLUP, so HIGH means switch is open, LOW means switch is closed to GND.
    // Adjust logic based on your switch wiring (NO/NC).
    ChannelData[4] = digitalRead(DIGITAL_SWITCH_1_PIN) == LOW ? 2047 : 0; // Assuming pullup and switch closes to GND
    ChannelData[5] = digitalRead(DIGITAL_SWITCH_2_PIN) == LOW ? 2047 : 0; // Assuming pullup and switch closes to GND
}


// --- Telemetry Processing ---
void ICACHE_RAM_ATTR ProcessTLMpacket()
{
    // Important: For SX1280, CRC is often handled by the chip.
    // If ELRS uses an application-level CRC in the payload, calculate it here.
    // The provided SX1280Driver assumes CRC is handled by SX1280 SetPacketParams(..., crcType=0x02)
    // and relies on IRQ_CRC_ERROR for bad packets.
    // This part of the code relies on RXdataBuffer and a custom CRC check if needed.
    // For now, keeping the original CRC check as it might be an ELRS application-level CRC.

    uint8_t calculatedCRC = ota_crc.calc(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
    uint8_t inCRC = Radio.RXdataBuffer[7];
    uint8_t type = Radio.RXdataBuffer[0] & TLM_PACKET;
    uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;
    uint8_t TLMheader = Radio.RXdataBuffer[1];

    if (packetAddr != DeviceAddr)
    {
        // Serial.println("TLM device address error"); // Uncomment for debugging
        return;
    }

    if ((inCRC != calculatedCRC)) // ELRS application-level CRC check
    {
        // Serial.println("TLM crc error"); // Uncomment for debugging
        return;
    }

    if (type != TLM_PACKET)
    {
        // Serial.println("TLM type error"); // Uncomment for debugging
        return;
    }

    if (connectionState != connected)
    {
        connectionState = connected;
        LPD_DownlinkLQ.init(100);
        Serial.println("Got downlink connection!");
    }

    LastTLMpacketRecvMillis = millis();
    LQCALC.add();

    // --- Telemetry Output to Serial Monitor ---
    if (TLMheader == CRSF_FRAMETYPE_LINK_STATISTICS) // Assuming ELRS still uses this header for link stats
    {
        // Get RSSI/SNR from the radio directly for the *last received packet*
        // Radio.GetPacketStatus(&Radio.LastPacketRSSI, &Radio.LastPacketSNR); // This should be done by the SX1280_DIO1_ISR if it's RX_DONE

        uint8_t uplink_RSSI_1 = Radio.RXdataBuffer[2]; // RSSI of the packet sent by TX and received by RX
        int8_t uplink_SNR = Radio.RXdataBuffer[4];
        uint8_t uplink_Link_quality = Radio.RXdataBuffer[5];

        int8_t downlink_SNR = Radio.LastPacketSNR; // SNR of the TLM packet received by TX from RX
        int16_t downlink_RSSI = Radio.LastPacketRSSI; // RSSI of the TLM packet received by TX from RX
        uint8_t downlink_Link_quality = LPD_DownlinkLQ.update(LQCALC.getLQ()) + 1;
        uint8_t rf_Mode = 4 - ExpressLRS_currAirRate_Modparams->index;

        uint16_t voltage_raw = (Radio.RXdataBuffer[3] << 8) + Radio.RXdataBuffer[6];
        float voltage_mv = (float)voltage_raw * 100.0f; // Assuming 100mV per unit for a common ELRS scaling

        if (millis() - LastTelemetryPrinted > SERIAL_TELEMETRY_UPDATE_INTERVAL)
        {
            Serial.println("--- Telemetry Update ---");
            Serial.print("Uplink RSSI (RX's perspective): ");
            Serial.print(uplink_RSSI_1);
            Serial.println(" dBm");
            Serial.print("Uplink SNR (RX's perspective): ");
            Serial.print(uplink_SNR);
            Serial.println(" dB");
            Serial.print("Uplink LQ (RX's perspective): ");
            Serial.print(uplink_Link_quality);
            Serial.println("%");

            Serial.print("Downlink RSSI (TX's perspective): ");
            Serial.print(downlink_RSSI);
            Serial.println(" dBm");
            Serial.print("Downlink SNR (TX's perspective): ");
            Serial.print(downlink_SNR);
            Serial.println(" dB");
            Serial.print("Downlink LQ (TX's perspective): ");
            Serial.print(downlink_Link_quality);
            Serial.println("%");

            Serial.print("RF Mode Index: ");
            Serial.println(rf_Mode);

            Serial.print("RX Battery Voltage: ");
            Serial.print(voltage_mv / 1000.0f, 2);
            Serial.println("V");
            Serial.println("------------------------");
            LastTelemetryPrinted = millis();
        }
    }
}


// --- Packet Generation ---
void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
    uint8_t PacketHeaderAddr;
    uint8_t SwitchEncMode = 0b00; // Simplified, assuming basic switch encoding
    uint8_t Index = (ExpressLRS_currAirRate_Modparams->index & 0b11);
    uint8_t TLMrate = (ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111);
    PacketHeaderAddr = (DeviceAddr << 2) + SYNC_PACKET;
    Radio.TXdataBuffer[0] = PacketHeaderAddr;
    Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
    Radio.TXdataBuffer[2] = NonceTX;
    Radio.TXdataBuffer[3] = (Index << 6) + (TLMrate << 3) + (SwitchEncMode << 1);
    Radio.TXdataBuffer[4] = UID[3]; // Assuming UID is defined and accessible
    Radio.TXdataBuffer[5] = UID[4];
    Radio.TXdataBuffer[6] = UID[5];
}

void ICACHE_RAM_ATTR GenerateRCDataPacket_11bit()
{
    uint8_t PacketHeaderAddr;
    PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
    Radio.TXdataBuffer[0] = PacketHeaderAddr;
    Radio.TXdataBuffer[1] = ((ChannelData[0]) >> 3); // Ch0 8 MSBs
    Radio.TXdataBuffer[2] = ((ChannelData[1]) >> 3); // Ch1 8 MSBs
    Radio.TXdataBuffer[3] = ((ChannelData[2]) >> 3); // Ch2 8 MSBs
    Radio.TXdataBuffer[4] = ((ChannelData[3]) >> 3); // Ch3 8 MSBs

    Radio.TXdataBuffer[5] = ((ChannelData[0] & 0b00000111) << 5) + // Ch0 3 LSBs
                            ((ChannelData[1] & 0b00000111) << 2) + // Ch1 3 LSBs
                            ((ChannelData[2] & 0b00000110) >> 1);  // Ch2 2 MSBs (of 3 LSBs) - 1 bit will be in next byte

    Radio.TXdataBuffer[6] = ((ChannelData[2] & 0b00000001) << 7) + // Ch2 1 LSB
                            ((ChannelData[3] & 0b00000111) << 4);  // Ch3 3 LSBs

    // Incorporate digital switches (Channels 4 and 5) into the remaining bits of byte 6
    Radio.TXdataBuffer[6] |= ((CRSF_to_BIT(ChannelData[4])) << 3); // Switch 1 (Ch4)
    Radio.TXdataBuffer[6] |= ((CRSF_to_BIT(ChannelData[5])) << 2); // Switch 2 (Ch5)
}

void ICACHE_RAM_ATTR GenerateMSPData()
{
    uint8_t PacketHeaderAddr;
    PacketHeaderAddr = (DeviceAddr << 2) + MSP_DATA_PACKET;
    Radio.TXdataBuffer[0] = PacketHeaderAddr;
    Radio.TXdataBuffer[1] = MSPPacket.function;
    Radio.TXdataBuffer[2] = MSPPacket.payloadSize;
    Radio.TXdataBuffer[3] = 0; // Placeholder for first payload byte
    Radio.TXdataBuffer[4] = 0; // Placeholder for second payload byte
    Radio.TXdataBuffer[5] = 0; // Placeholder for third payload byte
    Radio.TXdataBuffer[6] = 0; // Placeholder for fourth payload byte
    if (MSPPacket.payloadSize <= 4)
    {
        MSPPacket.payloadReadIterator = 0;
        for (int i = 0; i < MSPPacket.payloadSize; i++)
        {
            Radio.TXdataBuffer[3 + i] = MSPPacket.readByte();
        }
    }
    else
    {
        Serial.println("Unable to send MSP command. Packet too long.");
    }
}


// --- RF Link Rate Configuration ---
void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index)
{
    Serial.print("Setting RF link rate to index: ");
    Serial.println(index);
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    // Call the simplified SX1280 driver's config
    // Note: BW, SF, CR values are specific to SX1280 and 2.4GHz
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen);
    hwTimer.updateInterval(ModParams->interval);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    connectionState = connected; // Assume connected after setting link rate (will be checked by TLM)

#ifdef PLATFORM_ESP32
    //updateLEDs(connectionState, ExpressLRS_currAirRate_Modparams->TLMinterval);
#endif
}


// --- Frequency Hopping and Telemetry Management ---
void ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresult = (NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if (modresult == 0) // if it time to hop, do so.
    {
        Radio.SetFrequency(FHSSgetNextFreq());
    }
}

void ICACHE_RAM_ATTR HandleTLM()
{
    if (ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
    {
        uint8_t modresult = (NonceTX) % TLMratioEnumToValue((expresslrs_tlm_ratio_e)ExpressLRS_currAirRate_Modparams->TLMinterval);
        if (modresult != 0) // If it's not a TLM slot, do nothing
        {
            return;
        }
        Radio.RXnb(); // Put radio into RX mode, expect interrupt
        WaitRXresponse = true; // Set flag
    }
}


// --- Main RC Data Transmission Logic ---
void ICACHE_RAM_ATTR SendRCdataToRF()
{
    // Read current analog and digital inputs before sending
    ReadAnalogChannels();
    ReadDigitalSwitches();

    // --- This Part Handles the Telemetry Response ---
    // If it's a TLM slot (determined by NonceTX and TLMinterval)
    if ((uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
    {
        uint8_t modresult = (NonceTX) % TLMratioEnumToValue((expresslrs_tlm_ratio_e)ExpressLRS_currAirRate_Modparams->TLMinterval);
        if (modresult == 0)
        { // It's a TLM slot, so we expect a response or have waited
            if (WaitRXresponse == true)
            {
                WaitRXresponse = false; // We were waiting, but no response this cycle means lost TLM
                LQCALC.inc(); // Increment missed TLM counter for LQ calculation
                return; // Skip TX this cycle, we were waiting for RX
            }
            else
            {
                NonceTX = NonceTX + 1; // Proceed to next nonce if we weren't waiting
            }
        }
    }

    uint32_t SyncInterval;
    SyncInterval = (connectionState == connected) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
    bool skipSync = false; // No ARM channel concept without CRSF

    // Decide whether to send a Sync packet or an RC data/MSP packet
    if ((!skipSync) && ((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq()) && ((NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0)))
    {
        GenerateSyncPacketData();
        SyncPacketLastSent = millis();
        // Serial.println("Sending Sync Packet"); // Uncomment for debugging
    }
    else
    {
        if ((millis() > (MSP_PACKET_SEND_INTERVAL + MSPPacketLastSent)) && MSPPacketSendCount)
        {
            GenerateMSPData();
            MSPPacketLastSent = millis();
            MSPPacketSendCount--;
            // Serial.println("Sending MSP Packet"); // Uncomment for debugging
        }
        else
        {
            GenerateRCDataPacket_11bit(); // Send the RC data from analog/digital inputs
            // Serial.println("Sending RC Data Packet"); // Uncomment for debugging
        }
    }

    // Calculate the CRC (ExpressLRS application-level CRC) and put it into the buffer
    uint8_t crc = ota_crc.calc(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
    Radio.TXdataBuffer[7] = crc;
    Radio.TXnb(Radio.TXdataBuffer, 8); // Transmit the 8-byte packet
}


// --- Parameter Update (Simplified) ---
// This function needs to be triggered by *your* custom logic
// (e.g., a button press, a serial command you implement, etc.)
void HandleUpdateParameter()
{
    // No Lua updates to send to OpenTX. Telemetry is printed to Serial.
    if (millis() - LastTelemetryPrinted > SERIAL_TELEMETRY_UPDATE_INTERVAL) {
        // This is handled in ProcessTLMpacket now when a packet is received
        // You could add a periodic check here if no TLM is ever received.
    }

    if (UpdateParamReq == false)
    {
        return; // No update requested
    }

    // --- Example: Implement a simple button-based mode change ---
    // This is placeholder logic. You'd replace this with your actual input method.
    // E.g., a button press might increment ExpressLRS_nextAirRateIndex
    // For now, let's just show an example of setting fixed values for testing
    // or how you might hook up a hypothetical input.

    // If a certain condition (e.g., specific button combination) is met, change settings
    static uint32_t lastManualUpdate = 0;
    if (millis() - lastManualUpdate > 5000) // Allow manual update every 5 seconds for testing
    {
        // Example: Cycle through air rates
        // ExpressLRS_nextAirRateIndex = (ExpressLRS_nextAirRateIndex + 1) % (RATE_MAX - 1); // Cycle, exclude RATE_MAX itself
        // config.SetRate(ExpressLRS_nextAirRateIndex);
        // Serial.print("Manual Rate Change Request: "); Serial.println(ExpressLRS_nextAirRateIndex);

        // Example: Toggle Bind Mode
        // bindMode = !bindMode;
        // Serial.print("Bind Mode: "); Serial.println(bindMode ? "ON" : "OFF");

        // Example: Enter Web Update Mode (e.g., if a specific button is held at boot)
        // if (digitalRead(SOME_BOOT_BUTTON) == LOW) { // Hypothetical boot button
        //     webUpdateMode = true;
        // }

        // For now, we clear the flag after checking, assuming an external trigger sets it
        // and its logic is simple (e.g. fixed changes for testing).
        UpdateParamReq = false;
        lastManualUpdate = millis();

        if (config.IsModified()) // If config was changed by any logic above
        {
            hwTimer.stop();
            WaitEepromCommit = true;
        }
    } else {
        UpdateParamReq = false; // Clear flag if not enough time passed for another update
    }
}


// --- Radio ISRs (Callbacks from LoRaRadio) ---
void ICACHE_RAM_ATTR RXdoneISR()
{
    // A packet was received by the radio. Get its status and process.
    uint8_t rxPayloadLength = 0;
    uint8_t rxStartBufferPointer = 0;
    Radio.GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
    Radio.GetPacketStatus(&Radio.LastPacketRSSI, &Radio.LastPacketSNR);

    // Read the actual payload into RXdataBuffer
    Radio.ReadRegisters(0x0800 + rxStartBufferPointer, Radio.RXdataBuffer, rxPayloadLength);

    ProcessTLMpacket();
}

void ICACHE_RAM_ATTR TXdoneISR()
{
    NonceTX = NonceTX + 1; // Must be done before calling HandleFHSS/HandleTLM
    HandleFHSS(); // Handle frequency hopping for the next packet
    HandleTLM();  // Decide if the next slot should be for RX (telemetry)
}


// --- MSP Packet Processing (Simplified) ---
// This part remains as it handles MSP packets for things like VTX control.
// However, the *source* of these MSP packets is no longer UART CRSF.
// If you want to use MSP, you'd need to send bytes to msp.processReceivedByte(c)
// from your chosen input method (e.g., a custom serial command, or from WiFi).
void OnRFModePacket(mspPacket_t *packet)
{
    uint8_t rfMode = packet->readByte();
    // CHECK_PACKET_PARSING() macro from MSP.h should be handled by the MSP class internally.
    // If you're using this, ensure msp.currentParseError is correctly managed.

    Serial.print("MSP: Request RF Mode: ");
    Serial.println(rfMode);
    // You might want to update config and commit, or just directly apply
    if (rfMode <= RATE_MAX -1) { // Check valid range
        config.SetRate(enumRatetoIndex((expresslrs_RFrates_e)rfMode));
        UpdateParamReq = true; // Trigger update of radio settings
    }
}

void OnTxPowerPacket(mspPacket_t *packet)
{
    uint8_t txPower = packet->readByte();
    Serial.print("MSP: Request Power: ");
    Serial.println(txPower);
    if (txPower <= PWR_MAX -1) { // Check valid range
        config.SetPower((PowerLevels_e)txPower);
        UpdateParamReq = true; // Trigger update of radio settings
    }
}

void OnTLMRatePacket(mspPacket_t *packet)
{
    uint8_t tlmRate = packet->readByte();
    Serial.print("MSP: Request TLM interval: ");
    Serial.println(tlmRate);
    if (tlmRate <= TLM_RATIO_MAX -1) { // Check valid range
        config.SetTlm((expresslrs_tlm_ratio_e)tlmRate);
        UpdateParamReq = true; // Trigger update of radio settings
    }
}

void ProcessMSPPacket(mspPacket_t *packet)
{
    if (packet->function == MSP_ELRS_FUNC)
    {
        uint8_t opcode = packet->readByte();
        switch (opcode)
        {
        case MSP_ELRS_RF_MODE:
            OnRFModePacket(packet);
            break;
        case MSP_ELRS_TX_PWR:
            OnTxPowerPacket(packet);
            break;
        case MSP_ELRS_TLM_RATE:
            OnTLMRatePacket(packet);
            break;
        default:
            Serial.print("MSP: Unknown ELRS opcode: ");
            Serial.println(opcode, HEX);
            break;
        }
    }
    else if (packet->function == MSP_SET_VTX_CONFIG)
    {
        // This is a standard MSP message to configure a VTX
        MSPPacket = *packet; // Copy the received packet to a global buffer to be sent
        MSPPacketSendCount = 6; // Mark to send this VTX config packet 6 times
        Serial.println("MSP: Received SET_VTX_CONFIG. Will forward.");
    }
    else
    {
        Serial.print("MSP: Received unknown function: ");
        Serial.println(packet->function, HEX);
    }
}


// --- SETUP ---
void setup()
{
    Serial.begin(115200); // For debugging and telemetry output

#ifdef PLATFORM_ESP32
    // --- GPIO Pin Configuration ---
    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
    pinMode(GPIO_PIN_LED_RED, OUTPUT);
    digitalWrite(GPIO_PIN_LED_GREEN, HIGH); // Green LED on indicates boot
    pinMode(GPIO_PIN_BUZZER, OUTPUT);
    tone(GPIO_PIN_BUZZER, 400, 200); // Simple boot beep
    delay(200);
    noTone(GPIO_PIN_BUZZER);

    // --- Analog Input Setup ---
    analogReadResolution(12); // ESP32 ADC is 12-bit (0-4095)
    // No explicit pinMode for ADC pins needed, but ensures the pins are used as inputs
    pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
    pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
    pinMode(ANALOG_CHANNEL_3_PIN, INPUT);
    pinMode(ANALOG_CHANNEL_4_PIN, INPUT);

    // --- Digital Input Setup ---
    pinMode(DIGITAL_SWITCH_1_PIN, INPUT_PULLUP); // Use INPUT_PULLUP for switches
    pinMode(DIGITAL_SWITCH_2_PIN, INPUT_PULLUP);

    // --- Get ESP32 MAC Address for UID if MY_UID not defined (or override first 3 bytes with OUI) ---
    esp_efuse_mac_get_default(baseMac);
    // If MY_UID is not defined in platformio.ini, you'd generate a default here:
    // UID[0] = 0xAA; UID[1] = 0xBB; UID[2] = 0xCC; // Placeholder OUI
    // UID[3] = baseMac[3]; UID[4] = baseMac[4]; UID[5] = baseMac[5]; // Use last 3 bytes of MAC
#endif

    // --- Initialize Core ELRS Components ---
    FHSSrandomiseFHSSsequence(); // Needs UID from common.h for seeding
    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;
    hwTimer.callbackTock = &TimerCallbackISR;

    Serial.println("ExpressLRS TX Module Booted (ESP32-C6 Custom Analog/Digital Input)...");

    POWERMGNT.init(); // Initialize power management
    Radio.currFreq = GetInitialFreq(); // Set initial frequency based on FHSS

    // --- Radio Initialization ---
    Radio.Begin(); // Calls SPI.begin() and basic SX1280 setup
    // Check if radio initialization was successful (Add error handling to LoRaRadio.Begin())
    // while (!Radio.Begin()) { ... error handling ... }
    POWERMGNT.setDefaultPower(); // Set default power based on config or hardcoded

    // --- EEPROM and Configuration Loading ---
    eeprom.Begin(); // Initialize EEPROM
    config.SetStorageProvider(&eeprom); // Link config to EEPROM
    config.Load(); // Load saved configuration (rate, tlm, power)

    // Apply loaded configuration to the radio
    SetRFLinkRate(config.GetRate()); // Set initial RF link rate
    // ExpressLRS_nextAirRateIndex = ExpressLRS_currAirRate_Modparams->index; // This is redundant
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)config.GetTlm(); // Set TLM interval
    POWERMGNT.setPower((PowerLevels_e)config.GetPower()); // Set TX power

    // --- Timer Initialization ---
    hwTimer.init();
    hwTimer.resume();
    // hwTimer.stop(); // Remove this if you want the timer to run immediately after setup
    LQCALC.init(10); // Initialize Link Quality calculation
}


// --- LOOP ---
void loop()
{
#ifdef PLATFORM_ESP32
   /* if (webUpdateMode)
    {
       // HandleWebUpdate(); // This handles WiFi update loop
        return;
    }*/
#endif

    HandleUpdateParameter(); // Check for and apply parameter updates (e.g., from button input)

    // If there's an outstanding eeprom write, and we've waited long enough for any IRQs to fire...
    if (WaitEepromCommit && (micros() - PacketLastSentMicros) > ExpressLRS_currAirRate_Modparams->interval)
    {
        // Re-apply radio settings from config before committing, in case they changed
        SetRFLinkRate(config.GetRate());
        ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)config.GetTlm();
        POWERMGNT.setPower((PowerLevels_e)config.GetPower());

        WaitEepromCommit = false; // Clear the flag
        Serial.println("EEPROM COMMIT: Saving configuration...");
        config.Commit(); // Write changes to EEPROM
        hwTimer.resume(); // Resume timer after write
    }

    // --- Connection State Management (based on last received telemetry) ---
    if (millis() > (RX_CONNECTION_LOST_TIMEOUT + LastTLMpacketRecvMillis))
    {
        connectionState = disconnected;
#ifdef GPIO_PIN_LED_RED
        digitalWrite(GPIO_PIN_LED_RED, LOW); // Red LED OFF when disconnected (or blink)
#endif
    }
    else
    {
        connectionState = connected;
#ifdef GPIO_PIN_LED_RED
        digitalWrite(GPIO_PIN_LED_RED, HIGH); // Red LED ON when connected
#endif
    }

    // --- Handle MSP commands (if you implement a way to send them) ---
    // Example: If you send MSP commands over a custom serial port or WiFi:
    /*
    if (Serial.available()) {
        uint8_t c = Serial.read();
        if (msp.processReceivedByte(c)) {
            ProcessMSPPacket(msp.getReceivedPacket());
            msp.markPacketReceived();
        }
    }
    */
}