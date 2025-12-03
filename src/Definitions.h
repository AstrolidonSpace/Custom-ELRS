#pragma once

#include "CRCExtra.h"
#include <Arduino.h>

// --- Global Defines (from platformio.ini) ---
// These will be overridden by -D flags in platformio.ini, but good for IDE parsing
// #define REGULATORY_DOMAIN_ISM_2400
// #define USE_SX1280_DRIVER
// #define PLATFORM_ESP32
// #define MY_UID "{0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE}" // OVERRIDE IN platformio.ini
// #define DEVICE_NAME "ELRS_Custom_TX_C6" // OVERRIDE IN platformio.ini
// #define DEFAULT_TX_AIR_RATE_HZ 250 // OVERRIDE IN platformio.ini
// #define DEFAULT_TLM_RATIO_ENUM TLM_RATIO_1_16 // OVERRIDE IN platformio.ini
// #define DEFAULT_TX_POWER PWR_100mW // OVERRIDE IN platformio.ini

#define MSP_PACKET_SEND_INTERVAL 200
#define RX_CONNECTION_LOST_TIMEOUT 3000

#ifndef DefaultPowerEnum
#define DefaultPowerEnum PWR_50mW // Example default
#endif

#ifndef RATE_DEFAULT
#define RATE_DEFAULT 1 // Fallback value, e.g., for 250Hz
#endif

#ifndef MaxPower
#define MaxPower PWR_250mW // Example max power for your module.
                           // Adjust this based on your actual hardware's capabilities.
                           // Most simple SX1280 modules with integrated PA go up to +13dBm (~20mW),
                           // but external PAs can boost this to 100mW, 250mW, etc.
#endif

// --- Your Custom Pin Definitions for ESP32-C6 (VERIFY THESE FOR YOUR BOARD!) ---
#ifndef GPIO_PIN_LED_GREEN
#define GPIO_PIN_LED_GREEN 8
#endif
#ifndef GPIO_PIN_LED_RED
#define GPIO_PIN_LED_RED 9
#endif
#ifndef GPIO_PIN_BUZZER
#define GPIO_PIN_BUZZER 10
#endif

// Analog Input Pins (Must be ADC-capable on ESP32-C6)
#ifndef ANALOG_CHANNEL_1_PIN
#define ANALOG_CHANNEL_1_PIN 1
#endif
#ifndef ANALOG_CHANNEL_2_PIN
#define ANALOG_CHANNEL_2_PIN 2
#endif
#ifndef ANALOG_CHANNEL_3_PIN
#define ANALOG_CHANNEL_3_PIN 3
#endif
#ifndef ANALOG_CHANNEL_4_PIN
#define ANALOG_CHANNEL_4_PIN 4
#endif

// Digital Switch Input Pins
#ifndef DIGITAL_SWITCH_1_PIN
#define DIGITAL_SWITCH_1_PIN 5
#endif
#ifndef DIGITAL_SWITCH_2_PIN
#define DIGITAL_SWITCH_2_PIN 6
#endif

// SX1280 SPI Pins (VERIFY THESE FOR YOUR ESP32-C6 BOARD + SX1280 WIRING!)
#ifndef SX1280_SCK
#define SX1280_SCK 12
#endif
#ifndef SX1280_MISO
#define SX1280_MISO 13
#endif
#ifndef SX1280_MOSI
#define SX1280_MOSI 11
#endif
#ifndef SX1280_NSS // Chip Select
#define SX1280_NSS 14
#endif
#ifndef SX1280_BUSY
#define SX1280_BUSY 15
#endif
#ifndef SX1280_DIO1
#define SX1280_DIO1 16
#endif
#ifndef SX1280_RST
#define SX1280_RST 17
#endif


// --- ExpressLRS Common Defines (From original common.h) ---
#define DEVICE_VCC_3V3 3300 // default VCC for targets with 3.3V power

// Universal Unique Identifier (UID)
// This is typically defined in your build flags for custom builds, but needs a default if not.
// const uint8_t UID[6] = MY_UID; // Defined in common.h, but MY_UID comes from build_flags

#define RC_DATA_PACKET 0x00
#define SYNC_PACKET 0x01
#define TLM_PACKET 0x02
#define MSP_DATA_PACKET 0x03

//#define CRCCaesarCipher 0x07

// Connection State
enum connectionState_e
{
    disconnected = 0,
    connected = 1,
};
extern connectionState_e connectionState; // Declared in main.cpp, used globally

// Default device address (can be changed based on UID)
extern uint8_t DeviceAddr; // Declared in common.cpp

// --- Modulation Settings (Simplified for our use) ---
// These are rough equivalents and might need fine-tuning based on SX1280 datasheet/ELRS source
struct expresslrs_mod_settings_s
{
    uint32_t bw;
    uint8_t sf;
    uint8_t cr;
    uint16_t PreambleLen;
    uint16_t interval; // Time in microseconds between packets
    uint8_t index;     // Index in an array of settings
    uint8_t enum_rate; // The rate enum value
    uint8_t TLMinterval; // Telemetry interval enum
    uint8_t FHSShopInterval;
};

// Rate enums (matching ELRS original)
typedef enum
{
    RATE_250HZ = 0, // 250Hz, 4ms interval
    RATE_500HZ = 1, // 500Hz, 2ms interval
    RATE_100HZ = 2, // 100Hz, 10ms interval
    RATE_50HZ = 3,  // 50Hz, 20ms interval
    RATE_MAX // Keep this for array sizing
} expresslrs_RFrates_e;


// Telemetry ratio enums
typedef enum
{
    TLM_RATIO_NO_TLM = 0, // No telemetry
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7,
    TLM_RATIO_MAX // Max value for array sizing
} expresslrs_tlm_ratio_e;

// Power Levels
typedef enum
{
    PWR_10mW = 0,
    PWR_25mW = 1,
    PWR_50mW = 2,
    PWR_100mW = 3,
    PWR_250mW = 4,
    PWR_500mW = 5,
    PWR_1000mW = 6,
    PWR_2000mW = 7,
    PWR_MAX // Max value for array sizing
} PowerLevels_e;

// RF Performance Parameters (simplified)
struct expresslrs_rf_pref_params_s
{
    uint32_t SyncPktIntervalConnected; // Interval for sync packets when connected (us)
    uint32_t SyncPktIntervalDisconnected; // Interval for sync packets when disconnected (us)
};

// Global pointers to current modulation and performance settings
extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
extern expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;
extern GENERIC_CRC8 ota_crc;
extern volatile uint8_t NonceTX;

// Function prototypes (definitions in common.cpp usually)
expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index);
expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index);
uint8_t enumRatetoIndex(expresslrs_RFrates_e rate);
uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e rate);


// Function to convert CRSF channel value to 10-bit or 11-bit representation
// CRSF uses 11-bit values (172 to 1811), centered at 992 (0-2047 range roughly)
// For our analog inputs, we'll map 0-4095 (ADC) to 0-2047 (ELRS 11-bit channel) directly
// These are kept for reference but might not be directly used if we generate 11-bit data from start.
static inline uint16_t CRSF_to_UINT11(uint16_t val)
{
    // CRSF range is typically 172 to 1811 (1639 points).
    // ELRS uses 0-2047 (2048 points) for 11-bit channels.
    // So, we'll map our analog input (0-4095 from ADC) to 0-2047.
    return val; // Assume val is already in 0-2047 range from analogRead mapping
}

static inline uint8_t CRSF_to_BIT(uint16_t val)
{
    return (val > 1023) ? 1 : 0; // If value is above half (center), consider it 'on'
}


// --- SPI Pins (for SX1280) from platformio.ini ---
// They are passed as #defines, but sometimes easier to get directly if needed
// const int SX1280_SCK_PIN = SX1280_SCK; // This approach is not ideal, use defines directly