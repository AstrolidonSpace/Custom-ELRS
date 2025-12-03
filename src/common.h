#pragma once

#include <Arduino.h>
#include "elrs_eeprom.h"
#include "Definitions.h" // Include our central definitions
#include "common.h"

// LoRa Spreading Factors (SF)
#define LORA_SF5  0x05
#define LORA_SF6  0x06
#define LORA_SF7  0x07
#define LORA_SF8  0x08
#define LORA_SF9  0x09
#define LORA_SF10 0x0A
#define LORA_SF11 0x0B
#define LORA_SF12 0x0C

// LoRa Bandwidths (Example values, refer to SX1280 datasheet)
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

// --- Packet Types (could be in Definitions.h, but often grouped with common ELRS protocol) ---
// Defined as #defines in Definitions.h for simplicity, so no need for enum here.
// #define RC_DATA_PACKET 0x00
// #define SYNC_PACKET 0x01
// #define TLM_PACKET 0x02
// #define MSP_DATA_PACKET 0x03

// --- CRCCaesarCipher (Declared here, defined in common.cpp) ---
// Defined as a #define in Definitions.h. We need an extern if common.cpp defines it as a variable.
// If it's a #define in Definitions.h, common.h should *not* extern it as a variable.
// If it was originally an extern, then your previous error was because Definitions.h had it as a #define
// and common.h as an extern variable. Let's make it a const variable in common.cpp for clarity,
// so common.h needs this extern.
extern const uint8_t CRCCaesarCipher; // Declared as extern, definition will be in common.cpp
// --- Connection State (Declared here, defined in main.cpp, enum in Definitions.h) ---
extern connectionState_e connectionState; // Enum is in Definitions.h

// --- UID (Declared here, defined in common.cpp) ---
// MY_UID comes from platformio.ini, but the actual array instance is in common.cpp
extern uint8_t UID[6];

// --- Device Address (Declared here, defined in common.cpp) ---
extern uint8_t DeviceAddr;

// --- Frequency Correction (Declared here, defined in common.cpp) ---
extern long FreqCorrection;

// --- ELRS Global Parameters (Declared here, defined in common.cpp) ---
// These are the *global pointers* to the currently active modulation and performance settings.
// The structs themselves (expresslrs_mod_settings_s, expresslrs_rf_pref_params_s)
// and their arrays are defined in common.cpp, and their types are in Definitions.h.
extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
extern expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

extern expresslrs_mod_settings_s ExpressLRS_AirRates_SX1280[];
#define NUM_AIR_RATES_SX1280 (sizeof(ExpressLRS_AirRates_SX1280) / sizeof(expresslrs_mod_settings_s))

extern expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[];
#define NUM_RF_PERF_SETTINGS (sizeof(ExpressLRS_AirRateRFperf) / sizeof(expresslrs_rf_pref_params_s))


// --- Function Prototypes (Definitions in common.cpp) ---
// These functions provide access to the ELRS rate configurations and conversions.
expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index);
expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index);
uint8_t enumRatetoIndex(expresslrs_RFrates_e rate);
uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e rate);

// --- Packet CRC Type (For telemetry link statistics) ---
// CRSF_FRAMETYPE_LINK_STATISTICS is often used as a TLM header in ELRS.
// This is typically a define. If it conflicts with an enum somewhere, we'll make it a const.
// For now, let's assume it's a #define. If you got a conflict for this specific value before,
// let me know.
#ifndef CRSF_FRAMETYPE_LINK_STATISTICS
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#endif

// A simple way to get 1-bit value from a channel (for switches)
// Already in Definitions.h as CRSF_to_BIT. No need to redefine.
// static inline uint8_t CRSF_to_BIT(uint16_t val);

// Define this if it's used elsewhere (e.g. debug.h or other utility)
// #define DEBUG_SUPPRESS