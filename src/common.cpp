#include "common.h"
#include "Definitions.h" // Include our central definitions
#include "LoRaRadio.h"   // Needed for LORA_BW_, LORA_SF, LORA_CR defines

// --- Global Variable Definitions ---
// The actual definition for CRCCaesarCipher as a const variable
const uint8_t CRCCaesarCipher = 0x07; // Defined here, declared extern in common.h

// MY_UID should come from platformio.ini - if not, use a default fallback
#ifndef MY_UID
#define MY_UID {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE}
#endif
uint8_t UID[6] = MY_UID; // Defined here, declared extern in common.h

// Device address, derived from UID.
uint8_t DeviceAddr = UID[5]; // Using last byte of UID as device address

// Common frequency correction for radio chips (often 0 or very small for SX1280)
long FreqCorrection = 0; // Adjust if your module needs frequency calibration

// --- SX1280 (2.4GHz) Modulation Settings ---
// These values are CRITICAL and must match the ExpressLRS 2.4GHz implementation.
// Consult ExpressLRS's actual 2.4GHz targets/modules/common.h for precise values.
// This is an EDUCATED GUESS for common settings.
expresslrs_mod_settings_s ExpressLRS_AirRates_SX1280[] = {
    // BW (kHz), SF, CR (4/x), PreambleLen, interval (us), index, enum_rate, TLMinterval_placeholder, FHSShopInterval
    {LORA_BW_1600, LORA_SF5, LORA_CR_4_5, 12, 2000, 0, RATE_500HZ, TLM_RATIO_NO_TLM, 1}, // Added 1 for FHSShopInterval
    {LORA_BW_800, LORA_SF7, LORA_CR_4_5, 10, 4000, 1, RATE_250HZ, TLM_RATIO_NO_TLM, 1}, // Added 1
    {LORA_BW_400, LORA_SF9, LORA_CR_4_5, 8, 10000, 2, RATE_100HZ, TLM_RATIO_NO_TLM, 1}, // Added 1
    {LORA_BW_200, LORA_SF10, LORA_CR_4_5, 6, 20000, 3, RATE_50HZ, TLM_RATIO_NO_TLM, 1}, // Added 1
};


// --- RF Performance Parameters (Simplified) ---
// These define things like how often sync packets are sent.
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[] = {
    // SyncPktIntervalConnected (us), SyncPktIntervalDisconnected (us)
    {20000, 50000}, // For 500Hz/250Hz (send sync every 20ms/50ms)
    {20000, 50000},
    {20000, 50000},
    {20000, 50000},
};


// Global pointers to the currently active modulation and performance settings
expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;


// --- Function Implementations ---
ICACHE_RAM_ATTR expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index)
{
    if (index < 0) {
        return &ExpressLRS_AirRates_SX1280[0];
    } else if (index >= NUM_AIR_RATES_SX1280) { // NUM_AIR_RATES_SX1280 is from common.h
        return &ExpressLRS_AirRates_SX1280[NUM_AIR_RATES_SX1280 - 1];
    }
    return &ExpressLRS_AirRates_SX1280[index];
}

ICACHE_RAM_ATTR expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index)
{
    // Simplistic: Map all rates to the same performance params for now,
    // or you could have a different entry for each air rate if needed.
    if (index < 0) {
        return &ExpressLRS_AirRateRFperf[0];
    } else if (index >= NUM_RF_PERF_SETTINGS) { // NUM_RF_PERF_SETTINGS is from common.h
        return &ExpressLRS_AirRateRFperf[NUM_RF_PERF_SETTINGS - 1];
    }
    return &ExpressLRS_AirRateRFperf[index];
}

uint8_t enumRatetoIndex(expresslrs_RFrates_e rate)
{
    for (uint8_t i = 0; i < NUM_AIR_RATES_SX1280; i++) // NUM_AIR_RATES_SX1280 is from common.h
    {
        if (ExpressLRS_AirRates_SX1280[i].enum_rate == rate)
        {
            return i;
        }
    }
    return 0; // Default to first rate if not found
}

uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e rate)
{
    // Maps TLM_RATIO_ENUM to actual "skip" value for NonceTX modulo
    switch (rate)
    {
        case TLM_RATIO_NO_TLM: return 0;
        case TLM_RATIO_1_128: return 128;
        case TLM_RATIO_1_64: return 64;
        case TLM_RATIO_1_32: return 32;
        case TLM_RATIO_1_16: return 16;
        case TLM_RATIO_1_8: return 8;
        case TLM_RATIO_1_4: return 4;
        case TLM_RATIO_1_2: return 2;
        case TLM_RATIO_MAX: return 0; // Should not be used as an actual ratio
        default: return 0; // Should not happen
    }
}