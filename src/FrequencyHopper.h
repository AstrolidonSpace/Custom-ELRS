#pragma once

#include <Arduino.h>
#include "Definitions.h" // For regulatory domain and constants

// Frequency hopping sequence and related variables
extern volatile uint8_t FHSSptr;        // Current index in the hopping sequence
extern volatile uint8_t FHSSsequence[256]; // The actual hopping sequence
extern uint32_t FHSSfreqs_EU_868[]; // Example. We'll use ISM_2400.

#ifdef REGULATORY_DOMAIN_ISM_2400
extern uint32_t FHSSfreqs_ISM_2400[]; // Frequencies for 2.4GHz
#endif

// Function Prototypes
void FHSSrandomiseFHSSsequence();
uint32_t GetInitialFreq();
uint32_t FHSSgetCurrFreq();
uint32_t FHSSgetNextFreq();
uint8_t FHSSgetCurrIndex();