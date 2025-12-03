#pragma once

#include <Arduino.h>
#include "Definitions.h" // For regulatory domain and other constants

// Global variables for FHSS (defined in FHSS.cpp)
extern volatile uint8_t FHSSptr;         // Current index in the hopping sequence
extern volatile uint8_t FHSSsequence[256]; // The actual hopping sequence

// Declaration of frequency tables based on regulatory domain
#ifdef REGULATORY_DOMAIN_ISM_2400
extern uint32_t FHSSfreqs_ISM_2400[];
#define NR_FHSS_ENTRIES (sizeof(FHSSfreqs_ISM_2400) / sizeof(uint32_t))
#else
// This #error should ideally be caught by platformio.ini for our custom build,
// but leaving it as a safeguard if REGULATORY_DOMAIN_ISM_2400 isn't defined.
#error "No regulatory domain defined for FHSS frequencies, please define one in platformio.ini"
#endif

// Function Prototypes for FHSS (implementations in FHSS.cpp)
void FHSSrandomiseFHSSsequence();
uint32_t GetInitialFreq();
uint32_t FHSSgetCurrFreq();
uint32_t FHSSgetNextFreq();
uint8_t FHSSgetCurrIndex();