#include "FHSS.h"
#include "common.h" // For UID and FreqCorrection
#include "utils.h"  // For random() and esp_random() (if using Arduino random)

volatile uint8_t FHSSptr = 0;
volatile uint8_t FHSSsequence[256];

// --- Frequency Tables for SX1280 (2.4GHz ISM Band) ---
// These frequencies are crucial for correct operation.
// The values used here are placeholders/examples.
// For true ExpressLRS compatibility, you should get the precise
// frequency list from the official ExpressLRS 2.4GHz source code.
// They are typically 200kHz steps within the 2.4GHz ISM band.
#ifdef REGULATORY_DOMAIN_ISM_2400
uint32_t FHSSfreqs_ISM_2400[] = {
    2405000000UL, 2405200000UL, 2405400000UL, 2405600000UL, 2405800000UL,
    2406000000UL, 2406200000UL, 2406400000UL, 2406600000UL, 2406800000UL,
    2407000000UL, 2407200000UL, 2407400000UL, 2407600000UL, 2407800000UL,
    2408000000UL, 2408200000UL, 2408400000UL, 2408600000UL, 2408800000UL,
    2409000000UL, 2409200000UL, 2409400000UL, 2409600000UL, 2409800000UL,
    2410000000UL, 2410200000UL, 2410400000UL, 2410600000UL, 2410800000UL,
    2411000000UL, 2411200000UL, 2411400000UL, 2411600000UL, 2411800000UL,
    2412000000UL, 2412200000UL, 2412400000UL, 2412600000UL, 2412800000UL,
    2413000000UL, 2413200000UL, 2413400000UL, 2413600000UL, 2413800000UL,
    2414000000UL, 2414200000UL, 2414400000UL, 2414600000UL, 2414800000UL,
    2415000000UL, 2415200000UL, 2415400000UL, 2415600000UL, 2415800000UL,
    2416000000UL, 2416200000UL, 2416400000UL, 2416600000UL, 2416800000UL,
    2417000000UL, 2417200000UL, 2417400000UL, 2417600000UL, 2417800000UL,
    2418000000UL, 2418200000UL, 2418400000UL, 2418600000UL, 2418800000UL,
    2419000000UL, 2419200000UL, 2419400000UL, 2419600000UL, 2419800000UL,
    2420000000UL, 2420200000UL, 2420400000UL, 2420600000UL, 2420800000UL,
    2421000000UL, 2421200000UL, 2421400000UL, 2421600000UL, 2421800000UL,
    2422000000UL, 2422200000UL, 2422400000UL, 2422600000UL, 2422800000UL,
    2423000000UL, 2423200000UL, 2423400000UL, 2423600000UL, 2423800000UL,
    2424000000UL, 2424200000UL, 2424400000UL, 2424600000UL, 2424800000UL,
    2425000000UL
    // Add more frequencies up to the band limit, e.g., 2470MHz or 2480MHz,
    // ensuring the array size and NUM_FREQ_ISM_2400 are consistent.
    // The official ELRS has about 100 200kHz channels in 2.4GHz.
};
// NR_FHSS_ENTRIES is defined in FHSS.h using this array
#endif

// --- Helper for seeding pseudo-random number generator on ESP32 ---
// Uses the ESP-IDF's hardware RNG for better randomness
uint32_t get_random_seed()
{
    return esp_random();
}

void FHSSrandomiseFHSSsequence()
{
    // Initialize random seed using UID and hardware random bits
    uint32_t seed = get_random_seed();
    for (int i = 0; i < 6; i++) {
        seed ^= (uint32_t)UID[i] << (i * 4 % 32); // Incorporate UID bytes
    }
    randomSeed(seed);

    uint8_t num_channels;

#ifdef REGULATORY_DOMAIN_ISM_2400
    num_channels = NR_FHSS_ENTRIES; // Use the define from FHSS.h
#else
    // This should ideally be caught by platformio.ini or FHSS.h #error
    // If we reach here, it means no domain was defined, which is an issue.
    Serial.println("ERROR: No regulatory domain defined for FHSSrandomiseFHSSsequence!");
    return;
#endif

    // Fill FHSSsequence with initial ordered values (0 to num_channels-1)
    for (int i = 0; i < num_channels; i++) {
        FHSSsequence[i] = i;
    }

    // Fisher-Yates shuffle algorithm
    for (int i = num_channels - 1; i > 0; i--) {
        uint8_t j = random(0, i + 1); // Get a random index from 0 to i (inclusive)
        uint8_t temp = FHSSsequence[i];
        FHSSsequence[i] = FHSSsequence[j];
        FHSSsequence[j] = temp;
    }

    FHSSptr = 0; // Reset pointer to the beginning of the new randomized sequence
    Serial.print("FHSS sequence randomized with ");
    Serial.print(num_channels);
    Serial.println(" channels.");
}

uint32_t GetInitialFreq()
{
#ifdef REGULATORY_DOMAIN_ISM_2400
    // The very first frequency in the table before any hopping
    return FHSSfreqs_ISM_2400[0] - FreqCorrection; // FreqCorrection from common.h
#else
    // Fallback if no regulatory domain is defined
    return 2400000000UL; // A dummy frequency
#endif
}

uint32_t FHSSgetCurrFreq()
{
#ifdef REGULATORY_DOMAIN_ISM_2400
    if (FHSSptr >= NR_FHSS_ENTRIES) FHSSptr = 0; // Wrap around if out of bounds
    return FHSSfreqs_ISM_2400[FHSSsequence[FHSSptr]] - FreqCorrection;
#else
    return 0; // Error or dummy value
#endif
}

uint32_t FHSSgetNextFreq()
{
    // Increment pointer and wrap around
    FHSSptr = (FHSSptr + 1);
    if (FHSSptr >= NR_FHSS_ENTRIES) {
        FHSSptr = 0;
    }
    // Alternatively, simpler: FHSSptr = (FHSSptr + 1) % NR_FHSS_ENTRIES;

    return FHSSgetCurrFreq();
}

uint8_t FHSSgetCurrIndex()
{
    return FHSSptr;
}

// Ensure resetIsAvailable is removed if it was a source of errors.
// It seems to be part of an older or different FHSS logic.
/*
void resetIsAvailable(uint8_t* isAvailable) {
    for (unsigned int i = 0; i < NR_FHSS_ENTRIES; i++) {
        isAvailable[i] = 1; // Mark all channels as available
    }
}
*/