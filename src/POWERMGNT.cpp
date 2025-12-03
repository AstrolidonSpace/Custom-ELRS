#include "POWERMGNT.h"
#include "Definitions.h" // For PowerLevels_e, DefaultPowerEnum, MaxPower, GPIO_PIN_FAN_EN
#include "LoRaRadio.h"   // For Radio.SetTxPower()

// Initialize static member variable
// DefaultPowerEnum should be defined via platformio.ini or in Definitions.h
#ifndef DefaultPowerEnum
#define DefaultPowerEnum PWR_50mW // Default if not defined via build flags
#endif
#ifndef MaxPower
#define MaxPower PWR_250mW // Default max power for internal SX1280, adjust based on module
#endif

PowerLevels_e POWERMGNT::CurrentPower = (PowerLevels_e)DefaultPowerEnum;

POWERMGNT::POWERMGNT()
{
    // Constructor, could do initial setup here too
}

PowerLevels_e POWERMGNT::incPower()
{
    if (CurrentPower < (PowerLevels_e)MaxPower)
    {
        setPower((PowerLevels_e)((uint8_t)CurrentPower + 1));
    }
    return CurrentPower;
}

PowerLevels_e POWERMGNT::decPower()
{
    if (CurrentPower > 0) // PWR_10mW is usually 0
    {
        setPower((PowerLevels_e)((uint8_t)CurrentPower - 1));
    }
    return CurrentPower;
}

PowerLevels_e POWERMGNT::currPower()
{
    return CurrentPower;
}

void POWERMGNT::init()
{
    // GPIO_PIN_FAN_EN might not exist on all ESP32-C6 modules.
    // Only compile this if the pin is actually defined.
#ifdef GPIO_PIN_FAN_EN
    pinMode(GPIO_PIN_FAN_EN, OUTPUT);
    digitalWrite(GPIO_PIN_FAN_EN, LOW); // Start fan off
#endif
    Serial.println("POWERMGNT: Initialized.");
}

void POWERMGNT::setDefaultPower()
{
    setPower((PowerLevels_e)DefaultPowerEnum);
    Serial.print("POWERMGNT: Default power set to ");
    Serial.println((uint8_t)CurrentPower);
}

PowerLevels_e POWERMGNT::setPower(PowerLevels_e Power)
{
    if (Power > (PowerLevels_e)MaxPower)
    {
        Power = (PowerLevels_e)MaxPower;
    }

    // --- SX1280 Specific Power Mapping ---
    // The SetTxPower function in LoRaRadio.cpp takes an index (0x03 to 0x1F)
    // or a dBm value depending on how it was implemented.
    // Our LoRaRadio::SetTxPower currently takes an 'int8_t power_idx'
    // which maps to the internal SX1280 power values (-18dBm to +13dBm).
    // So we need to map our PowerLevels_e to these dBm values.

    int8_t sx1280_power_dbm; // In dBm, as used by SX1280 SetTxPower parameter

    switch (Power)
    {
    case PWR_10mW:
        sx1280_power_dbm = -5; // Example: maps to ~10mW for a common PA setup
        break;
    case PWR_25mW:
        sx1280_power_dbm = 0; // Example: maps to ~25mW
        break;
    case PWR_50mW:
        sx1280_power_dbm = 3; // Example: maps to ~50mW
        break;
    case PWR_100mW:
        sx1280_power_dbm = 6; // Example: maps to ~100mW
        break;
    case PWR_250mW:
        sx1280_power_dbm = 12; // Example: maps to ~250mW (max for standard SX1280 + typical PA)
        break;
    // Add more cases if PowerLevels_e defines higher powers and your hardware supports them.
    // For SX1280 without external amplifier, +13dBm (value 0x1F) is max.
    // A separate PA requires different values and possibly GPIO control.
    case PWR_500mW: // If your hardware has a PA that can do 500mW, set the appropriate SX1280 drive level
    case PWR_1000mW:
    case PWR_2000mW:
    default:
        // Default to a safe power, e.g., 50mW, if unsupported or higher than MaxPower
        sx1280_power_dbm = 3; // Corresponds to PWR_50mW
        Power = PWR_50mW;
        break;
    }

    Radio.SetTxPower(sx1280_power_dbm); // Call our custom LoRaRadio driver
    CurrentPower = Power; // Update the internal state

    // Fan control (if GPIO_PIN_FAN_EN is defined and hooked up)
#ifdef GPIO_PIN_FAN_EN
    // Activate fan if power is 250mW or higher
    (Power >= PWR_250mW) ? digitalWrite(GPIO_PIN_FAN_EN, HIGH) : digitalWrite(GPIO_PIN_FAN_EN, LOW);
#endif

    Serial.print("POWERMGNT: Set TX power to ");
    Serial.print((uint8_t)CurrentPower);
    Serial.print(" (LoRaRadio value: ");
    Serial.print(sx1280_power_dbm);
    Serial.println(" dBm)");
    return CurrentPower;
}