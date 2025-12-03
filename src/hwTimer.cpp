#include "hwTimer.h"
#include <Arduino.h>
#include "esp_timer.h"      // For ESP-IDF timer functions, esp_timer_create_args_t (might not be needed after GPTimer switch, but keep for now)
#include "driver/gptimer.h" // Modern GPTimer driver header

volatile uint32_t hwTimer::HWtimerInterval = 0;

// Global timer handle for GPTimer
gptimer_handle_t gptimer = NULL;
volatile uint32_t HWtimerInterval = 0; // Interval in microseconds
void (*callback)(void); // Pointer to the ISR callback function

// GPTimer callback function
static bool IRAM_ATTR gptimer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // In an ISR, avoid Serial.print or complex operations.
    // Just call the registered callback.
    if (callback) {
        callback();
    }
    return true; // Return true if a higher priority task needs to be woken up (FreeRTOS thing)
}

void hwTimer::init() {
    // Configure GPTimer
    gptimer_config_t timer_config;
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT; // Use default clock source
    timer_config.direction = GPTIMER_COUNT_UP;      // Count up
    timer_config.resolution_hz = 1000000;           // 1 MHz, 1 tick per microsecond
    // Removed: .intr_priority = 0, (not available in some GPTimer_config_t)
    // Removed: .group_id = 0, (usually auto assigned or not needed for default setup)

    // Corrected: use gptimer_new_timer as suggested by compiler
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // Create new timer unit

    // Configure timer alarm
    gptimer_alarm_config_t alarm_config;
    alarm_config.alarm_count = HWtimerInterval; // Set alarm value
    // Corrected: avoid designated initializer for nested flags struct
    // Instead, directly assign the flags or ensure a C-style struct init.
    // For simple boolean flags, direct assignment often works if flags is a struct.
    alarm_config.flags.auto_reload_on_alarm = true; // Auto-reload on reaching alarm

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config)); // Set alarm action

    // Register the interrupt callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_on_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    Serial.println("hwTimer: GPTimer Initialized and Started.");
}

void hwTimer::resume() {
    if (gptimer != NULL) {
        // Re-set alarm value just in case interval changed while stopped
        gptimer_alarm_config_t alarm_config;
        alarm_config.alarm_count = HWtimerInterval;
        alarm_config.flags.auto_reload_on_alarm = true; // Corrected initialization
        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
        ESP_ERROR_CHECK(gptimer_start(gptimer));
    }
    Serial.println("hwTimer: GPTimer Resumed.");
}

void hwTimer::stop() {
    if (gptimer != NULL) {
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
    }
    Serial.println("hwTimer: GPTimer Stopped.");
}

void hwTimer::updateInterval(uint32_t interval_us) {
    HWtimerInterval = interval_us;
    if (gptimer != NULL) {
        // Reconfigure alarm with new interval
        gptimer_alarm_config_t alarm_config;
        alarm_config.alarm_count = HWtimerInterval;
        alarm_config.flags.auto_reload_on_alarm = true; // Corrected initialization
        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
        // No need to restart if it's already running and auto-reload is true
    }
    Serial.print("hwTimer: Interval updated to "); Serial.print(interval_us); Serial.println(" us.");
}

// Initialize static member
void (*hwTimer::callbackTock)() = nullptr;