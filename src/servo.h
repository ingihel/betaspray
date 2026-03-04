#pragma once

// GPIO pin assignments (left rail, GPIO4 through 11)
// All are RTC/Touch/ADC capable, there should be no UART/JTAG/USB/PSRAM/strapping conflicts
// GPIO9-11 are FSPI-capable but free since FSPI peripheral is unused
// Alternate iomux options are listed for posterity's sake
#define SERVO_PIN_0 4  // ADC1_3, TOUCH4
#define SERVO_PIN_1 5  // ADC1_4, TOUCH5
#define SERVO_PIN_2 6  // ADC1_5, TOUCH6
#define SERVO_PIN_3 7  // ADC1_6, TOUCH7
#define SERVO_PIN_4 8  // ADC1_7, TOUCH8
#define SERVO_PIN_5 9  // ADC1_8, TOUCH9, FSPIHD
#define SERVO_PIN_6 10 // ADC1_9, TOUCH10, FSPICS0
#define SERVO_PIN_7 11 // ADC2_0, TOUCH11, FSPID

// How many servos are being used in the current configuration
#define NUM_SERVOS 1

// How long to hold position before releasing the PWM signal (ms)
#define SERVO_DURATION_MS 500

// Initialize all servo LEDC channels
void servo_init(void);

// 50 Hz at 5% DC

// Drive servo `id` (0..NUM_SERVOS-1) to `angle` (0-180 degrees)
// Holds for SERVO_DURATION_MS then releases
void servo_drive(int id, int angle);
