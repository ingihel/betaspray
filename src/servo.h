#pragma once

// GPIO pin assignments
// GPIO1: ADC1_CH0, free from camera/JTAG/USB/PSRAM/strapping conflicts
// Alternate iomux options are listed for posterity's sake
#define SERVO_PIN_0 1  // ADC1_0
#define SERVO_PIN_1 2  // ADC1_1, TOUCH2
#define SERVO_PIN_2 6  // ADC1_5, TOUCH6
#define SERVO_PIN_3 7  // ADC1_6, TOUCH7
#define SERVO_PIN_4 8  // ADC1_7, TOUCH8
#define SERVO_PIN_5 9  // ADC1_8, TOUCH9, FSPIHD
#define SERVO_PIN_6 10 // ADC1_9, TOUCH10, FSPICS0
#define SERVO_PIN_7 11 // ADC2_0, TOUCH11, FSPID

// How many servos are being used in the current configuration
#define NUM_SERVOS 2

// Time (ms) for a full 180° hardware fade.  Partial moves scale proportionally.
// 2000 ms ≈ 4× the SG90's natural ~0.5 s traversal time.
#define SERVO_FADE_TIME_MS 2000

// How long to hold the final position before releasing the PWM signal (ms)
#define SERVO_DURATION_MS 500

// Initialize all servo LEDC channels
void servo_init(void);

// Drive servo `id` (0..NUM_SERVOS-1) to `angle` (0-180 degrees).
// Uses LEDC hardware fade for smooth, jitter-free duty-cycle ramping.
// Holds for SERVO_DURATION_MS then releases.
void servo_drive(int id, int angle);

// Testbench for X-axis servo (full 180 degree range).
//   Phase 1 — 15-degree increments: 0, 15, 30 ... 180
//   Phase 2 — coarse jumps: 30 -> 45 -> 90 -> 180
void servo_testbench_x(int id);

// Testbench for Y-axis servo (active range 90-180 degrees, flat at 90 degrees, straight up at 180 degrees).
//   Phase 1 — 15-degree increments: 90, 105, 120 ... 180
//   Phase 2 — coarse jumps: 120 -> 135 -> 180
void servo_testbench_y(int id);
