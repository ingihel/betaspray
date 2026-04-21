#pragma once

// GPIO pin assignments
#define SERVO_PIN_0  4  // SERVO0
#define SERVO_PIN_1  6  // SERVO1
#define SERVO_PIN_2 18  // SERVO2
#define SERVO_PIN_3  8  // SERVO3
#define SERVO_PIN_4  9  // SERVO4
#define SERVO_PIN_5  5  // SERVO5
#define SERVO_PIN_6  7  // SERVO6
#define SERVO_PIN_7 10  // SERVO7

// How many servos are being used in the current configuration
#define NUM_SERVOS 6

// Time (ms) for a full 180° hardware fade.  Partial moves scale proportionally.
// 2000 ms ≈ 4× the SG90's natural ~0.5 s traversal time.
#define SERVO_FADE_TIME_MS 2000

// Brief settle time (ms) after a fade completes before releasing the PWM signal.
// Keeps the signal active just long enough for the motor to stop oscillating.
// Servo holds position mechanically once PWM is off — no sustained current draw.
#define SERVO_DURATION_MS 50

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

// Testbench for Y-axis servo (active range 90-180 degrees, flat at 90 degrees, straight up at 180
// degrees).
//   Phase 1 — 15-degree increments: 90, 105, 120 ... 180
//   Phase 2 — coarse jumps: 120 -> 135 -> 180
void servo_testbench_y(int id);
