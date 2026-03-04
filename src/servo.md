# Servo API

SG90 servo control via ESP32-S3 LEDC (50 Hz PWM). Drives one servo at a time
for a fixed duration.

---

## Configuration macros (`servo.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `NUM_SERVOS` | `1` | Number of active servos (1-8) |
| `SERVO_DURATION_MS` | `500` | How long to hold position before releasing (ms) |

---

## Pin assignments

| Macro | GPIO | Notes |
|-------|------|-------|
| `SERVO_PIN_0` | 4 | ADC1_3, TOUCH4 |
| `SERVO_PIN_1` | 5 | ADC1_4, TOUCH5 |
| `SERVO_PIN_2` | 6 | ADC1_5, TOUCH6 |
| `SERVO_PIN_3` | 7 | ADC1_6, TOUCH7 |
| `SERVO_PIN_4` | 8 | ADC1_7, TOUCH8 |
| `SERVO_PIN_5` | 9 | ADC1_8, TOUCH9, FSPIHD |
| `SERVO_PIN_6` | 10 | ADC1_9, TOUCH10, FSPICS0 |
| `SERVO_PIN_7` | 11 | ADC2_0, TOUCH11, FSPID |

Only the first `NUM_SERVOS` pins are initialized.

---

## API

### `servo_init()`

Initialize LEDC timer and channels for all active servos. Call once before
any `servo_drive()` calls.

```c
servo_init();
```

### `servo_drive(int id, int angle)`

Drive servo `id` to `angle` degrees. Blocks for `SERVO_DURATION_MS` ms, then
releases the PWM signal.

| Parameter | Range | Description |
|-----------|-------|-------------|
| `id` | `0` to `NUM_SERVOS - 1` | Servo index |
| `angle` | `0` to `180` | Target angle in degrees |

```c
servo_drive(0, 0);    // servo 0 → 0°
servo_drive(0, 90);   // servo 0 → 90° (center)
servo_drive(0, 180);  // servo 0 → 180°
```

Out-of-range `id` logs an error and returns immediately. `angle` is clamped
to [0, 180].

---

## PWM timing (SG90)

| Angle | Pulse width | LEDC duty (14-bit) |
|-------|-------------|-------------------|
| 0°    | 1.0 ms      | 819               |
| 90°   | 1.5 ms      | 1228              |
| 180°  | 2.0 ms      | 1638              |

Period = 20 ms (50 Hz), resolution = 14-bit (0-16383).

---

## Example usage

```c
#include "servo.h"

// in app_main, after system init:
servo_init();

// drive servo 0 to center
servo_drive(0, 90);

// sweep servo 0 from 0° to 180° in steps
for (int angle = 0; angle <= 180; angle += 45) {
    servo_drive(0, angle);
}
```

## Enabling more servos

Change `NUM_SERVOS` in `servo.h` (max 8). No other changes needed - each
index maps directly to an LEDC channel and its corresponding `SERVO_PIN_N`.
