# Servo API

SG90 servo control via ESP32-S3 LEDC (50 Hz PWM, 14-bit resolution).
Uses the LEDC hardware fade engine for smooth, jitter-free position transitions.

---

## Configuration macros (`servo.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `NUM_SERVOS` | `2` | Number of active servos (1–8) |
| `SERVO_FADE_TIME_MS` | `2000` | Duration (ms) of a full 180° hardware fade. Partial moves scale proportionally. |
| `SERVO_DURATION_MS` | `500` | How long to hold position after reaching target before releasing PWM (ms) |

---

## Pin assignments

| Macro | GPIO | Notes |
|-------|------|-------|
| `SERVO_PIN_0` | 1 | ADC1_0 — X-axis servo |
| `SERVO_PIN_1` | 2 | ADC1_1, TOUCH2 — Y-axis servo |
| `SERVO_PIN_2` | 6 | ADC1_5, TOUCH6 |
| `SERVO_PIN_3` | 7 | ADC1_6, TOUCH7 |
| `SERVO_PIN_4` | 8 | ADC1_7, TOUCH8 |
| `SERVO_PIN_5` | 9 | ADC1_8, TOUCH9, FSPIHD |
| `SERVO_PIN_6` | 10 | ADC1_9, TOUCH10, FSPICS0 |
| `SERVO_PIN_7` | 11 | ADC2_0, TOUCH11, FSPID |

Only the first `NUM_SERVOS` channels are initialised. GPIO 5–11 conflict with
camera pins — do not increase `NUM_SERVOS` beyond 2 without first reassigning
those pins.

---

## API

### `servo_init()`

Configures the LEDC timer, one channel per active servo, and installs the LEDC
fade ISR. Must be called once before any other servo function.

```c
servo_init();
```

### `servo_drive(int id, int angle)`

Moves servo `id` to `angle` degrees using a hardware-faded PWM ramp, holds for
`SERVO_DURATION_MS`, then releases the PWM signal (duty → 0).

| Parameter | Range | Description |
|-----------|-------|-------------|
| `id` | `0` to `NUM_SERVOS - 1` | Servo index |
| `angle` | `0` to `180` | Target angle in degrees (clamped) |

```c
servo_drive(0, 0);    // servo 0 → 0°
servo_drive(0, 90);   // servo 0 → 90°
servo_drive(0, 180);  // servo 0 → 180°
```

**Behaviour details — important for future modifications:**

- **Position tracking:** `servo_current_angle[id]` records the last commanded
  angle. On the first call (`from == -1`) or when the target equals the current
  position, the duty is set directly (no fade). All subsequent calls fade from
  the tracked position.

- **Duty-restore before fade:** After each call the PWM is released (duty set
  to 0). On the next call, before starting the hardware fade, the duty register
  is restored to `angle_to_duty(from)` and the code waits `SERVO_PERIOD_MS + 5`
  ms (`vTaskDelay`). This is necessary because `ledc_update_duty` is
  non-blocking — the hardware applies the new duty at the next PWM period
  boundary. If `ledc_set_fade_with_time` is called before that boundary, it
  reads duty = 0 and the fade starts from the wrong level, causing the servo
  to snap. **If `LEDC_FREQ_HZ` is changed, `SERVO_PERIOD_MS` updates
  automatically and this delay stays correct.**

- **Fade floor:** `fade_ms` is floored at `SERVO_PERIOD_MS` (one PWM period)
  to prevent zero-duration fades on very small angle deltas.

- **Blocking:** `servo_drive` blocks the calling task for the full fade
  duration plus `SERVO_DURATION_MS`. Do not call from a time-critical task
  or ISR.

- **Release:** After hold, duty is set to 0. The servo is unpowered between
  calls. This prevents stall current at 3.3 V but means the servo does not
  hold position under load between calls.

### `servo_testbench_x(int id)`

Exercises a full-range (0–180°) servo through two sequences:
- Phase 1 — 15° increments: 0, 15, 30 … 180
- Phase 2 — coarse jumps: 30 → 45 → 90 → 180

### `servo_testbench_y(int id)`

Exercises a 90°-range servo whose flat position is at 90° and straight-up is
at 180°. All angles are offset +90° relative to the X testbench.
- Phase 1 — 15° increments: 90, 105, 120 … 180
- Phase 2 — coarse jumps: 120 → 135 → 180

---

## PWM timing (SG90)

| Angle | Pulse width | LEDC duty (14-bit, 50 Hz) |
|-------|-------------|--------------------------|
| 0°    | 0.5 ms      | 410                      |
| 90°   | 1.45 ms     | 1188                     |
| 180°  | 2.4 ms      | 1966                     |

Period = 20 ms (50 Hz), resolution = 14-bit (0–16383).

`DUTY_MIN` (410) and `DUTY_MAX` (1966) use the extended SG90 pulse range
(0.5–2.4 ms). The original 1–2 ms range only achieves ~90° of travel.
At 3.3 V, back off slightly from the extremes if the servo buzzes or binds.

---

## Enabling more servos

Change `NUM_SERVOS` in `servo.h` (max 8). Note: GPIO 5–11 conflict with
camera pins. Safe additions beyond servo1 require PCB rework.
