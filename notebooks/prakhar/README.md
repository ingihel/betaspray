# Prakhar Gupta — ECE 445 Lab Notebook

---

## Entry 1 — 2026-02-24

### Objectives
- Review and discuss the overall project design with TA
- Begin schematic capture for the vision mapping subsystem in KiCad
- Consult with the machine shop (Greg) regarding PCB mechanical constraints
- Identify next steps for the first PCB submission round

---

### Work Record

#### Team Discussion — System Overview

Met with Ingi, Max and Gayatri (our TA) to walk through the proposal and confirm subsystem responsibilities. Reviewed the high-level modules:

- **Vision Mapping Subsystem**: Camera module → ESP32-S3 (image capture, CV pipeline for hold detection)
- **Projection Subsystem**: ESP32-S3 PWM → 4× 2-axis servo gimbals + laser modules
- **User Interface Subsystem**: ESP32-S3 Wi-Fi HTTP server <--> web/mobile client

Discussed requirements for upcoming deadlines, i.e design document and design doc review.

Also discussed the vision-in-the-loop servo correction scheme: the camera detects the laser spot position on the wall, computes a 2D error vector relative to the intended hold coordinate, and sends corrective PWM adjustments to the gimbal. 

We also discussed staggering the project into a moduled way, so we have a solid MVP as a backup.

#### Vision Subsystem Schematic 

Started the vision subsystemschematic sheet.
The camera selected for this schematic is the **OV5640** with autofocus.  Autofocus should provide incremental improvements 
to computer vision for warped wall surfaces/ increasing distance from the wall.


The OV5640 uses a **DVP (Digital Video Port) parallel interface** to transfer image data to the ESP32-S3. Our purchased part 
comes with a breakout board, so for the first revision of the PCB i used two 9-pin connectors to interface the camera. The pin mapping is:

| Signal | Direction | Description |
|--------|-----------|-------------|
| D2–D9  | Input     | 8-bit parallel pixel data bus |
| HS     | Input     | HREF — horizontal reference (line valid) |
| VS     | Input     | VSYNC — vertical sync (frame valid) |
| PC     | Input     | PCLK — pixel clock |
| XC     | Input     | XCLK — external master clock from ESP32-S3 |
| SDA    | Bidirectional | I²C data (SCCB) for camera configuration |
| SCL    | Input     | I²C clock (SCCB) for camera configuration |
| PD     | Input     | Power Down — active-high to put camera in low-power mode |
| RT     | Input     | Reset — active-low reset to camera |
| G      | —         | Ground reference |
| 3V     | —         | 3.3V supply to camera module |

**Design note**:Left the XCLK line as unconnected. It is probably easier for us to use the internal 24 mhz clk 

I also added a schematic note flagging that a VM (voice-coil motor) pin for the OV5640's autofocus actuator is *not* connected in this revision — autofocus motor control is deferred to a future PCB revision if needed.

Also added a schematic annotation listing ESP32-S3-DevKitC-1 pins that must be avoided:
- **Strapping pins** (never connect hardware): GPIO0, GPIO3, GPIO45, GPIO46
- **PSRAM SPI pins** (N8R8 variant only): GPIO35, GPIO36, GPIO37
- **UART port**: GPIO43, GPIO44
- **USB port**: GPIO19, GPIO20
- **Secondary JTAG**: GPIO39–42 (usable as GPIO but reserve if JTAG debugging needed)
- **RGB LED**: GPIO38 (onboard LED, can reuse as GPIO)

Using these restraints, i was able to successfully do a pin assignment.

I found this reference: [ESP32 Webcam with Autofocus (Adafruit OV5640)](https://www.instructables.com/ESP32-Webcam-With-Autofocus-Using-Adafruit-Ov5640-/) 
which was super helpful

#### Machine Shop Consultation (Greg)

Spoke with Greg in the machine shop to understand constraints for the first PCB order. He asked for:
1. **Board dimensions** — need to finalize the PCB outline. Target is fitting within the 10 cm × 10 cm device footprint; board itself will be smaller (~10 × 15 cm per BOM estimate).
2. **Mounting holes** — need to add 1/8 in mounting holes to the KiCad PCB layout for mechanical attachment to the enclosure base. Holes must be quarter inch
from the corners

#### To-Do / Action Items

Ahead of the meeting next week, I need to sanity check my PCB schematic, route the signals,
and work on a design doc. The design doc needs to incorporate the change of basis / transform
math and/or code so we can demonstrate that using PWM control over the servos is feasible.

Probably also need to modularize the system components and make FSMs for each subsystem, and
make a plan of attack so as to integrate these correctly.

Lastly, need to look into HAL based module to allow toggling of openloop / closedloop control 
for the servos

