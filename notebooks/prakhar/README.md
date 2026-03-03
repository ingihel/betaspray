# Prakhar Gupta — ECE 445 Lab Notebook

---

## Entry 1 — 2026-02-03

### Objectives
- Confirm ECE 445 project approval
- Begin coordinating team logistics for proposal submission
- Troubleshoot flash integrity issue on microcontroller

---

### Work Record

#### Project Approval

Got confirmation that our ECE 445 project is approved and listed on the course project page. Proposal is due next week, so we need to start immediately. Shared the course project listing with the team.

#### Microcontroller Debugging

Running into weird flash integrity bugs on my ESP32. The flashing process completes but the firmware doesn't behave as expected. Suspect it may be a power issue or corrupt binary. Planning to go to the soldering lab to try re-flashing with cleaner hardware. Coordinated with Max to visit the soldering lab together — he still needs to register for lab access, but I noted the reservations are not strictly enforced. I have a Pinecil, iron, and multimeter at home if needed.

#### Action Items
- Draft initial project proposal this week
- Visit soldering lab to debug flash integrity issue
- Register for soldering lab access (Max)

---

## Entry 2 — 2026-02-13

### Objectives
- Finalize and submit project proposal by 6 PM deadline
- Research existing climbing hold detection datasets
- Establish collaborative document workflow

---

### Work Record

#### Proposal Sprint — ECEB Meeting

Met with Ingi and Max at ECEB around noon to push the proposal over the finish line. Ingi shared several sample ECE 445 proposals from the course project archive as reference material for structure and depth. We agreed to write in LaTeX and use Overleaf so all three of us can edit simultaneously.

Divided up sections and drafted the proposal together, iterating on structure and content throughout the afternoon. Made substantial edits to align the document with our actual design intent before finalizing.

#### Climbing Hold Detection Research

Max found a Roboflow dataset for climbing hold detection: [hold-detector dataset](https://universe.roboflow.com/climb-ai/hold-detector-rnvkl/images/0cOF7EinlDjRntPlZgVF). This is a promising training dataset for the computer vision pipeline. The images show labeled holds on a climbing wall, which is exactly the input domain our system will operate in. Will investigate whether we can use this dataset directly or if we need to fine-tune/augment it.

#### Proposal Submitted

Submitted the final `.tex` source and compiled PDF before the 6 PM deadline.

#### Action Items
- Review Roboflow hold-detector dataset for usability with our CV pipeline
- Begin schematic capture now that the proposal is locked
- Team contract due in one week — start async draft

---

## Entry 3 — 2026-02-17

### Objectives
- Divide PCB schematic tasks among team members
- Set up version-controlled KiCad project repository
- Clarify parts ordering procedure with TA
- Lock in recurring weekly TA meeting time

---

### Work Record

#### PCB Task Division

First PCB submission is due in two weeks. We divided schematic ownership:

| Subsystem | Owner |
|-----------|-------|
| Power subsystem | Ingi |
| Vision/camera subsystem | Prakhar |
| Servo/motor controller subsystem | Max |

For the minimum viable product, the plan is to prioritize getting a working vision subsystem and to validate servo control early using an ESP32 eval board over PWM GPIO before the PCB arrives.

#### Git Repository Setup

Initialized the KiCad project as a Git repository and invited Max (`mlbeach2`). Key constraint discussed: **the KiCad project must reference only a custom symbol library checked into the repo, not any system-level library.** This ensures the project builds correctly on any machine without path dependencies. Ingi set his system libs to read-only as a stopgap but agreed to migrate to the in-repo lib.

#### Parts Ordering

Digikey order was placed through the course. However, the TA clarified that Amazon purchases are out-of-pocket — reimbursements are not generally done, and the course business office has constraints that prevent some orders from going through. I added miscellaneous parts to the parts list this week. Will need to personally purchase any remaining items from Amazon.

#### TA Meeting Schedule

After back-and-forth with the TA, we landed on **Tuesdays at 1 PM** as our recurring weekly meeting slot (TA staff meeting occupies Thursdays 1–2 PM, so Thursday afternoon was also an option). Tentative meeting location: Evo Cafe / ECEB. TA also asked us to share our CAD progress and confirm we've spoken to Greg at the E-Shop.

#### Action Items
- Complete vision subsystem schematic (my task) by end of week
- Export all symbols to in-repo custom KiCad library
- Coordinate E-Shop consultation with Greg re: PCB mechanical constraints
- Purchase remaining Amazon parts out of pocket

---

## Entry 4 — 2026-02-20

### Objectives
- Synchronous work session at ECEB to progress PCB CAD
- Draft team contract
- Confirm weekly TA meeting logistics

---

### Work Record

#### ECEB Work Session

Met with Ingi and Max at ECEB (room 3003 had space) for a synchronous CAD session. Each of us worked on our assigned schematic sheets. Having everyone in the same room made it easier to resolve symbol conflicts and agree on connector pinouts on the spot.

Discussed the need to meet with Greg at the E-Shop soon to confirm PCB mechanical constraints before we finalize the board outline and submit.

#### Team Contract

Drafted and worked on the ECE 445 team contract template (due this week). Covered expectations around meeting attendance, task ownership, and communication norms.

#### TA Meeting Confirmed

Tuesday 1 PM recurring slot with the TA is confirmed. TA also reiterated: no reimbursements for Amazon purchases — anything we buy that doesn't meet the course/business office constraints comes out of pocket. Will factor this into our parts decisions going forward.

#### Action Items
- Finalize schematic sheets before Friday PCB submission deadline
- Meet with Greg (E-Shop) before the PCB submission
- Submit team contract this week
- Ensure all KiCad symbols are in the project-local library before submission

---

## Entry 5 — 2026-02-24

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

---

## Entry 6 — 2026-03-02

### Objectives
- Design document review with Prof. Gruev
- Address feedback and identify follow-up action items

---

### Work Record

#### Design Doc Review — Prof. Gruev

Met with Prof. Gruev for the design document review. Overall the document was in reasonable shape but he had several concrete pieces of feedback to address:

**SG90 Servo voltage domains**: Prof. Gruev flagged that the SG90 control signal (GPIO) and power supply may operate at different voltage levels. Need to go back to the SG90 datasheet and confirm whether the PWM control line can be driven directly from the ESP32-S3's 3.3 V GPIO or whether a level shifter is required between the 3.3 V logic and the 5 V servo supply. This is a potential issue in the current schematic that needs to be resolved before fab.

**Pixel-to-area conversion math**: He asked for a back-of-envelope calculation showing the relationship between camera pixels and physical area on the wall at a fixed projection distance. This is important for validating that our camera resolution is sufficient to localize holds to within our ±5 cm accuracy requirement. Need to work out the field of view, image resolution, and angular resolution to derive a cm/pixel figure at our target standoff distance.

**ESP32 CV feasibility test**: Prof. Gruev wants us to empirically validate that the ESP32-S3 can execute our computer vision algorithm at an acceptable frame rate. Running OpenCV-style inference on-device is non-trivial and we should benchmark it early rather than discovering a performance bottleneck late in the semester.

**PCB submission**: Submit the current PCB design to the TA by end of day.

**SG90 current draw and voltage ripple**: Prof. Gruev asked us to look more carefully at how the SG90 behaves across its operating voltage range and what that means for our 5 V rail. Servos are inductive loads and draw sharp current spikes when they start moving — without bulk capacitance on the supply rail these transients cause voltage droops that can propagate through the lm3940IT-3.3 LDO and destabilize the 3.3 V logic rail. Went through the datasheet and did some analysis after the meeting.

From the SG90 datasheet, current draw varies significantly with operating voltage and mechanical load:

| Condition | 4.8 V | 6.0 V |
|-----------|-------|-------|
| No-load (holding) | ~10 mA | ~15 mA |
| Moderate load (moving) | ~100–200 mA | ~150–250 mA |
| Stall (shaft blocked) | ~650 mA | ~700 mA |

We have 8 servos total (4 gimbals × 2 axes). Idle draw is about 80 mA aggregate, which is fine. The concern is simultaneous slewing — if all gimbals update at once during a projection sequence, aggregate draw could hit 1–1.5 A under normal load. With the ESP32, camera, and lasers also on the USB-C 5 V / 3 A rail, headroom is tight.

Transient ripple is the bigger risk. Back of envelope to hold droop under 200 mV for a single servo stepping from idle to load (ΔI ≈ 200 mA, Δt ≈ 2 ms):

```
C = I·Δt / ΔV = 0.2 × 0.002 / 0.2 = 2000 µF
```

For multiple servos switching together, 4700–10000 µF of bulk capacitance on the 5 V servo rail is warranted, placed close to the servo connector bank. The lm3940IT-3.3 output also needs local decoupling (100 µF + 100 nF) to ride out fast transients before they affect the ESP32. Longer term, an LC filter or separate buck converter for the servo rail would fully isolate servo switching noise from the logic supply.

#### Action Items
- Pull SG90 datasheet and verify GPIO voltage compatibility; add level shifter to schematic if needed
- Compute pixel-to-area (cm/pixel) at fixed camera-to-wall distance and include in design doc
- Run CV benchmark on ESP32-S3 eval board and record frame rate / CPU utilization
- Submit PCB design to Gayatri by EOD
- Add 4700+ µF bulk capacitance to 5 V servo rail in KiCad, near servo connectors
- Add 100 µF + 100 nF decoupling on lm3940IT-3.3 output if not already present
- Evaluate LC filter or separate buck converter to isolate servo 5 V from logic 5 V
