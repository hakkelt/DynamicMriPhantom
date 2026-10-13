# Dynamic MRI Phantom Controller

Arduino-based stepper motor controller for synchronized phantom motion in dynamic MRI acquisitions. Creates known, repeatable motion patterns for validating reconstruction algorithms (e.g., golden-angle radial, compressed sensing).

## Overview

This controller enables ground-truth validation of dynamic MRI reconstructions by:

- **Repeatable motion**: Deterministic alternating rotation pattern (clockwise 75 RPM, counterclockwise 150 RPM)
- **Synchronization options**: Currently button-triggered; TTL-based MRI synchronization to be added
- **Known trajectory**: Exact motion parameters enable precise ground-truth calculation
- **Motion cycle**: ~1.2 seconds per complete rotation pattern for acquisition planning

**Current triggering**: Push button on pin 5 starts continuous rotation until Arduino reset

## Hardware

- Arduino Nano (ATmega328)
- Stepper motor: 200 steps/revolution (1.8° step angle)
- DRV8825 or A4988 driver (DIR/STEP interface)
- Push button for triggering

**Pin Connections:**

| Pin | Connection |
|-----|------------|
| D2  | Motor driver DIR |
| D3  | Motor driver STEP |
| D5  | Push button (with internal pull-up) |

For complete wiring with DRV8825 driver, see: [DRV8825 Arduino Tutorial](https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/)

## Motion Pattern

Continuous alternating pattern triggered by TTL rising edge:

1. **Clockwise**: 200 steps @ 2000μs/step (75 RPM, 0.8s)
2. **Counterclockwise**: 200 steps @ 1000μs/step (150 RPM, 0.4s)

**Cycle time**: 1.2 seconds per complete CW+CCW cycle

## Building and Uploading

Requires [PlatformIO IDE](https://platformio.org/) (VS Code recommended).

```bash
# Clone and open
git clone <repository-url>
cd "Dynamic MRI Phantom"
code .

# Build and upload
pio run --target upload
```

## Usage

1. Wire hardware according to pin table and [DRV8825 tutorial](https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/)
2. Upload firmware to Arduino Nano
3. Connect push button between pin 5 and GND
4. Power on system
5. Press button to start continuous rotation
6. Press Arduino reset button to stop motion

**Note**: Button-based triggering will remain available as an option alongside TTL synchronization when added in future versions.

## Configuration

Edit [src/main.cpp](src/main.cpp) to modify:

- `stepsPerRevolution` - Motor steps per revolution (default: 200)
- Delay values (2000, 1000) - Rotation speeds
- Pin assignments (`dirPin`, `stepPin`, `buttonPin`)
- Motion pattern logic in `loop()`

## Notes

- Button uses Arduino internal pull-up resistor (connect one side to pin 5, other to GND)
- Stepper driver current limit must be set according to motor specifications
- Motor step timing is deterministic (microsecond-level resolution) for accurate ground-truth generation
- For MRI use: ensure MRI-compatible components or use long cables to position electronics outside scanner room