# PCB-Range-Finder

# PCB Ultrasonic Range Finder System

A microcontroller-based ultrasonic distance measurement system with integrated voltage regulation, LCD display output, and real-time user control via an analog input. The system measures distance using an ultrasonic transducer, processes the signal on an MSP430-class microcontroller, and displays distance and control parameters on an I²C LCD interface.

---

## Overview

This project implements a complete embedded sensing pipeline:

- Generates a 10 µs ultrasonic trigger pulse
- Measures echo return time with microsecond resolution
- Converts time-of-flight into distance (mm and inches)
- Reads an analog potentiometer via ADC and maps it to a duty percentage
- Displays all information on a 2-line LCD
- Uses a custom PCB with onboard voltage regulation and signal conditioning

The design is modular and scalable, separating hardware abstraction, sensor handling, display logic, and application logic.

---

## System Architecture

**Inputs**
- Ultrasonic distance sensor (Trigger / Echo)
- Potentiometer (ADC input)

**Outputs**
- I²C LCD (distance + control value)
- Status LEDs

**Power**
- External supply → Linear voltage regulator → 3.3V system rail

**Core Components**
- Microcontroller (MSP430 family)
- Ultrasonic transducer module
- I²C LCD module
- Adjustable potentiometer
- Linear voltage regulator

---
## Firmware Structure

---


### Key Responsibilities

| File | Purpose |
|------|----------|
| `main.c` | Coordinates sensor reading, computation, and display |
| `Junior_Design.*` | Abstracts timers, GPIO, ADC, and delays |
| `lcd_i2c.c` | Handles LCD initialization and I²C communication |
| `ultrasonic.c` | Generates trigger pulses and measures echo timing |

---

## Distance Computation

Echo time is measured in 10 µs timer counts.

Distance is calculated using:

- **Millimeters:**
distance_mm = (counts × 3.4 mm) / 2

- **Inches:**
distance_in = counts × 0.0669


The division by 2 accounts for round-trip travel of the ultrasonic pulse.

---

## User Interface

LCD Output:

Line 1: "Junior Design"
Line 2: "XXXmm YY.Zin ZZ%"


Where:
- `XXXmm` is distance in millimeters
- `YY.Zin` is distance in inches with one decimal
- `ZZ%` is the scaled potentiometer duty value

A blinking LED indicates system activity.

---

## Power Regulation

The PCB includes an onboard linear regulator to provide a stable 3.3 V rail for the microcontroller and peripherals. Load testing was performed using a known resistor and measured voltage drop to verify current output capability and thermal behavior.

---

## Build and Flash

1. Clone or copy the repository.
2. Open the firmware folder in your embedded IDE.
3. Ensure the support drivers (`Junior_Design.*`, `lcd_i2c.c`, `ultrasonic.c`) are included in the build.
4. Compile and flash to the target microcontroller.

---

## Notes

- The project uses blocking delays for simplicity and determinism.
- Timing accuracy depends on correct clock initialization in `Init_HW()`.
- Ultrasonic accuracy is affected by temperature and target reflectivity.

---

## Future Improvements

- Replace blocking delays with interrupts
- Add temperature compensation for speed of sound
- Implement digital filtering for echo noise rejection
- Add serial logging or USB output

---

## License

This project is provided for educational and experimental use. Modify and extend freely.


## Firmware Structure

