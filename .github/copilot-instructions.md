# Copilot Instructions for Ash_Trailer ESP-IDF Project

## Project Overview
This project is an ESP32C6 embedded system application using the ESP-IDF framework. It features BLE, WiFi, stepper motor control, and JSON parsing. The main logic resides in `main/main.c`, with hardware abstraction and motor control in `components/stepper_motor/`.

## Architecture & Components
- **main/**: Entry point (`main.c`). Integrates BLE, WiFi, and stepper motor logic.
- **components/stepper_motor/**: Custom stepper motor driver. Key files: `stepper_motor.c`, `stepper_motor.h`.
- **components/cJSON/**: JSON parsing (external library).
- **build/**: CMake and ESP-IDF build artifacts.
- **CMakeLists.txt**: Top-level and per-component build configuration.

## Build & Flash Workflow
- Use ESP-IDF tools (`idf.py build`, `idf.py flash`, `idf.py monitor`).
- CMake-based build system. Do not use legacy Makefiles.
- All dependencies (e.g., FreeRTOS, BLE, WiFi, cJSON) are managed via ESP-IDF and CMake.
- Example build command:
  ```bash
  idf.py build
  idf.py flash
  idf.py monitor
  ```

## Key Patterns & Conventions
- **FreeRTOS**: Tasks, queues, semaphores for concurrency. See `main.c` and motor driver.
- **BLE**: Uses NimBLE stack. Service/characteristic UUIDs defined in `main.c`.
- **Stepper Motor**: Control via GPIO, with timing and movement logic in `stepper_motor.c`.
- **Logging**: Use ESP-IDF's `ESP_LOG*` macros for debug output.
- **Configuration**: Motor parameters and pins are defined as macros in `stepper_motor.h`.
- **External Libraries**: cJSON is included as a component.

## Integration Points
- **BLE <-> Motor**: BLE commands can trigger motor actions (see `main.c`).
- **WiFi/HTTP**: Network features are present; see `main.c` for setup and usage.
- **Persistent Storage**: Uses NVS (Non-Volatile Storage) via ESP-IDF.

## Debugging & Testing
- Use `idf.py monitor` for serial output and debugging.
- GDB support available via ESP-IDF (see `build/gdbinit/`).
- No formal unit tests detected; test via hardware and serial logs.

## Examples
- To add a new hardware driver, create a new component in `components/`, register in `main/CMakeLists.txt`, and include in `main.c`.
- To extend BLE functionality, update UUIDs and handlers in `main.c`.

## References
- ESP-IDF documentation: https://docs.espressif.com/projects/esp-idf/en/latest/
- Project README: `README.md`
- Stepper motor driver: `components/stepper_motor/stepper_motor.c`

---
*Update this file as project architecture or workflows evolve. Focus on actionable, project-specific guidance for AI agents.*
