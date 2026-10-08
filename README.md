# Automated Parking System

STM32 firmware for a six-space parking prototype, split across **Discovery** and **Nucleo** boards. The application combines gate control, vehicle detection, occupancy indicators, and LCD guidance.

**Status:** the repository contains the two application entry points and source-based documentation. The original Cube project, supporting drivers, and wiring diagrams are still needed for a complete firmware build.

## What it does

| Board | Responsibilities | Application |
| --- | --- | --- |
| Discovery | Entry/exit gates, ultrasonic distance measurement, buzzer, and free-space display | [main.c](firmware/discovery/main.c) |
| Nucleo | Six occupancy inputs, per-space LEDs, and guidance toward the less occupied side | [main.c](firmware/nucleo/main.c) |

The two applications have separate state. The published code does not implement a communication link between the boards.

## Explore the project

- [Architecture and control flow](docs/architecture.md)
- [Pins and peripherals visible in the source](docs/hardware.md)
- [Development, checks, and build requirements](docs/development.md)
- [Technical review and remaining work](docs/review.md)

```text
firmware/
  discovery/main.c
  nucleo/main.c
docs/
  architecture.md
  hardware.md
  development.md
  review.md
tests/
  check_display_format.py
```

## Run the available checks

The display-formatting check runs on a Linux host with Python 3 and GCC:

```bash
python3 tests/check_display_format.py
```

It checks the isolated C helper used by the Discovery LCD. It does not compile the STM32 applications or validate the physical system. Formatting uses clang-format 18.1.8 and the repository's [.clang-format](.clang-format).

## Source history

The original `DISCOVERY_BOARDmain.c` and `NUCLEO_BOARDmain.c` files were moved into board-specific directories. Their STM32 `USER CODE` boundaries and vendor notices are preserved. The 2026-10-08 maintenance pass standardizes formatting and removes repeated heap allocation from the LCD conversion helper.

The original STMicroelectronics notices remain in the source. A repository-wide reuse license has not been selected.
