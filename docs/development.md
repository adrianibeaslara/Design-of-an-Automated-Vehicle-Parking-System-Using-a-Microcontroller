# Development and validation

## Application layout

Each board has its own main.c. Import the relevant file into the matching STM32 project; compiling the two entry points into one application is not the intended layout.

## What is missing for a target build

- main.h, device headers, CMSIS and STM32 HAL sources.
- liquidcrystal_i2c.h and its implementation.
- Original Cube .ioc settings or equivalent target configuration.
- Startup files, linker script, and interrupt integration.
- A documented compiler/toolchain version and board selection.

The repository therefore does not offer a complete firmware build command. Recover these dependencies before adding target CI or claiming a successful STM32 build.

## Formatting

The maintenance pass uses clang-format 18.1.8. With that version installed:

```bash
clang-format --dry-run --Werror firmware/discovery/main.c firmware/nucleo/main.c
clang-format -i firmware/discovery/main.c firmware/nucleo/main.c
```

Use the first command to check formatting and the second only when applying changes.

## Host check

Requires Linux, Python 3, and GCC:

```bash
python3 tests/check_display_format.py
```

The check extracts the actual Discovery LCD helper, compiles it as C11 with warnings and AddressSanitizer/UndefinedBehaviorSanitizer, and validates two-digit formatting for 0–99 and repeated calls. The helper returns a shared buffer for synchronous use by the main loop; it is not reentrant.

This check cannot establish sensor accuracy, ISR correctness, actuator timing, or board-level behavior.

## Hardware validation to add when the project is recovered

Record board and toolchain versions, then verify entry/exit events, counter bounds, input bounce, all six occupancy inputs, LCD behavior, servo positions, and timing under compiler optimization.
