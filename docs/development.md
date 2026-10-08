# Development and validation

## Application layout

Each board has its own `main.c`. Import the relevant file into a target project for **STM32L-DISCOVERY / STM32L152RB** or **NUCLEO-L152RE / STM32L152RE**. Compiling both entry points into one application is not the intended layout.

The source combines Cube-style HAL initialization with manual register configuration. The thesis explains the peripheral design, including calculations assuming a 32 MHz timer clock. It does not identify a reproducible compiler/IDE version or supply the complete target configuration.

## Missing target-build dependencies

- `main.h`, device headers, CMSIS and STM32 HAL sources.
- `liquidcrystal_i2c.h` and its implementation/configuration.
- Original Cube `.ioc` settings or equivalent target configuration.
- Startup files, linker scripts, HAL MSP configuration and interrupt integration.
- Compiler/toolchain versions and a build command for each board.

Thesis bibliography item [13] cites [eziya/STM32_HAL_I2C_HD44780](https://github.com/eziya/STM32_HAL_I2C_HD44780) as an LCD-library reference. The original project's exact revision and local adaptations have not been recovered; linking it does not establish drop-in compatibility.

There is therefore no complete STM32 build command in this repository. Recover and check these dependencies before introducing target CI.

## Formatting

With **clang-format 18.1.8** installed:

```bash
clang-format --dry-run --Werror firmware/discovery/main.c firmware/nucleo/main.c
clang-format -i firmware/discovery/main.c firmware/nucleo/main.c
```

The first command checks formatting; the second applies it using the repository's `.clang-format`.

## Host check

Requires Linux, Python 3, and GCC:

```bash
python3 tests/check_display_format.py
```

The check extracts the actual Discovery LCD helper, compiles it as C11 with warnings and AddressSanitizer/UndefinedBehaviorSanitizer, and validates two-digit formatting for 0–99 and repeated calls. The helper returns a shared buffer for synchronous main-loop use; it is not reentrant.

This check cannot establish sensor accuracy, ISR correctness, actuator timing or board-level behavior. It also does not validate the historical firmware version used for the thesis demonstrations.

## Documentation and image checks

For a documentation change, verify local links, image decoding, attribution and rendering. Avoid presenting checks from an earlier firmware maintenance pass as newly performed hardware tests.

The source photographs and diagrams under `docs/assets/` are unchanged embedded figures extracted from the supplied thesis PDF. Their provenance is indexed in [assets/README.md](assets/README.md).

## Bench work after project recovery

Record board revisions and toolchain settings, then verify:

- Actual peripheral clock, PWM period/pulses, ultrasonic trigger/echo and timer wraparound.
- Entry/exit events, the valid 0–6 counter range, and behavior with the physical RC filter.
- All six occupancy inputs under different lighting conditions.
- LCD addresses, power behavior, servo endpoints and vehicle/barrier clearance.

The [historical results](results.md) are a starting point for this work, not a substitute for it.
