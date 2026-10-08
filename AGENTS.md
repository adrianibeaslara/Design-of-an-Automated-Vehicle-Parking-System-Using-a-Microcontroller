# Repository instructions

This repository documents a historical STM32 parking prototype.

- Read docs/architecture.md, docs/hardware.md, and docs/review.md before changing firmware.
- Keep Discovery and Nucleo application entry points separate.
- Preserve STMicroelectronics notices and Cube USER CODE markers.
- Do not invent board models, clock frequencies, schematics, measured results, or successful builds.
- Changes to interrupts, pin configuration, sensor interpretation, or timing require target-specific evidence and hardware validation.
- Deterministic helpers may be changed with meaningful host tests.
- Run: python3 tests/check_display_format.py
- Format C with clang-format 18.1.8 using .clang-format; check with --dry-run --Werror.
- Explain the actual scope of validation. Host checks do not validate STM32 firmware.
- Keep credentials, build output, and local IDE metadata out of Git.
