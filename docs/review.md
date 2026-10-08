# Technical review — 2026-10-08

This review is based on the published source. The original hardware and complete build project were unavailable.

## Corrected in this maintenance pass

**Repeated allocation in the Discovery LCD helper.** int_to_string previously allocated three bytes on every call. The LCD call site did not release that allocation. The helper now uses a fixed three-byte static buffer and an unsigned format specifier. The intended two-digit output is preserved, with host checks for all values from 0 to 99 and repeated calls.

**Repository navigation and style.** The two application files now live under board-specific directories. Formatting is consistent; generated USER CODE boundaries and vendor notices are retained.

## Remaining engineering work

| Observation in the source | Required follow-up |
| --- | --- |
| Entry/exit edges update an unsigned counter without bounds | Define valid transitions and enforce the 0–6 range after checking intended gate behavior |
| A debounce constant is defined but does not guard the button handlers | Validate bounce behavior and introduce a measured debounce strategy |
| Interrupt handlers and main loops share ordinary global variables | Review volatile access, atomicity, and event handling for the actual MCU/compiler |
| Nucleo repeats GPIO/I2C initialization after manual register setup | Check whether generated initialization overwrites the intended configuration |
| The two boards maintain separate counts | Confirm whether independent indicators were intentional or whether communication is needed |
| Register values and timing comments depend on missing target configuration | Reconcile clocks, prescalers, capture ranges, and overflow behavior on the real target |
| Custom interrupt handlers are present in main.c | Integrate them with the project's interrupt source and startup vectors without duplicate definitions |

These observations are documented rather than presented as validated hardware fixes. No full STM32 compilation, bench test, or reliability measurement was performed.
