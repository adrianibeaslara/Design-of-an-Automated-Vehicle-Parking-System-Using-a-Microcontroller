# Technical review — 2026-10-08

This review combines inspection of the published source with the completed 2023–2024 thesis. The hardware was not retested during repository maintenance, and the complete build project remains unavailable.

## Corrected during source maintenance

**Repeated allocation in the Discovery LCD helper.** `int_to_string` previously allocated three bytes on every call, without a corresponding release at the LCD call site. It now uses a fixed three-byte static buffer and an unsigned format specifier. The earlier host check validated two-digit output for 0–99 and repeated calls.

**Navigation and style.** The entry points live in board-specific directories. Formatting is consistent; Cube `USER CODE` boundaries and STMicroelectronics notices are retained.

The thesis documentation update does not change the firmware.

## What the thesis resolves

- Exact board variants and the main sensor/actuator models are now identified.
- The prototype was physically built and its operation documented in chapter 10.
- Button bounce was observed during the project and mitigated with a **hardware low-pass RC filter** (§10.2). The unused debounce constant in the source does not mean the historical prototype had no mitigation.
- Original pin maps, software flowcharts and prototype photographs are now included.
- The detailed Discovery LCD description identifies I2C2 despite the I2C1 label in figure 7.1.

See [hardware](hardware.md) and [historical results](results.md) for source references.

## Remaining engineering work

| Observation | Follow-up |
| --- | --- |
| Entry/exit edges update an unsigned counter without bounds | Define valid transitions and enforce the 0–6 range after checking intended gate behavior |
| No software debounce guard; thesis relies on hardware RC filtering | Recover R/C values and check the actual input waveform and counting under bounce |
| Interrupts and main loops share ordinary global variables | Review volatile access, atomicity and event handling for the actual MCU/compiler |
| Nucleo repeats GPIO/I2C initialization after manual register setup | Check whether generated initialization overwrites intended configuration |
| Boards maintain separate counts | Document the intended consistency behavior before adding a communication link |
| Timing comments conflict with register calculations | Check the clock and signals; the stated 32 MHz assumption gives 50 Hz servo PWM and 64 ms buzzer compare intervals |
| Discovery ultrasonic setup references PC6 as well as PD2 | Reconcile source, original target configuration and physical connections |
| Custom interrupt handlers are in main.c | Check startup vectors and generated interrupt sources for duplicate definitions |
| PDF electrical figures 6.1/6.2 contain filename placeholders | Recover the actual drawings and produce an independently checked assembly record |

These are follow-up items, not validated hardware fixes. The thesis provides qualitative demonstrations and explicitly discusses IR sensitivity, barrier clearance and power effects. It does not establish quantitative reliability or production readiness.

No full STM32 compilation or fresh bench validation was performed during this documentation update.
