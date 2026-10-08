# Thesis results and limitations

This page summarizes **historical functional demonstrations reported in chapter 10** of the completed 2023–2024 thesis. It does not report a new execution of the maintained sources or a quantitative evaluation.

## Reported demonstrations

| Area | Thesis-reported observation | Reference |
| --- | --- | --- |
| Entry gate | Ultrasonic distance influences proximity warning; a button participates in gate control | §10.1, printed pages 81–82 |
| Exit gate | IR presence detection and button input control the exit barrier | §10.1, printed page 82 |
| Available-space count | Button events change the LCD count; contact bounce initially caused multiple updates | §10.2, printed pages 82–84 |
| Hardware debounce | A low-pass RC filter stabilizes the button signal; the corrected demonstration reports one count update per press | §10.2, printed page 84 |
| Occupancy LEDs | Bay IR inputs are processed by the MCU before the corresponding LEDs are activated | §10.3, printed pages 84–85 |
| Guidance display | Occupied bay labels and arrows change with the sensed occupancy distribution | §10.3, printed pages 84–85 |
| Complete sequences | Entry, parking and exit, multiple vehicles, and a four-vehicle/two-free-space case are described | §10.4, printed page 86 |

The [photographs](gallery.md) provide an original visual record of the completed assembly. Still images alone do not establish correct timing or reliable detection.

## Lessons recorded during the project

**Input conditioning matters.** Raw button edges produced repeated interrupts. The thesis reports a hardware RC solution; reproducing that behavior requires its component values and wiring, rather than relying on the unused debounce constant in the published code.

**IR calibration is part of the prototype setup.** Ambient light affected detection, requiring adjustment of the sensor potentiometers. No dataset quantifying sensitivity or error rate is supplied.

**Mechanical clearance limits gate behavior.** The entry sensor/barrier spacing and exit sensor position create practical vehicle-clearance constraints. The thesis identifies further mechanical adjustment as useful.

**Power changes actuator behavior.** Battery supply conditions affected exit-barrier speed. This is an observed prototype limitation, not a characterized or controlled speed-regulation mechanism.

## Evaluation scope

| Evidence | Availability in the supplied thesis |
| --- | --- |
| Physical prototype photographs | Available and reproduced in this repository |
| Functional narratives and video references | Available in chapter 10 |
| Labeled test dataset, repeat counts or automated assertions | Not supplied |
| Measured accuracy, failure rate, latency or current consumption | No quantitative benchmark supplied |
| Complete target build and configuration | Not restored by the PDF |
| New test of the maintained firmware on STM32 hardware | Not performed |

Two descriptions need care when interpreting the thesis: one multi-vehicle narrative repeats the same bay label for different cars, and several timer/I²C labels conflict with the more detailed description or source. These inconsistencies are not treated as new measured evidence.

## Historical video references

The thesis links three complete-sequence demonstrations:

- [Sequence 1](https://digistorage.net/sodgcv4s)
- [Sequence 2](https://digistorage.net/u2zxhg62)
- [Sequence 3](https://digistorage.net/rzrqqmgw)

It also links the [RC-filter counting demonstration](https://digistorage.net/c4tyn80x).

These are original external references. Their playback availability could not be verified during this documentation update; the results summarized above come from the thesis text, not a fresh viewing of these videos.

## Next reproducible validation

Recover the Cube projects and complete wiring first. Then record the toolchain, compiler settings and power arrangement; verify timer signals and input conditioning; and measure repeated gate/count/occupancy scenarios under controlled lighting and supply conditions. Define valid counter transitions and quantitative acceptance criteria before drawing reliability conclusions.

[Technical source review](review.md) · [Development requirements](development.md)
