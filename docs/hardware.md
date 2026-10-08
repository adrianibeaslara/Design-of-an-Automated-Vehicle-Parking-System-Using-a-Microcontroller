# Hardware inventory from source

These tables are an inventory of references in the published C files. The exact board variants, pin headers, component models, supply voltages, and full wiring are not established by those files.

## Discovery

| Function | Reference visible in the source |
| --- | --- |
| Entry button | PA11 / EXTI11 |
| Exit button | PA12 / EXTI12 |
| Exit presence input | PC7 / EXTI7 |
| Ultrasonic trigger output | PD2 |
| Ultrasonic echo | PA5 / TIM2 capture |
| Servo outputs | PB6 / TIM4 CH1; PB7 / TIM4 CH2 |
| Buzzer | PA1 / TIM2 |
| Indicator LED | PA4 |
| LCD interface | I2C2 and external HD44780 helper functions |

The source also configures PC6 in the ultrasonic timing section. Reconcile that configuration with PD2 and the original Cube project before treating either comment as a complete connection specification.

## Nucleo

| Parking space | Occupancy input | LED output |
| --- | --- | --- |
| P1 | PC5 | PA0 |
| P2 | PC7 | PA1 |
| P3 | PC9 | PA9 |
| P4 | PC10 | PA8 |
| P5 | PC12 | PA4 |
| P6 | PC13 | PA5 |

Occupancy handlers interpret low input levels as occupied and high levels as available. The LCD is accessed through I2C1 and the external HD44780 helper functions.

## Needed to reproduce the physical system

1. Exact Discovery/Nucleo part numbers and original Cube configuration.
2. Schematics or an independently checked wiring table, including LCD I2C pins.
3. Sensor and servo models, electrical specifications, and power arrangement.
4. The LCD driver implementation and its configuration.
5. Original measurements or a new bench validation of detection, timing, and gate movement.

The inventory above is enough to navigate the code; it is not a verified assembly guide.
