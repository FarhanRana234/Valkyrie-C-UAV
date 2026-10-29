# Wiring and pin layout

Rails: **24 V** (adapter), **5 V** (LM2596), **3.3 V** (ESP32 3V3 pin), **GND** (one common ground for everything).
Boards: ESP32 DevKit (30/38-pin), MKS TMC2209 V2.0 (tilt / pitch), L298N dual H-bridge driver (pan), 5 V relay module, Hall sensor module(s), servo.

## 1. ESP32 pin map (all pins in one place)
| GPIO | Connects to | Direction |
|---|---|---|
| 5 | TMC2209 EN (active low) | out |
| 13 | Dart pusher servo signal | out (PWM) |
| 16 | TMC2209 PDN_UART (RX2, direct) | in |
| 17 | TMC2209 PDN_UART (TX2, through 1 kOhm) | out |
| 18 | TMC2209 STEP | out |
| 19 | TMC2209 DIR | out |
| 21 | L298N IN1 | out |
| 22 | L298N IN2 | out |
| 23 | TMC2209 DIAG (StallGuard) | in |
| 25 | L298N IN3 | out |
| 26 | L298N IN4 | out |
| 27 | Relay IN (flywheels) | out |
| 32 | Hall module #1 DO (dart present) | in |
| 34 | Hall module #1 AO (sensor health check) | in (ADC, input only) |
| 33 | Hall module #2 DO (optional: tilt 90 degree home) | in |
| 1, 3 | USB serial to the PC (reserved) | do not use |
| 0, 2, 12, 15, 6-11 | boot / flash pins | do not use |
| 3V3 | TMC2209 VIO, Hall modules VCC | power out |
| VIN (5 V) | from LM2596 OUT+ | power in |
| GND | common ground | |

## 2. Power
```
24 V adapter (+) --[5 A fuse]--+--> LM2596 IN+            (always on: keeps ESP32 / PC link alive)
                               |
                               +--[emergency-stop switch]--> MOTOR 24 V rail:
                                                              TMC2209 VMOT, L298N 12V/VM, relay COM
24 V adapter (-) ------------------------------------------> GND (common)
LM2596 IN-  -> GND         LM2596 OUT- -> GND
LM2596 OUT+ -> 5 V rail    (set to 5.0 V with a multimeter BEFORE connecting anything)
5 V rail -> ESP32 VIN, relay module VCC, servo red wire, L298N 5V Terminal
```
- 100 uF (35 V or higher) electrolytic across VMOT and GND at the TMC2209 driver.
- 470 to 1000 uF across the 5 V rail near the servo.
- 0.1 uF ceramic across the terminals of each 130 motor.
- ESP32 USB cable to the PC carries data. If the ESP32 is fed from VIN and USB together, use a board with a diode between them.

## 3. 6-Wire Stepper Wiring Logic (Bipolar Mode)
A 6-wire stepper consists of two center-tapped coils. To drive it with modern 4-pin bipolar drivers (TMC2209 & L298N):
- Identify Phase A (2 wires with full resistance, e.g., ~7.0 $\Omega$ end-to-end or ~3.5 $\Omega$ to center tap).
- Identify Phase B (2 wires with full resistance across the second coil).
- **Center Taps (Red & Black wires):** Leave disconnected and individually insulated with heat-shrink or electrical tape.
- Use only the 4 end-of-coil wires to connect to driver outputs.

## 4. Pitch / Tilt Axis: MKS TMC2209 V2.0 + 6-Wire Tamagawa Seiki NEMA 17
| Driver pin | Connects to |
|---|---|
| VMOT | motor 24 V rail |
| GND (power) | GND |
| VIO | ESP32 **3V3** (not 5 V) |
| GND (logic) | GND |
| STEP | GPIO 18 |
| DIR | GPIO 19 |
| EN | GPIO 5 |
| DIAG | GPIO 23 |
| PDN_UART | GPIO 16 directly, and GPIO 17 through a 1 kOhm resistor (one wire shared) |
| MS1, MS2 | GND (UART address 0) |
| CLK | GND |
| 1A, 1B | Coil Phase A end-wires (e.g., Green / Yellow pair) |
| 2A, 2B | Coil Phase B end-wires (e.g., Blue / White pair) |
| *NC* | Red & Black center-tap wires (isolate with tape/heatshrink) |

*VREF setting:* Set to ~1.2 V for 1.2 A peak (verify board sense resistors).

## 5. Pan Axis: L298N Dual H-Bridge + 6-Wire Tamagawa Seiki NEMA 17
| L298N Pin | Connects to |
|---|---|
| 12V / VM Terminal | Motor 24 V rail (after emergency stop) |
| GND Terminal | Common Ground (GND) |
| 5V Terminal | 5 V rail (Remove onboard 12V jumper if supplying 24V to avoid overheating regulator) |
| ENA, ENB | Tied to 5V via jumpers (always enabled) |
| IN1 | ESP32 GPIO 21 |
| IN2 | ESP32 GPIO 22 |
| IN3 | ESP32 GPIO 25 |
| IN4 | ESP32 GPIO 26 |
| OUT1, OUT2 | Coil Phase A end-wires |
| OUT3, OUT4 | Coil Phase B end-wires |
| *NC* | Red & Black center-tap wires (isolate with tape/heatshrink) |

## 6. Flywheel launcher: relay + two 130 motors in series on 24 V
| Part | Connects to |
|---|---|
| Relay VCC | 5 V rail |
| Relay GND | GND |
| Relay IN | GPIO 27 |
| Relay COM | motor 24 V rail (after the emergency stop) |
| Relay NO | motor 1 (+) |
| Motor 1 (-) | motor 2 (+) |
| Motor 2 (-) | GND |
| 1N4007 / 1N5408 diode | across the two motors in series: cathode (stripe) to the 24 V side (motor 1 +), anode to GND side (motor 2 -) |

## 7. Dart pusher servo
| Servo wire | Connects to |
|---|---|
| Red | 5 V rail |
| Brown / black | GND |
| Orange / yellow | GPIO 13 |

## 8. Dart sensor: Hall module + magnet
Module pins (check the silkscreen): VCC, GND, DO, AO.
| Module pin | Connects to |
|---|---|
| VCC | ESP32 **3V3** (not 5 V, so DO and AO stay at 3.3 V) |
| GND | GND |
| DO | GPIO 32 |
| AO | GPIO 34 |
- **Magnet:** small neodymium disc (3–5 mm diameter, 1–2 mm thick) glued into dart tail or lever.
- **Position:** target sensor sit 3–8 mm from magnet at rest position.
- **Logic:** DO LOW = dart present.

## 9. Optional: tilt 90 degree home (second Hall module)
| Module pin | Connects to |
|---|---|
| VCC | ESP32 3V3 |
| GND | GND |
| DO | GPIO 33 |
| AO | not connected |

## 10. USB
- OV2710 camera: USB to PC / Pi.
- ESP32 DevKit: micro-USB to PC (data link, 921600 baud).

## 11. Stepper motor to belt (reference)
GT2 20T pulley on each motor shaft, GT2 60T pulley on the axis (3:1), 6 mm closed-loop belt.
