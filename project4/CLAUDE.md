# CSCI502/702 Final Project — Dagu Rover 5
## Context for Claude Code

---

## Student
- **Name:** Mukhamejan Talap
- **Course:** CSCI502/702 — Hardware/Software Co-Design, Spring 2026
- **Role:** Robotics student (all 5 tasks required)
- **Deadline:** April 22/24 class demo, April 24 23:59 report + code to Moodle + GitHub

---

## Hardware

### Robot chassis
- **Dagu Rover 5** tracked chassis
- 4 DC motors (7.2V rated, 2.5A stall)
- Gearbox ratio: 86.8:1
- Quadrature encoders: 1000 state changes per 3 wheel rotations → **333 counts/rev** (x1), **1332 counts/rev** (x4 decoding)
- Encoder signals: 5V logic (A = white, B = yellow, Vcc = red, GND = black)

### Compute
- **Raspberry Pi 3B** — Raspbian 6.12.47, armv7l
- SSH: `pi@172.20.10.3` (from `jan@jan-Legion-Slim-5-16APH8`)

### Power
- 11.1V LiPo battery (TurtleBot type)
- **XL4015E1** buck converter → adjusted to **7.2V** output → L298N motor power input
- RPi powered separately via USB power bank (5V microUSB)
- **Common GND mandatory**: RPi + L298N + XL4015 + battery all share one GND bus

### Motor driver
- **L298N** dual H-bridge
- **Remove ENA/ENB jumpers** (required for PWM speed control)

### Level shifter
- Pololu 4-channel bidirectional logic level shifter (5V ↔ 3.3V)
- Required for encoder A/B signals (5V encoder → 3.3V RPi GPIO)

---

## GPIO Wiring (BCM numbering)

### L298N ↔ RPi

| RPi GPIO (BCM) | L298N Pin | Physical Pin | Function |
|----------------|-----------|--------------|----------|
| GPIO12         | ENA       | 32           | HW PWM0 — Left motor speed |
| GPIO23         | IN1       | 16           | Left motor direction |
| GPIO24         | IN2       | 18           | Left motor direction |
| GPIO27         | IN3       | 13           | Right motor direction |
| GPIO22         | IN4       | 15           | Right motor direction |
| GPIO13         | ENB       | 33           | HW PWM1 — Right motor speed |
| GND            | GND       | 6            | Common ground |

**Motor A (Left):** OUT1/OUT2  
**Motor B (Right):** OUT3/OUT4  

**Direction logic:**
- IN1=1, IN2=0 → forward
- IN1=0, IN2=1 → reverse
- IN1=0, IN2=0 → coast

### Encoder ↔ RPi (via level shifter)

| Connection | Detail |
|------------|--------|
| RPi 5V (Pin 2) | → Encoder Vcc |
| RPi GND (Pin 6) | → Encoder GND |
| Encoder A → level shifter | → GPIO17 |
| Encoder B → level shifter | → GPIO25 |

---

## Build & Run

```bash
# Enable pigpio daemon on RPi
sudo systemctl start pigpiod

# Compile motor control
g++ motor_l298n_pigpio.cpp -o motor_l298n_pigpio -lpigpio -lpthread

# Compile encoder
g++ encoder.cpp -o encoder -lpigpio -lpthread

# Run (must be sudo for pigpio)
sudo ./motor_l298n_pigpio
```

PWM frequency recommendation: **1000 Hz** (`gpioSetPWMfrequency(pin, 1000)`)

---

## Project Files

```
project4/
├── CLAUDE.md                  ← this file
├── final_project.pdf          ← full assignment spec (Tasks 1–5)
├── Rover_5.pdf                ← Dagu Rover 5 chassis datasheet
├── XL4015E1_BG.pdf            ← buck converter datasheet
├── motor_l298n_pigpio.cpp     ← provided template: single motor, pigpio, C++
├── encoder.cpp                ← provided template: quadrature encoder, pigpio, C++
└── front_page_for_video.pptx  ← title slide for demo video
```

---

## Tasks

### Task 1 — Hardware connection (20%) 
- Wire L298N per GPIO table above
- Adjust XL4015 to 7.2V output
- Remove ENA/ENB jumpers
- Verify common GND

### Task 2 — Dual motor PWM control
- Extend `motor_l298n_pigpio.cpp` to control both left + right motors
- Test: 30% fwd → 60% fwd → stop → 30% reverse → stop
- Use HW PWM on GPIO12 (ENA) and GPIO13 (ENB)

### Task 3 — Virtual joystick from phone (WebSocket)
- RPi runs C++ Crow web server
- Phone browser → nipple.js joystick → WebSocket → RPi
- Differential drive mixer: `left = y + x`, `right = y - x`, clamp to [-1, 1]
- If no shared WiFi: configure RPi as hotspot via `hostapd`
- Emergency stop on disconnect (watchdog timeout)

### Task 4 — PD yaw control via IMU (Robotics required)
- IMU mounted at chassis center
- Read yaw from quaternion: `ψ = atan2(2(wz + xy), 1 − 2(y² + z²))`
- Wrapped yaw error: `eψ = atan2(sin(ψ_ref − ψ), cos(ψ_ref − ψ))`
- PD controller: `ω_cmd = Kp·eψ − Kd·ψ̇`
- Wheel commands: `vR = (L/2)·ω_cmd`, `vL = -(L/2)·ω_cmd`
- Stop condition: `|eψ| < ε_ψ` AND `|ψ̇| < ε_ψ̇`
- Run as separate thread
- Rover 5 dimensions: wheelbase L ≈ 0.17m, wheel radius r ≈ 0.03m

### Task 5 — Encoder reading (Robotics required)
- Wire encoders through level shifter (5V → 3.3V)
- Encoder resolution: 333 counts/rev → x4 decoding = **1332 counts/rev**
- Modify `encoder.cpp` template
- Demonstrate speed readout at different PWM duty cycles
- Wheel angle: `θ = (2π / counts_per_rev) * count`

---

## Suggested Implementation Order
1. Task 1 (hardware) → 2 (dual motor) → 5 (encoders) → 4 (PD control) → 3 (joystick)

---

## Notes
- pigpio requires `sudo` to run
- Always `gpioTerminate()` on exit to release GPIO
- Software PWM causes motor whine + jitter — always use HW PWM (GPIO12, GPIO13)
- L298N has significant voltage drop under load (bipolar technology)
- All 4 GNDs must be tied together or motors won't spin
