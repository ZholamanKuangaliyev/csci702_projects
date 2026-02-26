# Task 3 — UML Class Diagram
## IMU Data Processing & Visualization Program
### CSCI 702 — Project 2

---

## 1. Program Overview

The system is a two-node distributed application that streams real-time IMU
(Inertial Measurement Unit) sensor data from a **Raspberry Pi** to a **Host PC**,
where the orientation is rendered as a rotating 3D cube using OpenGL.

**Data flow:**

```
[BerryIMU/MPU6050] → I²C → [RPi: pi_sender.cpp] → UDP (port 8080) → [Host: pc_receiver.cpp] → OpenGL Cube
```

The Raspberry Pi reads raw accelerometer and gyroscope data over I²C, feeds
the measurements into a Madgwick AHRS filter to estimate 3D orientation as a
unit quaternion, and transmits that quaternion as a 4×double UDP packet at
~100 Hz.  The host receives the packet, converts the quaternion to an OpenGL
rotation matrix, and re-renders a 3D cube every frame.

---

## 2. UML Class Diagram (ASCII)

```
 ╔══════════════════════════════════╗   ╔══════════════════════════════════════╗
 ║  Raspberry Pi — pi_sender.cpp    ║   ║  Host PC — pc_receiver.cpp           ║
 ╚══════════════════════════════════╝   ╚══════════════════════════════════════╝

 ┌──────────────────────┐               ┌──────────────────────────┐
 │      I2CDevice       │               │    QuaternionUtils       │
 │ ──────────────────── │               │     <<utility>>          │
 │ #file: int           │               │ ──────────────────────── │
 │ #bus: int            │               │ +normalize(q): Quat      │
 │ #deviceAddress: int  │               │ +quatToMat4(q): float[16]│
 │ ──────────────────── │               │ +mul(a,b): Quat          │
 │ +I2CDevice(bus,addr) │               │ +fromAxisAngle(...): Quat│
 │ +~I2CDevice()        │               └──────────────────────────┘
 │ +writeRegister8(..)  │
 │ +readRegister16(..)  │               ┌──────────────────────────┐
 └──────────┬───────────┘               │      UDPReceiver         │
            │ extends (inheritance)     │     <<service>>          │
            ▼                           │ ──────────────────────── │
 ┌──────────────────────┐               │ -sockfd: int             │
 │      IMUSensor       │               │ ──────────────────────── │
 │ ──────────────────── │               │ +UDPReceiver(port)       │
 │ -accelScale: float   │               │ +initUDP(): void         │
 │ -gyroScale: float    │               │ +receiveQuaternion():Quat│
 │ -filter: Madgwick... │               └──────────────────────────┘
 │ ──────────────────── │
 │ +IMUSensor(bus,addr) │               ┌──────────────────────────┐
 │ +configureIMU()      │               │    OpenGLRenderer        │
 │ +readPhysicalValues()│               │ ──────────────────────── │
 │ +updateAndGetOrient()│               │ -g_q: Quat               │
 └──────────┬───────────┘               │ -g_width: int            │
            │                           │ -g_height: int           │
            │ has-a (composition)       │ -g_rot_step: float       │
            ▼                           │ ──────────────────────── │
 ┌──────────────────────┐               │ +display(): void         │
 │   MadgwickFilter     │               │ +reshape(w,h): void      │
 │ ──────────────────── │               │ +idle(): void            │
 │ -beta: float         │               │ +keyboard(..): void      │
 │ -q0,q1,q2,q3: float  │               │ -drawAxes(len): void     │
 │ ──────────────────── │               │ -drawCubeSolid(s): void  │
 │ +MadgwickFilter()    │               │ -drawCubeWire(s): void   │
 │ +update(gx,gy,gz,    │               └──────────────────────────┘
 │   ax,ay,az,dt): void │
 │ +getQuaternion():Quat│
 └──────────────────────┘

 ┌──────────────────────┐
 │      UDPSender       │
 │     <<service>>      │           ═══ UDP / IP — port 8080 ═══►
 │ ──────────────────── │
 │ -sockfd: int         │
 │ -servaddr:sockaddr_in│
 │ ──────────────────── │
 │ +UDPSender(ip, port) │
 │ +sendQuaternion(q)   │
 │ +close()             │
 └──────────────────────┘

                ┌─────────────────┐
                │   Quat          │
                │  <<struct>>     │
                │ ─────────────── │
                │ +w : double     │
                │ +x : double     │
                │ +y : double     │
                │ +z : double     │
                └─────────────────┘
                   (shared between
                    both sides)
```

**Relationship legend:**

| Symbol | Meaning |
|--------|---------|
| `──▷` (hollow arrow) | Inheritance / Generalization (is-a) |
| `──*` (filled diamond) | Composition (has-a, owns lifetime) |
| `──◇` (hollow diamond) | Aggregation (uses, does not own) |
| `- - →` (dashed arrow) | Dependency (uses / returns) |
| `══►` (thick arrow) | Network communication |

---

## 3. Class Descriptions

### `Quat` (struct — shared)
A plain data structure representing a unit quaternion **q = w + xi + yj + zk**.
Quaternions compactly encode 3D orientation without gimbal lock.
Used as the universal data token passed between all components.

---

### `I2CDevice` (base class — RPi)
A hardware abstraction layer (HAL) that encapsulates the Linux I²C kernel
interface (`/dev/i2c-N`).  It opens the bus file descriptor, assigns the slave
address via `ioctl`, and exposes two register-access primitives:
- `writeRegister8` — writes a single byte to a named register.
- `readRegister16` — reads two consecutive bytes and combines them
  (MSB-first) into a signed 16-bit integer, the native word size of most
  MEMS sensors.

---

### `IMUSensor` (derived from `I2CDevice` — RPi)
Specializes `I2CDevice` for the BerryIMU / MPU-6050 class of sensors.
Stores scaling factors (±2 g for accelerometer, ±250 °/s for gyroscope)
and owns a `MadgwickFilter` instance.  Key method:
- `readPhysicalValues()` — reads the six raw 16-bit registers (Ax,Ay,Az,
  Gx,Gy,Gz), multiplies by the respective scale factors, and returns a
  `vector<float>` of physical-unit values (g and °/s).
- `updateAndGetOrientation(dt)` — calls `readPhysicalValues`, feeds the
  result into the filter, and returns the updated `Quat`.

---

### `MadgwickFilter` (RPi)
Implements Sebastian Madgwick's gradient-descent AHRS algorithm.
Maintains an internal quaternion estimate (q0–q3) and a tunable gain
parameter **β** that balances gyroscope integration against accelerometer
correction.  `update()` consumes one timestep of sensor data and
advances the quaternion estimate.  `getQuaternion()` exposes the result.

---

### `UDPSender` (RPi — modelled from `main`)
Wraps the POSIX UDP socket that ships quaternion packets to the host.
Creates a `SOCK_DGRAM` socket, fills a `sockaddr_in` with the host IP
and port 8080, and calls `sendto` with a 32-byte payload `{w, x, y, z}`
at ~100 Hz (10 ms sleep between sends).

---

### `UDPReceiver` (Host — modelled from `initUDP / idle`)
Wraps the POSIX UDP socket on the host side.  Binds to `INADDR_ANY:8080`,
sets the socket to non-blocking mode (`O_NONBLOCK`), and in `receiveQuaternion`
calls `recvfrom` to obtain the 32-byte quaternion payload sent by the RPi.

---

### `QuaternionUtils` (Host — utility)
A collection of stateless quaternion math operations used by the renderer:
- `normalize` — scales q to unit length to prevent drift.
- `quatToMat4` — converts q to a column-major 4×4 OpenGL rotation matrix
  using the standard quaternion-to-rotation-matrix formulae.
- `mul` — Hamilton product for composing two rotations.
- `fromAxisAngle` — constructs q from an axis vector and angle (used by
  the interactive keyboard mode in `cube_quat.cpp`).

---

### `OpenGLRenderer` (Host — modelled from display/idle/keyboard callbacks)
Owns the global GLUT state and the current orientation quaternion `g_q`.
- `display()` — clears the framebuffer, sets up a perspective camera with
  `gluLookAt`, draws fixed world-axes, applies `quatToMat4(g_q)` to the
  modelview matrix, and renders a solid+wireframe cube with body-fixed axes.
- `reshape()` — maintains correct aspect ratio on window resize.
- `idle()` — called by GLUT each frame; polls `UDPReceiver` for a fresh
  quaternion and triggers a redraw.
- `keyboard()` — handles ESC/q for clean exit (and in the standalone
  `cube_quat.cpp` variant also handles w/a/s/d/e/c for manual rotation).

---

## 4. Key Design Relationships

| Relationship | Type | Description |
|---|---|---|
| `IMUSensor` → `I2CDevice` | Inheritance | IMUSensor is-a I2CDevice; inherits file I/O |
| `IMUSensor` → `MadgwickFilter` | Composition | Filter is created and owned by IMUSensor |
| `IMUSensor` → `Quat` | Dependency | Returns Quat from `updateAndGetOrientation` |
| `MadgwickFilter` → `Quat` | Dependency | Returns Quat from `getQuaternion` |
| `UDPSender` → `Quat` | Dependency | Serializes and transmits Quat |
| `UDPReceiver` → `Quat` | Dependency | Deserializes and returns Quat |
| `OpenGLRenderer` → `Quat` | Aggregation | Holds current orientation as `g_q` |
| `OpenGLRenderer` → `QuaternionUtils` | Dependency | Calls normalize, quatToMat4 |
| `OpenGLRenderer` → `UDPReceiver` | Dependency | Polls receiver in `idle()` |
| `UDPSender` → `UDPReceiver` | Network | UDP datagrams over IP, port 8080 |

---

## 5. References

- S. O. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays," April 2010.
- BerryIMU / OzzMaker I²C interfacing guide: https://ozzmaker.com/berryimu/
- UML@Classroom textbook — Chapters 1, 2, 4, 8
- Visual Paradigm UML Class Diagram Tutorial: https://www.visual-paradigm.com/guide/uml-unified-modeling-language/uml-class-diagram-tutorial/
