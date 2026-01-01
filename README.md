```markdown
# 🤖✨ Zumo Shield Smart Bots (Arduino) — *Zumo 2025* vs *Zumo_Drive_hit_detect*
```
```
      ____                      ____  __      _      ____
     /_  /_  __  __ _ __ ___   / __ \/ /___  (_)__  / __ )____  __
      / / / / / /  ' // _  /  / /_/ / / __ \/ / _ \/ __  / __ \/ /
     / /_/ /_/ / / / // / /  / ____/ / /_/ / /  __/ /_/ / /_/ / /
    /___/\__,_/_/ /_//_/ /_/ /_/   /_/\____/_/\___/_____/\____/_/

                 Z U M O   S H I E L D   ( U N O / L E O )
```

# Overview

This repo contains **two different autonomous control programs** for the **Pololu Zumo Shield for Arduino**.

They share the same spirit (small robot, big attitude), but they target **different priorities**:

- **`Zumo 2025.ino`** → *Arena survival brain*  
  Edge avoidance + wandering + optional IMU heading hold + bump reaction.

- **`Zumo_Drive_hit_detect.ino`** → *Collision charger*  
  Uses accelerometer “contact” detection to trigger full-speed charge + randomized turn patterns.

---

## ✅ Hardware Target

- **Pololu Zumo Shield for Arduino** (Arduino Uno / Leonardo)
- Optional but recommended:
  - **Reflectance sensor array** (for edge detection / border detection)
  - **IMU** (accelerometer + magnetometer; gyro on some revisions)

> ⚠️ Not the same as the **Zumo 32U4 robot**. That uses different libraries/headers.

---

# Quick Start

## 1) Install the Pololu library

Install the **ZumoShield** library (Pololu):

- Arduino IDE → **Sketch → Include Library → Manage Libraries…**
- Search **ZumoShield**
- Install **ZumoShield by Pololu**

Or install from ZIP from the Pololu repo if you prefer.

## 2) Open the sketch you want

- `Zumo 2025.ino`  ✅ recommended for ring/table survival  
- `Zumo_Drive_hit_detect.ino` ✅ fun/aggressive collision behavior

## 3) Select board + port and upload

- Tools → Board → (Uno / Leonardo)
- Tools → Port → (your COM port)
- Upload

---

# Programs

---

## 🧠 `Zumo 2025.ino` — Smart Sumo / Edge Avoider (NO line following)

### What it’s built to do
This sketch treats **not falling off the world** as the top priority.

**Core features**
- **Edge detection** using reflectance array  
  If it sees a bright border/edge → **backup + turn**.
- **Wander behavior**  
  Drives forward and explores, periodically changing direction.
- **IMU bump reaction** (accelerometer “jerk”)  
  If hit → brief “attack burst” forward.
- **IMU heading hold** (magnetometer compass) *(if IMU is present)*  
  Keeps the robot driving straighter and more repeatably.
- **Button UI**
  - Short press: cycle modes
  - Long press: STOP / RUN
- **Buzzer feedback** for mode changes and events.

### When to use it
- Black sumo ring with white border
- Table edge avoidance (bright tape edge)
- “Roam but don’t fall” robots

### Why it wins more often (in ring-style environments)
Because it can **survive longer**:
- It actively avoids edges
- It does not rely on random luck to stay on the surface

---

## ⚡ `Zumo_Drive_hit_detect.ino` — Accelerometer Contact Charger (your provided sketch)

### What it’s built to do
This sketch is a **contact-driven brawler**.

**Core features**
- Reads accelerometer X/Y and detects **sudden force/jerk**
- On “contact”, it:
  - plays a **charge melody**
  - switches to **FULL_SPEED**
  - runs one of **20 random turn patterns**
- Motor trim constants help it drive straighter.

### Strengths
- Very punchy, very aggressive
- Great for:
  - “ram anything you hit” behavior
  - small obstacle courses where you *want* chaotic unpredictability

### Weaknesses (important)
- **No edge/border detection**, so on a sumo ring it can:
  - charge hard
  - and then drive right off the edge
- Accelerometer “contact” can false-trigger from:
  - bumpy surfaces
  - acceleration from turns
  - vibration

---

# Side-by-Side Comparison

| Feature | `Zumo 2025.ino` | `Zumo_Drive_hit_detect.ino` |
|---|---:|---:|
| Reflectance sensors | ✅ used for edge/border detection | ❌ not used |
| Ring survival priority | ✅ **high** | ❌ **low** |
| Bump/impact reaction | ✅ yes (jerk → short burst) | ✅ yes (jerk → full charge) |
| Magnetometer/compass | ✅ heading hold (if IMU present) | ❌ not used |
| Turn strategy | simple, reliable escape turns | many random time-based turn patterns |
| Best use | sumo ring / table / “don’t fall” | chaotic charger / collision toy |
| Biggest risk | tuning thresholds | driving off the ring |

---

# Controls (Zumo 2025)

- **Short press**: cycles modes  
  `AUTO → RING_ONLY → WANDER_ONLY → AUTO`
- **Long press**: toggles  
  `STOP ↔ RUN`

---

# Calibration Notes (Zumo 2025)

### Reflectance calibration
During startup calibration, try to let the robot “see”:
- the darker surface
- the bright border/edge

This makes edge detection far more reliable.

### Compass calibration (if IMU exists)
The robot will spin to capture min/max magnetometer values for heading.

---

# Tuning Guide

## `Zumo 2025.ino` key knobs

### `EDGE_BRIGHT_THRESHOLD`
- If it escapes too often → **increase**
- If it misses the edge and falls off → **decrease**

### `BUMP_JERK_THRESHOLD`
- False bumps → **increase**
- Never detects bumps → **decrease**

### `BASE_SPEED`
- Too wild/slippy → **lower**
- Too slow to be useful → **raise**

---

## `Zumo_Drive_hit_detect.ino` key knobs

### `XY_ACCELERATION_THRESHOLD`
- More sensitive contact detection → **lower**
- Fewer false triggers → **higher**

### Turn durations
- If it barely turns → increase durations
- If it over-rotates → reduce durations

---

# File Layout (expected)

```
.
├── Zumo 2025.ino
├── Zumo_Drive_hit_detect.ino
├── LICENSE
└── README.md
```

---

# License

MIT License (see `LICENSE`)

---

# Credits

- Pololu Zumo Shield Arduino library:
  - https://github.com/pololu/zumo-shield-arduino-library

- This repo includes:
  - `Zumo 2025.ino` (edge-avoid / wander / IMU assist)
  - `Zumo_Drive_hit_detect.ino` (accelerometer contact charger with turn patterns)

---

# Roadmap Ideas (optional fun)

- Merge both into a **hybrid “Ring Safe Charger”**:
  - keep edge detection as #1 priority
  - keep charge melody + full-speed burst on confirmed contact
  - reduce to the 5 best turn patterns
  - add optional “search spin” when no opponent is hit for X seconds
```
