# Realtime Cloth Simulator (XPBD) — C++17 + raylib (Single File)

This project is a realtime cloth simulation using **XPBD (Extended Position-Based Dynamics)** with a minimal realtime viewer built on **raylib**.

You edit one file (`cloth_sim.cpp`), compile, and run.

---

## Features

### Physics / Simulation
- Grid cloth mesh (triangulated)
- **XPBD constraints**
  - Structural (stretch): horizontal + vertical neighbors
  - Shear: diagonal neighbors
  - Bend: 2-edge distance constraints (second neighbors)
- Pin constraints (top corners + optional extra pins)
- External forces: gravity + wind field + drag
- Collisions:
  - Infinite ground plane
  - Two spheres
  - Positional projection + friction/bounce applied to velocities
- Optional self-collision:
  - Particle-particle separation using spatial hashing

### Realtime Viewer
- Realtime rendering of triangles (double-sided)
- Simple shading from triangle normals
- Built-in orbital camera controls via raylib
- HUD showing solver settings and toggles

### Export (Optional)
- OBJ sequence export: `obj_out/cloth_00000.obj`, `obj_out/cloth_00001.obj`, ...

---

## Requirements

- C++17 compiler:
  - macOS: `clang++` (Xcode Command Line Tools) or `g++`
  - Linux: `g++` or `clang++`
- raylib
- pkg-config (recommended)

---

## Install Dependencies

### macOS (Homebrew)
```bash
brew install raylib pkg-config
