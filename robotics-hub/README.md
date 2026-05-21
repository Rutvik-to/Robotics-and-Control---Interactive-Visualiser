# Robotics & Control — Siciliano Interactive Hub

Interactive React visualizations for **Siciliano, Sciavicco, Villani & Oriolo — _Robotics: Modelling, Planning and Control_** (Springer, 2009).

Each tab has a collapsible **"What is this?"** panel with plain-language explanation, the textbook equation (with section reference), and a guide to every slider.

## Seventeen interactive tabs

| Tab | Topic | Reference |
|-----|-------|-----------|
| Kinematics (2-link) | Planar arm FK/IK, click-to-IK, manipulability ellipse | Ch. 2-3 |
| DH parameters (3D) | Denavit-Hartenberg chain, 3D rotatable, ZYZ Euler angles | Ch. 2 |
| Trajectory planning | Cubic, quintic, trapezoidal profiles | Ch. 4 |
| Statics & duality | τ = Jᵀ F, force/velocity ellipsoid duality | Ch. 5 |
| DC motor drive | Cascade current+velocity PI, encoder quantization | Ch. 6 |
| PID joint control | Single-joint regulation, PD vs PD+g(q) vs PID | Ch. 8 |
| Arm dynamics | Full B(q)q̈ + C(q,q̇)q̇ + g(q) = τ, three controllers | Ch. 7-8 |
| Robust & adaptive | Sliding-mode + Slotine-Li parameter adaptation | Ch. 8 |
| Impedance control | Virtual spring-damper, wall contact | Ch. 9 |
| **Hybrid force/pos** | Selection-matrix force/position split + admittance | **Ch. 9** |
| **Constrained motion** | Sliding on tilted surface, task-space decomposition | **Ch. 13** |
| Visual servoing (IBVS) | Image-based, interaction matrix, 4-corner tracking | Ch. 10 |
| **PBVS (position-based)** | 3D pose recovery → Cartesian control, with noise toggle | **Ch. 10** |
| Mobile robot | Unicycle: drive to point or follow circle | Ch. 11 |
| **Differential drive** | Left/right wheel control + pure-pursuit figure-8 | **Ch. 11** |
| **Motion planning** | Artificial potential fields + RRT random sampling | **Ch. 12** |
| State space | Pole placement on s-plane, step response | App. B |

All math runs **client-side** in `src/utils/math.js` and each component.

---

## Requirements

- **Node.js 18+** (check with `node -v`)
- npm

## Run it

```bash
cd robotics-hub
npm install
npm run dev          # opens http://localhost:5173
```

## Build a static bundle

```bash
npm run build
npm run preview
```

---

## Project layout

```
robotics-hub/
├── index.html
├── package.json
├── vite.config.js
└── src/
    ├── main.jsx
    ├── App.jsx
    ├── styles.css
    ├── components/                   17 viz cards + 3 reusable
    │   ├── Explainer.jsx             collapsible "What is this?" panel
    │   ├── Plot.jsx                  reusable SVG line plot
    │   ├── Slider.jsx                labeled range input
    │   ├── Kinematics.jsx            Ch. 2-3 — 2-link FK/IK
    │   ├── DHKinematics.jsx          Ch. 2 — 3D DH chain
    │   ├── TrajectoryPlanning.jsx    Ch. 4 — cubic/quintic/trap
    │   ├── Statics.jsx               Ch. 5 — Jᵀ duality
    │   ├── MotorDrive.jsx            Ch. 6 — DC motor cascade
    │   ├── PIDControl.jsx            Ch. 8 — joint PID
    │   ├── ArmDynamics.jsx           Ch. 7-8 — full 2-DOF arm
    │   ├── RobustAdaptive.jsx        Ch. 8 — robust + adaptive
    │   ├── ImpedanceControl.jsx      Ch. 9 — virtual spring
    │   ├── HybridForce.jsx           Ch. 9 — hybrid force/pos + admittance
    │   ├── VisualServoing.jsx        Ch. 10 — IBVS
    │   ├── PBVS.jsx                  Ch. 10 — position-based VS
    │   ├── MobileRobot.jsx           Ch. 11 — unicycle
    │   ├── DiffDrive.jsx             Ch. 11 — differential drive
    │   ├── MotionPlanning.jsx        Ch. 12 — APF + RRT
    │   ├── ConstrainedMotion.jsx     Ch. 13 — task-space decomposition
    │   └── StateSpace.jsx            App. B — pole placement
    └── utils/
        └── math.js                   RK4, matrix ops, eigendecomp
```
