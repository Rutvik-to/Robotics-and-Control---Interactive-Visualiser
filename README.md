# Robotics-and-Control---Interactive-Visualiser

17 browser-based, slider-driven visualizations grounded in
Siciliano, Sciavicco, Villani & Oriolo — Robotics: Modelling, Planning and Control (Springer, 2009).

No login. No cloud. Runs entirely in your browser.
Built for students, hobbyists, and anyone who learns better by seeing the math move.

What's inside
#TabTopicChapter1Kinematics (2-link)Planar FK/IK, click-to-IK, manipulability ellipseCh. 2–32DH Parameters (3D)Denavit-Hartenberg chain, 3D rotatable, ZYZ Euler anglesCh. 23Trajectory PlanningCubic, quintic, trapezoidal motion profilesCh. 44Statics & Dualityτ = Jᵀ F, force/velocity ellipsoid dualityCh. 55DC Motor DriveCascade current + velocity PI, encoder quantizationCh. 66PID Joint ControlPD vs PD+g(q) vs PID, steady-state and overshoot liveCh. 87Arm DynamicsFull B(q)q̈ + C(q,q̇)q̇ + g(q) = τ, three controllersCh. 7–88Robust & AdaptiveSliding-mode + Slotine-Li parameter adaptationCh. 89Impedance ControlVirtual spring-damper, wall contactCh. 910Hybrid Force/PosSelection-matrix force/position split + admittanceCh. 911Constrained MotionSliding on tilted surface, task-space decompositionCh. 1312Visual Servoing (IBVS)Image-based, interaction matrix, 4-corner trackingCh. 1013PBVS3D pose recovery → Cartesian control, noise toggleCh. 1014Mobile RobotUnicycle: drive to point or follow circleCh. 1115Differential DriveLeft/right wheel control + pure-pursuit figure-8Ch. 1116Motion PlanningArtificial potential fields + RRT random samplingCh. 1217State SpacePole placement on s-plane, live step responseApp. B
Every tab has a collapsible "What is this?" panel with plain-language explanation, the textbook equation, and a guide to each slider.

Getting started
Requirements: Node.js 18+ and npm
bashgit clone https://github.com/YOUR_USERNAME/robotics-hub.git
cd robotics-hub
npm install
npm run dev
Opens at http://localhost:5173 — that's it.
Build a static bundle (to host on GitHub Pages, Vercel, etc.):
bashnpm run build
npm run preview

Project structure
robotics-hub/
├── index.html
├── package.json
├── vite.config.js
└── src/
    ├── App.jsx               # Tab routing
    ├── styles.css            # Global theme
    ├── components/           # 17 visualization components + 3 shared
    │   ├── Explainer.jsx     # Collapsible "What is this?" panel
    │   ├── Plot.jsx          # Reusable SVG line plot
    │   ├── Slider.jsx        # Labeled range input
    │   └── ...               # One file per visualization
    └── utils/
        └── math.js           # RK4, matrix ops, eigendecomp — all client-side
