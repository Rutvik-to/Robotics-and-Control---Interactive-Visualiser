import React, { useState } from 'react';
import PIDControl from './components/PIDControl.jsx';
import Kinematics from './components/Kinematics.jsx';
import ArmDynamics from './components/ArmDynamics.jsx';
import StateSpace from './components/StateSpace.jsx';
import DHKinematics from './components/DHKinematics.jsx';
import TrajectoryPlanning from './components/TrajectoryPlanning.jsx';
import Statics from './components/Statics.jsx';
import MotorDrive from './components/MotorDrive.jsx';
import RobustAdaptive from './components/RobustAdaptive.jsx';
import ImpedanceControl from './components/ImpedanceControl.jsx';
import VisualServoing from './components/VisualServoing.jsx';
import MobileRobot from './components/MobileRobot.jsx';
import HybridForce from './components/HybridForce.jsx';
import PBVS from './components/PBVS.jsx';
import DiffDrive from './components/DiffDrive.jsx';
import MotionPlanning from './components/MotionPlanning.jsx';
import ConstrainedMotion from './components/ConstrainedMotion.jsx';

const TABS = [
  { id: 'kin', label: 'Kinematics (2-link)', ch: 'Ch. 2', intro: 'Two-link planar arm. Forward maps angles to position; inverse maps position back to angles. Click in IK mode to set a target.' },
  { id: 'dh', label: 'DH parameters (3D)', ch: 'Ch. 2', intro: 'Universal recipe for describing any robot arm: 4 numbers per joint (a, α, d, θ). Drag yaw/pitch to rotate the 3D view.' },
  { id: 'traj', label: 'Trajectory planning', ch: 'Ch. 4', intro: 'Smooth motions through time: cubic, quintic, trapezoidal. Compare-all to see why quintic is the smoothest.' },
  { id: 'stat', label: 'Statics & duality', ch: 'Ch. 5', intro: 'τ = Jᵀ F — how end-effector forces become joint torques. Click on the workspace to apply a force.' },
  { id: 'motor', label: 'DC motor drive', ch: 'Ch. 6', intro: 'What\'s inside a robot joint: motor, encoder, cascade current+velocity PI loops.' },
  { id: 'pid', label: 'PID joint control', ch: 'Ch. 8', intro: 'Single joint regulation. Tune Kₚ, Kd, Ki and watch overshoot and steady-state error react in real time.' },
  { id: 'dyn', label: 'Arm dynamics', ch: 'Ch. 7-8', intro: 'Full 2-DOF nonlinear arm. Compare PD vs PD+gravity vs computed-torque.' },
  { id: 'robadapt', label: 'Robust & adaptive', ch: 'Ch. 8', intro: 'When the controller doesn\'t know the exact parameters. Robust uses worst-case effort; adaptive learns the parameters online.' },
  { id: 'imp', label: 'Impedance control', ch: 'Ch. 9', intro: 'Make the end-effector behave like a virtual spring-damper. Safe for contact — push and it gives.' },
  { id: 'hyb', label: 'Hybrid force/pos', ch: 'Ch. 9', intro: 'Control force in one direction and position in another — like wiping a table. Also includes admittance control.' },
  { id: 'cnstr', label: 'Constrained motion', ch: 'Ch. 13', intro: 'Robot sliding on a tilted surface. Split into tangent (motion-controlled) and normal (force-controlled) directions.' },
  { id: 'vs', label: 'Visual servoing (IBVS)', ch: 'Ch. 10', intro: 'Camera-driven control: align image features by moving the camera. Image-based formulation.' },
  { id: 'pbvs', label: 'PBVS (position-based)', ch: 'Ch. 10', intro: 'Position-based visual servoing: estimate 3D pose from features, then drive in Cartesian space. Cleaner paths but needs calibration.' },
  { id: 'mobile', label: 'Mobile robot', ch: 'Ch. 11', intro: 'Unicycle (Roomba-style) robot. Drive to a point or follow a circle.' },
  { id: 'dd', label: 'Differential drive', ch: 'Ch. 11', intro: 'Two-wheel robot. Drive each wheel separately, or track a figure-8 with pure pursuit.' },
  { id: 'plan', label: 'Motion planning', ch: 'Ch. 12', intro: 'Find a path through obstacles: artificial potential fields (gradient descent) or RRT (random sampling tree).' },
  { id: 'ss', label: 'State space', ch: 'App. B', intro: 'Where do the poles live? Drag ωₙ and ζ, watch eigenvalues move on the s-plane.' }
];

export default function App() {
  const [active, setActive] = useState('kin');
  const tab = TABS.find(t => t.id === active);

  return (
    <div className="app">
      <header className="header">
        <h1>Robotics & control — interactive companion</h1>
        <p className="subtitle">
          Visualizations grounded in Siciliano, Sciavicco, Villani & Oriolo,
          <em> Robotics: Modelling, Planning and Control</em> (Springer, 2009).
        </p>
      </header>

      <nav className="tabs" role="tablist">
        {TABS.map(t => (
          <button key={t.id}
            className={`tab ${active === t.id ? 'active' : ''}`}
            onClick={() => setActive(t.id)}
            role="tab"
            aria-selected={active === t.id}>
            <span style={{ fontFamily: 'var(--mono)', fontSize: 10, color: 'var(--text-faint)', marginRight: 6 }}>{t.ch}</span>
            {t.label}
          </button>
        ))}
      </nav>

      <p className="intro">{tab.intro}</p>

      {active === 'kin' && <Kinematics />}
      {active === 'dh' && <DHKinematics />}
      {active === 'traj' && <TrajectoryPlanning />}
      {active === 'stat' && <Statics />}
      {active === 'motor' && <MotorDrive />}
      {active === 'pid' && <PIDControl />}
      {active === 'dyn' && <ArmDynamics />}
      {active === 'robadapt' && <RobustAdaptive />}
      {active === 'imp' && <ImpedanceControl />}
      {active === 'hyb' && <HybridForce />}
      {active === 'cnstr' && <ConstrainedMotion />}
      {active === 'vs' && <VisualServoing />}
      {active === 'pbvs' && <PBVS />}
      {active === 'mobile' && <MobileRobot />}
      {active === 'dd' && <DiffDrive />}
      {active === 'plan' && <MotionPlanning />}
      {active === 'ss' && <StateSpace />}
    </div>
  );
}
