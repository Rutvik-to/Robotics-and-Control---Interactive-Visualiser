import React, { useState, useMemo, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate, inv2, matVec2 } from '../utils/math.js';

/**
 * 2-link planar arm dynamics (Siciliano §7.3, eq. 7.82).
 *
 *   B(q) q̈ + C(q,q̇) q̇ + g(q) = τ
 *
 * where (point masses at link tips, simplified — same structural form):
 *   B11 = m1 lc1² + I1 + m2 (l1² + lc2² + 2 l1 lc2 cos q2) + I2
 *   B12 = m2 (lc2² + l1 lc2 cos q2) + I2
 *   B22 = m2 lc2² + I2
 *   C   = m2 l1 lc2 sin(q2) * [-q̇2, -(q̇1+q̇2); q̇1, 0]
 *   g1  = (m1 lc1 + m2 l1) g cos(q1) + m2 lc2 g cos(q1+q2)
 *   g2  = m2 lc2 g cos(q1+q2)
 *
 * Controllers (Siciliano §8.5):
 *   - PD: τ = Kp e + Kd ė
 *   - PD + g: τ = Kp e + Kd ė + g(q)    (model-based gravity compensation)
 *   - Computed torque: τ = B(q)(q̈d + Kd ė + Kp e) + C(q,q̇) q̇ + g(q)
 */

function dynamics(q, qd, params) {
  const { m1, m2, l1, lc1, lc2, I1, I2, g } = params;
  const c2 = Math.cos(q[1]);
  const s2 = Math.sin(q[1]);

  const B11 = m1 * lc1 * lc1 + I1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * c2) + I2;
  const B12 = m2 * (lc2 * lc2 + l1 * lc2 * c2) + I2;
  const B22 = m2 * lc2 * lc2 + I2;
  const B = [[B11, B12], [B12, B22]];

  // Coriolis/centrifugal vector C(q,q̇) q̇
  const h = m2 * l1 * lc2 * s2;
  const C_qd = [
    -h * qd[1] * qd[1] - 2 * h * qd[0] * qd[1],
    h * qd[0] * qd[0]
  ];

  // Gravity
  const c1 = Math.cos(q[0]);
  const c12 = Math.cos(q[0] + q[1]);
  const grav = [
    (m1 * lc1 + m2 * l1) * g * c1 + m2 * lc2 * g * c12,
    m2 * lc2 * g * c12
  ];

  return { B, C_qd, grav };
}

export default function ArmDynamics() {
  const [m1, setM1] = useState(1.0);
  const [m2, setM2] = useState(0.8);
  const [Kp, setKp] = useState(60);
  const [Kd, setKd] = useState(18);
  const [q1d, setQ1d] = useState(0.8);
  const [q2d, setQ2d] = useState(-1.2);
  const [controller, setController] = useState('ct'); // 'pd' | 'pdg' | 'ct'
  const [running, setRunning] = useState(false);
  const [time, setTime] = useState(0);
  const animRef = useRef(null);

  const params = useMemo(() => ({
    m1, m2,
    l1: 1.0, l2: 0.8,
    lc1: 0.5, lc2: 0.4,
    I1: 0.1, I2: 0.05,
    g: 9.81
  }), [m1, m2]);

  // Simulation (re-runs whenever params change)
  const sim = useMemo(() => {
    const tFinal = 4.0, dt = 0.005;
    const qdes = [q1d, q2d];

    const f = (t, x) => {
      const q = [x[0], x[1]];
      const qdot = [x[2], x[3]];
      const e = [qdes[0] - q[0], qdes[1] - q[1]];
      const ed = [-qdot[0], -qdot[1]];

      const { B, C_qd, grav } = dynamics(q, qdot, params);

      let tau;
      if (controller === 'pd') {
        tau = [Kp * e[0] + Kd * ed[0], Kp * e[1] + Kd * ed[1]];
      } else if (controller === 'pdg') {
        tau = [Kp * e[0] + Kd * ed[0] + grav[0], Kp * e[1] + Kd * ed[1] + grav[1]];
      } else {
        // Computed torque: τ = B(q)(Kp e + Kd ė) + C q̇ + g
        const y = [Kp * e[0] + Kd * ed[0], Kp * e[1] + Kd * ed[1]];
        const By = [B[0][0] * y[0] + B[0][1] * y[1], B[1][0] * y[0] + B[1][1] * y[1]];
        tau = [By[0] + C_qd[0] + grav[0], By[1] + C_qd[1] + grav[1]];
      }

      // q̈ = B⁻¹ (τ − C q̇ − g)
      const rhs = [tau[0] - C_qd[0] - grav[0], tau[1] - C_qd[1] - grav[1]];
      const Binv = inv2(B);
      const qddot = matVec2(Binv, rhs);

      return [qdot[0], qdot[1], qddot[0], qddot[1]];
    };

    return simulate(f, [0, 0, 0, 0], 0, tFinal, dt);
  }, [Kp, Kd, q1d, q2d, controller, params]);

  // Animate playback
  useEffect(() => {
    if (!running) return;
    let startWall = performance.now();
    let startT = time;
    const tick = () => {
      const elapsed = (performance.now() - startWall) / 1000;
      const newT = startT + elapsed;
      if (newT >= 4.0) {
        setTime(4.0);
        setRunning(false);
        return;
      }
      setTime(newT);
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  // Reset time when sim params change
  useEffect(() => { setTime(0); }, [Kp, Kd, q1d, q2d, controller, m1, m2]);

  // Snapshot at current time
  const idx = Math.min(sim.length - 1, Math.floor(time / 0.005));
  const snap = sim[idx];
  const q = [snap.x[0], snap.x[1]];
  const { l1, l2 } = params;
  const x1 = l1 * Math.cos(q[0]), y1 = l1 * Math.sin(q[0]);
  const xe = x1 + l2 * Math.cos(q[0] + q[1]);
  const ye = y1 + l2 * Math.sin(q[0] + q[1]);
  // Target pose
  const x1d = l1 * Math.cos(q1d), y1d = l1 * Math.sin(q1d);
  const xed = x1d + l2 * Math.cos(q1d + q2d);
  const yed = y1d + l2 * Math.sin(q1d + q2d);

  const W = 320, H = 280;
  const scale = 70;
  const ox = W / 2, oy = H / 2;
  const toS = (x, y) => [ox + x * scale, oy - y * scale];
  const [bx, by] = toS(0, 0);
  const [jx, jy] = toS(x1, y1);
  const [ex, ey] = toS(xe, ye);
  const [jdx, jdy] = toS(x1d, y1d);
  const [edx, edy] = toS(xed, yed);

  // Plot series
  const q1Series = { name: 'q₁', color: 'var(--plot-1)', width: 1.8, points: sim.map(s => [s.t, s.x[0]]) };
  const q2Series = { name: 'q₂', color: 'var(--plot-2)', width: 1.8, points: sim.map(s => [s.t, s.x[1]]) };
  const q1dSeries = { name: 'q₁d', color: 'var(--plot-1)', width: 1, dash: '5 4', points: sim.map(s => [s.t, q1d]) };
  const q2dSeries = { name: 'q₂d', color: 'var(--plot-2)', width: 1, dash: '5 4', points: sim.map(s => [s.t, q2d]) };

  // Tracking errors at final time
  const fin = sim[sim.length - 1].x;
  const e1 = Math.abs(q1d - fin[0]);
  const e2 = Math.abs(q2d - fin[1]);

  const ctrlName = { pd: 'PD only', pdg: 'PD + gravity', ct: 'Computed torque' }[controller];

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>2-link arm — {ctrlName}</h3>
        <p className="eq">B(q) q̈ + C(q,q̇) q̇ + g(q) = τ</p>

        <Explainer
          title="Why is arm dynamics hard?"
          plain="Unlike a single joint, a two-link arm has 'cross-talk': moving joint 1 throws joint 2 off, and vice versa. The mass matrix B(q) describes how heavy each joint feels, but it CHANGES with the arm's pose. The Coriolis term C(q,q̇) captures the 'whip' effect — when you swing an arm, the forearm gets flung. PD control alone struggles because it doesn't know about these effects. Computed torque is like a chess player who plans ahead: it predicts B, C, g exactly and cancels them, leaving a clean linear system."
          equation="B(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ"
          equationNote="Siciliano eq. 7.82. Computed-torque control: τ = B(q)(q̈d + Kp·e + Kd·ė) + C(q,q̇)q̇ + g(q), eq. 8.57. The resulting closed-loop is two decoupled second-order systems."
          knobs={[
            { name: 'PD / PD+g / Computed torque', what: 'three controllers of increasing model awareness' },
            { name: 'Kₚ, Kd', what: 'gains in the outer error loop; matter most for computed torque' },
            { name: 'q₁d, q₂d', what: 'where you want the arm to end up' },
            { name: 'm₁, m₂', what: 'link masses — heavier links mean stronger coupling and gravity' }
          ]} />

        <div style={{ display: 'grid', gridTemplateColumns: 'auto 1fr', gap: 16, alignItems: 'start' }}>
          {/* Arm pose */}
          <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}
            style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
            <line x1={0} y1={oy} x2={W} y2={oy} stroke="var(--grid)" strokeWidth="0.5" />
            <line x1={ox} y1={0} x2={ox} y2={H} stroke="var(--grid)" strokeWidth="0.5" />
            {/* Desired pose (ghost) */}
            <line x1={bx} y1={by} x2={jdx} y2={jdy} stroke="var(--text-faint)" strokeWidth="3" strokeLinecap="round" strokeOpacity="0.35" />
            <line x1={jdx} y1={jdy} x2={edx} y2={edy} stroke="var(--text-faint)" strokeWidth="3" strokeLinecap="round" strokeOpacity="0.35" />
            <circle cx={edx} cy={edy} r="5" fill="none" stroke="var(--warn)" strokeWidth="1.5" strokeDasharray="2 2" />
            {/* Live arm */}
            <line x1={bx} y1={by} x2={jx} y2={jy} stroke="var(--plot-1)" strokeWidth="4" strokeLinecap="round" />
            <line x1={jx} y1={jy} x2={ex} y2={ey} stroke="var(--accent-2)" strokeWidth="4" strokeLinecap="round" />
            <circle cx={bx} cy={by} r="6" fill="var(--bg)" stroke="var(--text)" strokeWidth="1.5" />
            <circle cx={jx} cy={jy} r="5" fill="var(--bg-elev)" stroke="var(--text-dim)" strokeWidth="1.5" />
            <circle cx={ex} cy={ey} r="4" fill="var(--plot-2)" />
            <text x={6} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
              t = {time.toFixed(2)}s
            </text>
          </svg>

          {/* Time-series plot */}
          <Plot
            series={[q1Series, q2Series, q1dSeries, q2dSeries]}
            xLabel="t (s)"
            yLabel="q (rad)"
            xRange={[0, 4]}
            width={420}
            height={H}
            vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
          />
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">|e₁| final</div><div className="value">{e1.toFixed(3)}</div></div>
          <div className="metric"><div className="label">|e₂| final</div><div className="value">{e2.toFixed(3)}</div></div>
          <div className="metric"><div className="label">t</div><div className="value">{time.toFixed(2)}s</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => {
            if (time >= 4.0) setTime(0);
            setRunning(r => !r);
          }}>{running ? 'Pause' : (time >= 4.0 ? 'Replay' : 'Play')}</button>
          <button className="btn" onClick={() => { setRunning(false); setTime(0); }}>Reset</button>
        </div>

        <p className="note">
          <strong>PD only</strong> shows large steady-state offset under gravity — joints sag.
          <strong> PD + g(q)</strong> compensates gravity at the model level: e → 0 asymptotically (Siciliano eq. 8.49).
          <strong> Computed torque</strong> linearizes and decouples the system into two independent double-integrators (eq. 8.57) —
          identical fast convergence on both joints regardless of pose-dependent coupling.
        </p>
      </div>

      <div className="controls">
        <h3>Controller</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${controller === 'pd' ? 'primary' : ''}`} onClick={() => setController('pd')}>PD</button>
          <button className={`btn ${controller === 'pdg' ? 'primary' : ''}`} onClick={() => setController('pdg')}>PD + g</button>
          <button className={`btn ${controller === 'ct' ? 'primary' : ''}`} onClick={() => setController('ct')}>Comp. torque</button>
        </div>

        <Slider label="Kₚ" value={Kp} onChange={setKp} min={0} max={300} step={1} precision={0} />
        <Slider label="Kd" value={Kd} onChange={setKd} min={0} max={80} step={0.5} precision={1} />

        <h3 style={{ marginTop: 20 }}>Setpoint qd</h3>
        <Slider label="q₁d (rad)" value={q1d} onChange={setQ1d} min={-3} max={3} step={0.05} />
        <Slider label="q₂d (rad)" value={q2d} onChange={setQ2d} min={-3} max={3} step={0.05} />

        <h3 style={{ marginTop: 20 }}>Masses</h3>
        <Slider label="m₁ (kg)" value={m1} onChange={setM1} min={0.1} max={3} step={0.05} />
        <Slider label="m₂ (kg)" value={m2} onChange={setM2} min={0.1} max={3} step={0.05} />
      </div>
    </div>
  );
}
