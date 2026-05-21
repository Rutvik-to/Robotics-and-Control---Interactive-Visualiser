import React, { useMemo, useState } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Single-joint PID control with gravity disturbance.
 * Plant: I*q'' + b*q' + m*g*lc*sin(q) = tau
 * Control: tau = Kp*(qd - q) + Kd*(qd' - q') + Ki*integral(qd - q) dt
 * (Siciliano §8.3 — PD with gravity compensation / PID joint regulation)
 */
export default function PIDControl() {
  const [Kp, setKp] = useState(80);
  const [Kd, setKd] = useState(14);
  const [Ki, setKi] = useState(8);
  const [qd, setQd] = useState(1.5);
  const [g, setG] = useState(9.81);
  const [I, setI] = useState(1.0);
  const [b, setB] = useState(0.5);
  const [withGravityComp, setWithGravityComp] = useState(false);

  const m = 1.0, lc = 0.5;
  const tFinal = 6.0, dt = 0.005;

  const result = useMemo(() => {
    // State: [q, qdot, integral_error]
    const f = (t, x) => {
      const [q, qdot, eInt] = x;
      const e = qd - q;
      const edot = -qdot;
      const gravityComp = withGravityComp ? m * g * lc * Math.sin(q) : 0;
      const tau = Kp * e + Kd * edot + Ki * eInt + gravityComp;
      const qddot = (tau - b * qdot - m * g * lc * Math.sin(q)) / I;
      return [qdot, qddot, e];
    };
    return simulate(f, [0, 0, 0], 0, tFinal, dt);
  }, [Kp, Kd, Ki, qd, g, I, b, withGravityComp]);

  // Build series
  const tArr = result.map(s => s.t);
  const qSeries = { name: 'q(t)', color: 'var(--plot-1)', width: 2, points: result.map(s => [s.t, s.x[0]]) };
  const refSeries = { name: 'qd', color: 'var(--plot-2)', width: 1.5, dash: '5 4',
    points: tArr.map(t => [t, qd]) };

  // Metrics: steady-state error, overshoot, settling time (2%)
  const final = result[result.length - 1].x[0];
  const ssError = Math.abs(qd - final);
  let peak = result[0].x[0];
  result.forEach(s => { if (Math.abs(s.x[0] - 0) > Math.abs(peak - 0)) peak = s.x[0]; });
  // Overshoot relative to setpoint
  let maxQ = -Infinity;
  result.forEach(s => { if (s.x[0] > maxQ) maxQ = s.x[0]; });
  const overshoot = qd > 0 ? Math.max(0, ((maxQ - qd) / qd) * 100) : 0;
  // Settling time (within 2% of final)
  const band = 0.02 * Math.abs(qd) || 0.01;
  let tSettle = tFinal;
  for (let i = result.length - 1; i >= 0; i--) {
    if (Math.abs(result[i].x[0] - qd) > band) { tSettle = result[i].t; break; }
  }

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Step response: position tracking</h3>
        <p className="eq">I q̈ + b q̇ + mg·lc·sin(q) = τ &nbsp;|&nbsp; τ = Kₚ(qd − q) + Kd(q̇d − q̇) + Ki∫e dt</p>
        <Explainer
          title="What is PID control?"
          plain="Imagine balancing a broomstick on your palm. You watch how far it's tilted (the error), how fast it's tilting (the derivative), and how long it's been off-balance (the integral). A PID controller does the same thing automatically: it pushes back hard when the error is big (P), softens the push as you approach the target (D), and quietly compensates for any constant bias like gravity (I). Without gravity compensation, a hanging arm sags below the target because the controller can't push hard enough without an integral term."
          equation="τ = Kₚ·e + Kd·ė + Ki·∫e dt"
          equationNote="Siciliano eq. 8.13. Kₚ = how hard to push, Kd = how much damping, Ki = how patient to be about leftover error."
          knobs={[
            { name: 'Kₚ', what: 'strength of proportional pull-back — too high causes overshoot' },
            { name: 'Kd', what: 'damping — slows motion near the target, prevents oscillation' },
            { name: 'Ki', what: 'integral action — kills steady-state offsets but can overshoot if too aggressive' },
            { name: 'Gravity comp', what: 'adds m·g·lc·sin(q) feed-forward — exact cancellation, no integral needed' }
          ]} />
        <Plot
          series={[qSeries, refSeries]}
          xLabel="time (s)"
          yLabel="joint angle q (rad)"
          xRange={[0, tFinal]}
          width={640}
          height={280}
        />
        <div className="legend">
          <span><span className="swatch" style={{ background: 'var(--plot-1)' }} />measured q(t)</span>
          <span><span className="swatch" style={{ background: 'var(--plot-2)', borderTop: '1px dashed' }} />setpoint qd</span>
        </div>
        <div className="metrics">
          <div className="metric"><div className="label">SS error</div><div className="value">{ssError.toFixed(3)}</div></div>
          <div className="metric"><div className="label">Overshoot</div><div className="value">{overshoot.toFixed(1)}%</div></div>
          <div className="metric"><div className="label">Settling 2%</div><div className="value">{tSettle.toFixed(2)}s</div></div>
        </div>
        <p className="note">
          A heavy <code>Kₚ</code> with low <code>Kd</code> oscillates. Pure PD leaves a steady-state error
          under gravity since τ = 0 at q = qd cannot cancel <code>mg·lc·sin(qd)</code>. Enabling gravity compensation
          (Siciliano eq. 8.49) drives the error to zero without integral action.
        </p>
      </div>

      <div className="controls">
        <h3>Controller gains</h3>
        <Slider label="Kₚ proportional" value={Kp} onChange={setKp} min={0} max={400} step={1} precision={0} />
        <Slider label="Kd derivative" value={Kd} onChange={setKd} min={0} max={80} step={0.5} precision={1} />
        <Slider label="Ki integral" value={Ki} onChange={setKi} min={0} max={200} step={1} precision={0} />

        <h3 style={{ marginTop: 20 }}>Plant & reference</h3>
        <Slider label="qd setpoint (rad)" value={qd} onChange={setQd} min={-3} max={3} step={0.05} />
        <Slider label="gravity g (m/s²)" value={g} onChange={setG} min={0} max={20} step={0.1} precision={1} />
        <Slider label="inertia I" value={I} onChange={setI} min={0.1} max={5} step={0.05} />
        <Slider label="viscous b" value={b} onChange={setB} min={0} max={5} step={0.05} />

        <div className="btn-row">
          <button className={`btn ${withGravityComp ? 'primary' : ''}`}
            onClick={() => setWithGravityComp(!withGravityComp)}>
            Gravity comp: {withGravityComp ? 'ON' : 'OFF'}
          </button>
          <button className="btn" onClick={() => {
            setKp(80); setKd(14); setKi(8); setQd(1.5); setG(9.81); setI(1.0); setB(0.5);
          }}>Reset</button>
        </div>
      </div>
    </div>
  );
}
