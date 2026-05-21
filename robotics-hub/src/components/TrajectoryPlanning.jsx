import React, { useState, useMemo } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano Ch. 4: Trajectory planning in joint space.
 *
 *  Polynomial (cubic / quintic):
 *    Cubic q(t) = a0 + a1 t + a2 t² + a3 t³
 *      Boundary: q(0)=qi, q(T)=qf, q̇(0)=0, q̇(T)=0
 *      Closed form: a0 = qi, a1 = 0, a2 = 3(qf-qi)/T², a3 = -2(qf-qi)/T³
 *
 *    Quintic q(t) = a0 + a1 t + ... + a5 t⁵
 *      Boundary: q, q̇, q̈ at both ends zero except q(0)=qi, q(T)=qf
 *      Closed form: a3 = 10D/T³, a4 = -15D/T⁴, a5 = 6D/T⁵ where D=qf-qi
 *
 *  Trapezoidal velocity profile (bang-coast-bang):
 *    Three phases:
 *      0 ≤ t ≤ tc:     q̈ = +a_max  (acceleration)
 *      tc ≤ t ≤ T-tc:  q̈ = 0       (cruise at v_max)
 *      T-tc ≤ t ≤ T:   q̈ = -a_max  (deceleration)
 *    with tc·a = v_max, and v_max·(T-tc) = qf-qi (signed)
 */

// ---- Profile generators ----
function cubicProfile(qi, qf, T, N = 200) {
  const D = qf - qi;
  const pts = [];
  for (let i = 0; i <= N; i++) {
    const t = (i / N) * T;
    const s = t / T;
    const q = qi + D * (3 * s * s - 2 * s * s * s);
    const qd = (D / T) * (6 * s - 6 * s * s);
    const qdd = (D / (T * T)) * (6 - 12 * s);
    pts.push({ t, q, qd, qdd });
  }
  return pts;
}

function quinticProfile(qi, qf, T, N = 200) {
  const D = qf - qi;
  const pts = [];
  for (let i = 0; i <= N; i++) {
    const t = (i / N) * T;
    const s = t / T;
    const q = qi + D * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5);
    const qd = (D / T) * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4);
    const qdd = (D / (T * T)) * (60 * s - 180 * s ** 2 + 120 * s ** 3);
    pts.push({ t, q, qd, qdd });
  }
  return pts;
}

function trapezoidalProfile(qi, qf, T, vMax, N = 200) {
  const D = qf - qi;
  const sign = D >= 0 ? 1 : -1;
  // Required ramp time so that area = D under velocity curve.
  // v_max * (T - tc) = D  →  tc = T - |D|/v_max
  const tc = T - Math.abs(D) / vMax;
  // If tc <= 0, v_max isn't reachable in time T — use triangular profile.
  const triangular = tc <= 0;
  const tcReal = triangular ? T / 2 : tc;
  // For triangular profile, peak velocity:
  const vPeak = triangular ? 2 * Math.abs(D) / T : vMax;
  const a = vPeak / tcReal;
  const pts = [];
  for (let i = 0; i <= N; i++) {
    const t = (i / N) * T;
    let q, qd, qdd;
    if (t <= tcReal) {
      qdd = sign * a;
      qd = sign * a * t;
      q = qi + sign * 0.5 * a * t * t;
    } else if (t <= T - tcReal) {
      qdd = 0;
      qd = sign * vPeak;
      q = qi + sign * (0.5 * a * tcReal * tcReal + vPeak * (t - tcReal));
    } else {
      const tau = T - t;
      qdd = -sign * a;
      qd = sign * a * tau;
      q = qf - sign * 0.5 * a * tau * tau;
    }
    pts.push({ t, q, qd, qdd });
  }
  return { points: pts, tc: tcReal, triangular, vPeak };
}

export default function TrajectoryPlanning() {
  const [profile, setProfile] = useState('quintic'); // 'cubic' | 'quintic' | 'trap'
  const [qi, setQi] = useState(0);
  const [qf, setQf] = useState(2.0);
  const [T, setT] = useState(2.0);
  const [vMax, setVMax] = useState(1.5);
  const [compareAll, setCompareAll] = useState(false);

  const cubic = useMemo(() => cubicProfile(qi, qf, T), [qi, qf, T]);
  const quintic = useMemo(() => quinticProfile(qi, qf, T), [qi, qf, T]);
  const trap = useMemo(() => trapezoidalProfile(qi, qf, T, vMax), [qi, qf, T, vMax]);

  const active = profile === 'cubic' ? cubic : profile === 'quintic' ? quintic : trap.points;

  // Build series for three plots
  const colors = { cubic: 'var(--plot-4)', quintic: 'var(--plot-1)', trap: 'var(--accent-2)' };

  const buildSeries = (key) => {
    const pts = key === 'cubic' ? cubic : key === 'quintic' ? quintic : trap.points;
    return {
      q: { name: key, color: colors[key], width: 2, points: pts.map(p => [p.t, p.q]) },
      qd: { name: key, color: colors[key], width: 2, points: pts.map(p => [p.t, p.qd]) },
      qdd: { name: key, color: colors[key], width: 2, points: pts.map(p => [p.t, p.qdd]) },
    };
  };

  const seriesByProfile = {
    cubic: buildSeries('cubic'),
    quintic: buildSeries('quintic'),
    trap: buildSeries('trap')
  };

  const qSeries = compareAll
    ? [seriesByProfile.cubic.q, seriesByProfile.quintic.q, seriesByProfile.trap.q]
    : [seriesByProfile[profile].q];
  const qdSeries = compareAll
    ? [seriesByProfile.cubic.qd, seriesByProfile.quintic.qd, seriesByProfile.trap.qd]
    : [seriesByProfile[profile].qd];
  const qddSeries = compareAll
    ? [seriesByProfile.cubic.qdd, seriesByProfile.quintic.qdd, seriesByProfile.trap.qdd]
    : [seriesByProfile[profile].qdd];

  // Peak velocity / acceleration of active profile
  const peakV = Math.max(...active.map(p => Math.abs(p.qd)));
  const peakA = Math.max(...active.map(p => Math.abs(p.qdd)));

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Joint-space trajectory: position, velocity, acceleration</h3>
        <p className="eq">
          {profile === 'cubic' && 'q(t) = qᵢ + D(3s² − 2s³),  s = t/T'}
          {profile === 'quintic' && 'q(t) = qᵢ + D(10s³ − 15s⁴ + 6s⁵),  s = t/T'}
          {profile === 'trap' && 'Bang-coast-bang: ±a in ramps, 0 in cruise'}
        </p>

        <Explainer
          title="Why not just snap from A to B?"
          plain="If you commanded the robot to instantly jump from start to goal, the motors would explode and the arm would shake itself apart. We need a SMOOTH path through time — gentle on acceleration. Cubic curves (3rd-order polynomials) match position and velocity at both ends but jolt acceleration. Quintic (5th-order) also matches acceleration at the ends — zero jerk. Trapezoidal is what cars actually do: accelerate hard, cruise, decelerate hard. It's time-optimal but the square-wave acceleration vibrates the structure."
          equation="quintic: q(t) = qᵢ + D·(10s³ − 15s⁴ + 6s⁵),  s = t/T"
          equationNote="Siciliano eq. 4.16 (cubic), 4.21 (quintic), §4.2.4 (trapezoidal). Quintic is the smoothest, trapezoidal the fastest."
          knobs={[
            { name: 'Profile', what: 'cubic = smooth-ish; quintic = silky; trap = fastest' },
            { name: 'qᵢ → q_f', what: 'start and goal joint angles' },
            { name: 'T', what: 'how long the motion takes — shorter T means higher peak speed' },
            { name: 'v_max', what: 'velocity ceiling for trapezoidal; if T is too short for D/v_max, profile becomes triangular' },
            { name: 'Compare all', what: 'overlays all three so you can see the trade-offs at once' }
          ]} />

        <div style={{ display: 'grid', gap: 8 }}>
          <Plot
            series={qSeries}
            xLabel="t (s)"
            yLabel="q (rad)"
            xRange={[0, T]}
            height={150}
            width={640}
          />
          <Plot
            series={qdSeries}
            xLabel="t (s)"
            yLabel="q̇ (rad/s)"
            xRange={[0, T]}
            height={130}
            width={640}
          />
          <Plot
            series={qddSeries}
            xLabel="t (s)"
            yLabel="q̈ (rad/s²)"
            xRange={[0, T]}
            height={130}
            width={640}
          />
        </div>

        {compareAll && (
          <div className="legend" style={{ marginTop: 10 }}>
            <span><span className="swatch" style={{ background: colors.cubic }} />cubic (3-2)</span>
            <span><span className="swatch" style={{ background: colors.quintic }} />quintic (5-4-3)</span>
            <span><span className="swatch" style={{ background: colors.trap }} />trapezoidal</span>
          </div>
        )}

        <div className="metrics">
          <div className="metric"><div className="label">peak |q̇|</div><div className="value">{peakV.toFixed(2)}</div></div>
          <div className="metric"><div className="label">peak |q̈|</div><div className="value">{peakA.toFixed(2)}</div></div>
          <div className="metric"><div className="label">duration</div><div className="value">{T.toFixed(2)}s</div></div>
          {profile === 'trap' && (
            <div className="metric">
              <div className="label">{trap.triangular ? 'triangular' : 'tc ramp'}</div>
              <div className="value">{trap.triangular ? `v=${trap.vPeak.toFixed(2)}` : `${trap.tc.toFixed(2)}s`}</div>
            </div>
          )}
        </div>

        <p className="note">
          <strong>Cubic</strong> guarantees continuous position and velocity but has a step in acceleration at t=0 and t=T — fine
          for low-speed moves, but the discontinuity excites high-frequency vibration modes.
          <strong> Quintic</strong> zeros acceleration at both endpoints too — smoother torque demand, slightly higher peak velocity.
          <strong> Trapezoidal</strong> is time-optimal subject to a velocity ceiling: minimum time for given <code>v_max</code> and
          <code>a_max</code>, but acceleration is a square wave (Siciliano §4.2.4). If <code>v_max</code> is too high to reach in
          time T, it auto-degenerates to a triangular profile.
        </p>
      </div>

      <div className="controls">
        <h3>Profile</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${profile === 'cubic' ? 'primary' : ''}`} onClick={() => setProfile('cubic')}>Cubic</button>
          <button className={`btn ${profile === 'quintic' ? 'primary' : ''}`} onClick={() => setProfile('quintic')}>Quintic</button>
          <button className={`btn ${profile === 'trap' ? 'primary' : ''}`} onClick={() => setProfile('trap')}>Trapezoid</button>
        </div>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${compareAll ? 'primary' : ''}`}
            onClick={() => setCompareAll(!compareAll)}>
            Compare all: {compareAll ? 'ON' : 'OFF'}
          </button>
        </div>

        <h3>Endpoints</h3>
        <Slider label="qᵢ initial" value={qi} onChange={setQi} min={-3} max={3} step={0.05} />
        <Slider label="q_f final" value={qf} onChange={setQf} min={-3} max={3} step={0.05} />

        <h3 style={{ marginTop: 16 }}>Timing</h3>
        <Slider label="T duration (s)" value={T} onChange={setT} min={0.5} max={5} step={0.05} />
        {profile === 'trap' && (
          <Slider label="v_max" value={vMax} onChange={setVMax} min={0.2} max={5} step={0.05} />
        )}

        <div className="btn-row" style={{ marginTop: 16 }}>
          <button className="btn" onClick={() => { setQi(0); setQf(2); setT(2); setVMax(1.5); }}>Reset</button>
        </div>
      </div>
    </div>
  );
}
