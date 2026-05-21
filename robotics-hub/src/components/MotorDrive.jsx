import React, { useState, useMemo, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Siciliano §6.6: Permanent-magnet DC motor + drive.
 *
 *   Armature circuit:    L_a di/dt = V_a − R_a i − k_e ω
 *   Mechanical:          J_m dω/dt = k_t i − F_m ω − τ_L / k_r
 *
 * where:
 *   V_a = armature voltage (control input)
 *   i   = armature current
 *   ω   = motor angular velocity
 *   k_t = torque constant
 *   k_e = back-EMF constant (= k_t in SI)
 *   R_a = armature resistance
 *   L_a = armature inductance
 *   J_m = motor inertia (reflected)
 *   F_m = viscous friction
 *   τ_L = load torque at output, k_r = gear ratio
 *
 * Cascade control (§6.6):
 *   - Inner current loop (high BW): PI on (i_ref − i) → V_a
 *   - Outer velocity loop (lower BW): PI on (ω_ref − ω) → i_ref
 *
 * Encoder model (§6.4.1): incremental encoder with N counts/rev
 *   measured_position = round(θ · N / 2π) · 2π / N  (quantization)
 */

export default function MotorDrive() {
  const [omegaRef, setOmegaRef] = useState(50);
  const [tauLoad, setTauLoad] = useState(0.1);
  const [Ra, setRa] = useState(1.0);
  const [La, setLa] = useState(0.005);
  const [kt, setKt] = useState(0.1);
  const [Jm, setJm] = useState(0.001);
  const [Kp_i, setKpI] = useState(20);
  const [Ki_i, setKiI] = useState(800);
  const [Kp_w, setKpW] = useState(0.05);
  const [Ki_w, setKiW] = useState(0.5);
  const [encoderN, setEncoderN] = useState(2000);
  const [time, setTime] = useState(0);
  const [running, setRunning] = useState(false);
  const animRef = useRef(null);

  const Fm = 0.0005; // viscous
  const tFinal = 1.0;
  const dt = 0.00005; // very small dt for fast electrical dynamics

  const sim = useMemo(() => {
    // State: [i, omega, theta, eInt_i, eInt_w]
    const f = (t, x) => {
      const [i, w, theta, eI_i, eI_w] = x;

      // Outer velocity loop
      const eW = omegaRef - w;
      const iRef = Kp_w * eW + Ki_w * eI_w;
      const iRefSat = Math.max(-20, Math.min(20, iRef));

      // Inner current loop
      const eI = iRefSat - i;
      const Va = Kp_i * eI + Ki_i * eI_i;
      const VaSat = Math.max(-48, Math.min(48, Va));

      // Plant
      const di = (VaSat - Ra * i - kt * w) / La;
      const dw = (kt * i - Fm * w - tauLoad) / Jm;
      const dtheta = w;

      return [di, dw, dtheta, eI, eW];
    };

    // Simulate with smaller dt for stiffness
    const N = Math.floor(tFinal / dt);
    const result = [];
    let x = [0, 0, 0, 0, 0];
    let t = 0;
    // Sample every M steps
    const M = Math.max(1, Math.floor(N / 500));
    result.push({ t, x: [...x] });
    for (let k = 0; k < N; k++) {
      // Simple RK4 step inline for speed
      const k1 = f(t, x);
      const x2 = x.map((xi, j) => xi + 0.5 * dt * k1[j]);
      const k2 = f(t + 0.5 * dt, x2);
      const x3 = x.map((xi, j) => xi + 0.5 * dt * k2[j]);
      const k3 = f(t + 0.5 * dt, x3);
      const x4 = x.map((xi, j) => xi + dt * k3[j]);
      const k4 = f(t + dt, x4);
      x = x.map((xi, j) => xi + (dt / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
      t += dt;
      if (k % M === 0) result.push({ t, x: [...x] });
    }
    result.push({ t, x: [...x] });
    return result;
  }, [omegaRef, tauLoad, Ra, La, kt, Jm, Kp_i, Ki_i, Kp_w, Ki_w]);

  // Animation
  useEffect(() => {
    if (!running) return;
    let startWall = performance.now();
    let startT = time;
    const tick = () => {
      const elapsed = (performance.now() - startWall) / 1000;
      const newT = startT + elapsed;
      if (newT >= tFinal) {
        setTime(tFinal);
        setRunning(false);
        return;
      }
      setTime(newT);
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  useEffect(() => { setTime(0); }, [omegaRef, tauLoad, Ra, La, kt, Jm, Kp_i, Ki_i, Kp_w, Ki_w]);

  // Find snapshot
  const idx = Math.min(sim.length - 1, sim.findIndex(s => s.t >= time));
  const snap = sim[idx >= 0 ? idx : sim.length - 1];
  const thetaContinuous = snap.x[2];

  // Encoder quantization
  const counts = Math.round(thetaContinuous * encoderN / (2 * Math.PI));
  const thetaMeasured = counts * 2 * Math.PI / encoderN;
  const encoderError = thetaContinuous - thetaMeasured;

  // Plot series
  const omegaSeries = { name: 'ω', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, s.x[1]]) };
  const omegaRefSeries = { name: 'ω_ref', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, omegaRef]) };
  const iSeries = { name: 'i', color: 'var(--accent-2)', width: 2,
    points: sim.map(s => [s.t, s.x[0]]) };

  // Steady-state info
  const fin = sim[sim.length - 1].x;
  const wError = Math.abs(omegaRef - fin[1]);
  const iSS = fin[0];

  // Motor visualization: rotating shaft
  const W = 200, H = 200;
  const ox = W / 2, oy = H / 2;
  const r = 60;
  const shaftAng = thetaContinuous;
  const tickAng = thetaMeasured;

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>DC motor with cascade control + encoder</h3>
        <p className="eq">
          L_a·di/dt = V_a − R_a·i − k_e·ω &nbsp;|&nbsp; J_m·dω/dt = k_t·i − F_m·ω − τ_L
        </p>

        <Explainer
          title="What's inside a robot joint?"
          plain="Every joint has a motor and a sensor. The motor (DC here) has TWO physics going on at once: electrical (current flowing through a coil) and mechanical (the shaft spinning). Current makes torque (τ = k_t · i). Torque makes velocity, but velocity also creates a back-voltage that fights the current. To control velocity, we use TWO loops nested: an inner fast loop controls current, an outer slower loop controls velocity. The encoder counts gear teeth — it doesn't measure smooth angle but instead steps (2π / N counts). Coarse encoder = jagged velocity estimate."
          equation="V_a = L_a·di/dt + R_a·i + k_e·ω,   τ = k_t·i"
          equationNote="Siciliano eq. 6.20. Back-EMF k_e·ω opposes the applied voltage, naturally limiting speed. The torque constant k_t equals k_e in SI units."
          knobs={[
            { name: 'ω_ref', what: 'target shaft speed' },
            { name: 'Kp_ω / Ki_ω', what: 'outer velocity loop gains — set the response speed' },
            { name: 'Kp_i / Ki_i', what: 'inner current loop gains — much faster than outer loop' },
            { name: 'R_a, L_a', what: 'motor electrical parameters — affect current loop bandwidth' },
            { name: 'counts / rev', what: 'encoder resolution — try 50 to see quantization clearly' }
          ]} />

        <div style={{ display: 'grid', gridTemplateColumns: '200px 1fr', gap: 16, alignItems: 'start' }}>
          {/* Motor + encoder */}
          <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}
            style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
            {/* Outer shell */}
            <circle cx={ox} cy={oy} r={r + 12} fill="var(--bg-elev)" stroke="var(--border)" strokeWidth="1" />
            <circle cx={ox} cy={oy} r={r} fill="var(--bg)" stroke="var(--border)" strokeWidth="0.5" />

            {/* Encoder tick marks */}
            {[...Array(24)].map((_, k) => {
              const a = (k / 24) * 2 * Math.PI;
              const x1 = ox + (r - 4) * Math.cos(a);
              const y1 = oy + (r - 4) * Math.sin(a);
              const x2 = ox + r * Math.cos(a);
              const y2 = oy + r * Math.sin(a);
              return <line key={k} x1={x1} y1={y1} x2={x2} y2={y2} stroke="var(--text-faint)" strokeWidth="0.5" />;
            })}

            {/* True shaft (continuous) */}
            <line x1={ox} y1={oy}
              x2={ox + r * Math.cos(shaftAng - Math.PI / 2)}
              y2={oy + r * Math.sin(shaftAng - Math.PI / 2)}
              stroke="var(--plot-1)" strokeWidth="3" strokeLinecap="round" />

            {/* Encoder-measured shaft (quantized) */}
            <line x1={ox} y1={oy}
              x2={ox + (r - 10) * Math.cos(tickAng - Math.PI / 2)}
              y2={oy + (r - 10) * Math.sin(tickAng - Math.PI / 2)}
              stroke="var(--plot-2)" strokeWidth="1.5" strokeLinecap="round" strokeDasharray="3 2" />

            <circle cx={ox} cy={oy} r="4" fill="var(--text)" />

            <text x={6} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
              t = {time.toFixed(3)}s
            </text>
            <text x={6} y={H - 8} fill="var(--text-faint)" fontSize="9" fontFamily="var(--mono)">
              counts: {counts}
            </text>
          </svg>

          {/* Time-series */}
          <div style={{ display: 'grid', gap: 4 }}>
            <Plot
              series={[omegaSeries, omegaRefSeries]}
              xLabel=""
              yLabel="ω (rad/s)"
              xRange={[0, tFinal]}
              width={420}
              height={120}
              vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
            />
            <Plot
              series={[iSeries]}
              xLabel="t (s)"
              yLabel="i (A)"
              xRange={[0, tFinal]}
              width={420}
              height={110}
              vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
            />
          </div>
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">ω now</div><div className="value">{snap.x[1].toFixed(1)}</div></div>
          <div className="metric"><div className="label">i now (A)</div><div className="value">{snap.x[0].toFixed(2)}</div></div>
          <div className="metric"><div className="label">ω error SS</div><div className="value">{wError.toFixed(2)}</div></div>
          <div className="metric"><div className="label">i SS (A)</div><div className="value">{iSS.toFixed(2)}</div></div>
        </div>
        <div className="metrics">
          <div className="metric"><div className="label">encoder N</div><div className="value">{encoderN}</div></div>
          <div className="metric"><div className="label">resolution</div><div className="value">{(360 / encoderN).toFixed(3)}°</div></div>
          <div className="metric"><div className="label">quant err</div><div className="value">{(encoderError * 180 / Math.PI).toFixed(3)}°</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => {
            if (time >= tFinal) setTime(0);
            setRunning(r => !r);
          }}>{running ? 'Pause' : (time >= tFinal ? 'Replay' : 'Play')}</button>
          <button className="btn" onClick={() => { setRunning(false); setTime(0); }}>Reset</button>
        </div>

        <p className="note">
          The blue shaft is the true continuous angle θ; the dashed yellow line is what the encoder reports — quantized to
          <code> 2π/N</code>. Cascade control puts a fast PI current loop inside a slower PI velocity loop. Current saturates at ±20A
          during the transient (motor draws max torque), then settles to whatever current balances the load torque
          (<code>i_ss ≈ τ_L / k_t</code>). Lower the encoder resolution to see how velocity estimation noise grows when you
          differentiate the quantized signal.
        </p>
      </div>

      <div className="controls">
        <h3>Reference</h3>
        <Slider label="ω_ref (rad/s)" value={omegaRef} onChange={setOmegaRef} min={-100} max={100} step={1} precision={0} />
        <Slider label="τ_load (N·m)" value={tauLoad} onChange={setTauLoad} min={-1} max={1} step={0.01} />

        <h3 style={{ marginTop: 16 }}>Velocity loop</h3>
        <Slider label="Kp_ω" value={Kp_w} onChange={setKpW} min={0} max={0.5} step={0.005} precision={3} />
        <Slider label="Ki_ω" value={Ki_w} onChange={setKiW} min={0} max={10} step={0.05} precision={2} />

        <h3 style={{ marginTop: 16 }}>Current loop</h3>
        <Slider label="Kp_i" value={Kp_i} onChange={setKpI} min={0} max={100} step={0.5} precision={1} />
        <Slider label="Ki_i" value={Ki_i} onChange={setKiI} min={0} max={3000} step={10} precision={0} />

        <h3 style={{ marginTop: 16 }}>Motor</h3>
        <Slider label="R_a (Ω)" value={Ra} onChange={setRa} min={0.1} max={5} step={0.05} />
        <Slider label="L_a (mH)" value={La * 1000} onChange={v => setLa(v / 1000)} min={0.5} max={20} step={0.5} precision={1} />
        <Slider label="k_t (N·m/A)" value={kt} onChange={setKt} min={0.02} max={0.5} step={0.005} precision={3} />
        <Slider label="J_m (kg·m²)" value={Jm * 1000} onChange={v => setJm(v / 1000)} min={0.1} max={5} step={0.05} precision={2} />

        <h3 style={{ marginTop: 16 }}>Encoder</h3>
        <Slider label="counts / rev" value={encoderN} onChange={v => setEncoderN(Math.round(v))} min={50} max={10000} step={50} precision={0} />
      </div>
    </div>
  );
}
