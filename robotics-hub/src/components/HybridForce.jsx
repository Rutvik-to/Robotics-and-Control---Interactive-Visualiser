import React, { useState, useMemo, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Siciliano §9.4 (Hybrid force/position) and §9.4.3 (Admittance).
 *
 *  HYBRID (planar peg pressing on a surface):
 *    - x-direction: free motion, position controlled
 *    - y-direction: constrained by surface, force controlled
 *    Selection matrix S diagonally splits control:
 *      S = diag(1, 0) -> position in x
 *      I - S = diag(0, 1) -> force in y
 *    τ_pos = K_p (x_d - x)
 *    τ_force = K_f (F_d - F_meas)  with F_meas = K_env max(0, y - y_surf)
 *
 *  ADMITTANCE (Siciliano eq. 9.46):
 *    Measure force F_ext; integrate "virtual" dynamics to get desired position.
 *      M_a ẍd + D_a ẋd + K_a (xd - x_ref) = F_ext
 *    Then a fast inner position loop tracks xd.
 *    The DIFFERENCE from impedance: impedance is stiff position with soft force
 *    behaviour; admittance is the dual — soft position from measured force.
 */

export default function HybridForce() {
  const [mode, setMode] = useState('hybrid'); // 'hybrid' | 'admittance'

  // Hybrid params
  const [xd, setXd] = useState(1.5);
  const [Fd, setFd] = useState(5);        // desired contact force in y
  const [ySurf, setYSurf] = useState(0);  // surface y-coordinate
  const [Kenv, setKenv] = useState(800);  // surface stiffness
  const [Kp, setKp] = useState(80);
  const [Kf, setKf] = useState(10.0);
  const [Kd_, setKd_] = useState(15);     // position damping

  // Admittance params
  const [Ma, setMa] = useState(2);
  const [Da, setDa] = useState(15);
  const [Ka, setKa] = useState(40);
  const [pushF, setPushF] = useState(0);
  const [xRef, setXRef] = useState(0.5);

  const [time, setTime] = useState(0);
  const [running, setRunning] = useState(true);
  const animRef = useRef(null);

  const tFinal = 5.0;

  // HYBRID simulation: state = [x, xdot, y, ydot]
  const simHybrid = useMemo(() => {
    const dt = 0.002;
    const m = 1.0;
    const f = (t, st) => {
      const [x, xdot, y, ydot] = st;
      // Position control in x
      const tau_x = Kp * (xd - x) - Kd_ * xdot;
      // Force feedback in y
      const Fmeas = y > ySurf ? Kenv * (y - ySurf) : 0;
      // Move toward surface, then regulate force
      const Ferr = Fd - Fmeas;
      const tau_y = Kf * Ferr - Kd_ * ydot;
      // True dynamics (mass-on-surface): the surface pushes back when penetrated
      const Fsurf = y > ySurf ? -Kenv * (y - ySurf) : 0;
      const yddot = (tau_y + Fsurf - 0.5 * ydot) / m;
      const xddot = (tau_x) / m;
      return [xdot, xddot, ydot, yddot];
    };
    // Start above surface, off to the left
    return simulate(f, [0, 0, 0.3, 0], 0, tFinal, dt);
  }, [xd, Fd, ySurf, Kenv, Kp, Kf, Kd_]);

  // ADMITTANCE simulation: state = [x, xdot, xd_virtual, xddot_virtual]
  // Outer loop: virtual dynamics M_a ẍd + D_a ẋd + K_a (xd - xref) = F_ext
  // Inner loop: tight position controller follows xd
  const simAdm = useMemo(() => {
    const dt = 0.002;
    const m = 1.0;
    const Kp_inner = 200;
    const Kd_inner = 30;
    const f = (t, st) => {
      const [x, xdot, xd_v, xd_v_dot] = st;
      // Admittance virtual dynamics generate xd from external force
      const xd_v_ddot = (pushF - Da * xd_v_dot - Ka * (xd_v - xRef)) / Ma;
      // Inner position controller tracks xd_v
      const tau = Kp_inner * (xd_v - x) + Kd_inner * (xd_v_dot - xdot);
      const xddot = tau / m;
      return [xdot, xddot, xd_v_dot, xd_v_ddot];
    };
    return simulate(f, [xRef, 0, xRef, 0], 0, tFinal, dt);
  }, [Ma, Da, Ka, xRef, pushF]);

  useEffect(() => {
    if (!running) return;
    let wallStart = performance.now();
    let tStart = time;
    const tick = () => {
      const e = (performance.now() - wallStart) / 1000;
      const nt = tStart + e;
      if (nt >= tFinal) {
        setTime(0); wallStart = performance.now(); tStart = 0;
      } else {
        setTime(nt);
      }
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  useEffect(() => { setTime(0); }, [mode, xd, Fd, ySurf, Kenv, Kp, Kf, Ma, Da, Ka, pushF, xRef]);

  // Active simulation
  const sim = mode === 'hybrid' ? simHybrid : simAdm;
  const idx = Math.min(sim.length - 1, Math.floor(time / 0.002));
  const snap = sim[idx];

  // For hybrid: x, y, measured force
  const x_h = mode === 'hybrid' ? snap.x[0] : snap.x[0];
  const y_h = mode === 'hybrid' ? snap.x[2] : 0;
  const Fmeas_now = mode === 'hybrid' && snap.x[2] > ySurf ? Kenv * (snap.x[2] - ySurf) : 0;

  // SVG scene
  const W = 460, H = 240;
  const scale = 90;
  const ox = 60, oy = H - 60;
  const toS = (x, y) => [ox + x * scale, oy - y * scale];

  // Series for plots
  const xSeries = { name: 'x', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, s.x[0]]) };
  const xdSeries = { name: 'xd', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, mode === 'hybrid' ? xd : xRef]) };

  // For hybrid: F measured vs F desired
  const FmeasSeries = mode === 'hybrid' ? {
    name: 'F_meas', color: 'var(--warn)', width: 2,
    points: sim.map(s => [s.t, s.x[2] > ySurf ? Kenv * (s.x[2] - ySurf) : 0])
  } : null;
  const FdSeries = mode === 'hybrid' ? {
    name: 'Fd', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, Fd])
  } : null;

  // For admittance: xd_v (virtual) vs x (actual)
  const xdvSeries = mode === 'admittance' ? {
    name: 'xd_v', color: 'var(--accent-2)', width: 2, dash: '4 3',
    points: sim.map(s => [s.t, s.x[2]])
  } : null;

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>{mode === 'hybrid' ? 'Hybrid force/position control' : 'Admittance control'}</h3>
        <p className="eq">
          {mode === 'hybrid'
            ? 'S = diag(1,0): position in x · (I−S) = diag(0,1): force in y'
            : 'Mₐ ẍd + Dₐ ẋd + Kₐ(xd − x_ref) = F_ext → inner pos. loop tracks xd'}
        </p>

        <Explainer
          title={mode === 'hybrid' ? 'What is hybrid force/position?' : 'What is admittance control?'}
          plain={mode === 'hybrid'
            ? "Imagine wiping a table. You need to (1) press DOWN with a controlled force so the cloth touches, and (2) move SIDEWAYS along the surface with a controlled position. Hybrid control splits the workspace: each direction is either force-controlled OR position-controlled, never both. A selection matrix S decides which is which. The robot moves left/right freely while pushing into the surface with exactly the force you specified."
            : "Admittance is the mirror image of impedance. Impedance says 'I'm at xd, but if you push me with force F, I'll deflect by F/K.' Admittance says 'tell me the force you're applying; I'll compute where I should go, and a tight position loop drives me there.' Common in heavy industrial robots where you'd never trust their joints to be soft — instead, you measure force and SHIFT the setpoint to mimic compliance."}
          equation={mode === 'hybrid'
            ? 'τ = S·K_p(x_d − x) + (I−S)·K_f(F_d − F)'
            : 'F_ext → [Mₐ ẍd + Dₐ ẋd + Kₐ(xd−x_ref) = F_ext] → xd → fast pos. loop'}
          equationNote={mode === 'hybrid'
            ? 'Siciliano eq. 9.22. The selection matrix S is diagonal with 1\'s for position-controlled axes and 0\'s for force-controlled axes. In our peg example, S = diag(1,0).'
            : 'Siciliano §9.4.3. Admittance is preferred when the robot has stiff position control but a force/torque sensor at the wrist — most industrial robots.'}
          knobs={mode === 'hybrid' ? [
            { name: 'x_d', what: 'where the peg should move along the surface' },
            { name: 'F_d', what: 'how hard to press into the surface (N)' },
            { name: 'K_p', what: 'how stiff the position loop is in x' },
            { name: 'K_f', what: 'force feedback gain — too high causes oscillation when in contact' },
            { name: 'K_env', what: 'surface stiffness (the world, not the robot) — stiff env = harder to control force' }
          ] : [
            { name: 'push F', what: 'force YOU apply to the end-effector (sensor reading)' },
            { name: 'Mₐ, Dₐ, Kₐ', what: 'virtual mass-damper-spring shaping the response to F' },
            { name: 'x_ref', what: 'rest position the spring tries to return to' }
          ]} />

        {/* Scene */}
        {mode === 'hybrid' && (
          <svg width="100%" viewBox={`0 0 ${W} ${H}`}
            style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
            {/* Surface */}
            <line x1={ox - 20} y1={oy - ySurf * scale} x2={W - 20} y2={oy - ySurf * scale}
              stroke="var(--danger)" strokeWidth="2" />
            {/* Hatching */}
            {[...Array(20)].map((_, i) => (
              <line key={i}
                x1={ox - 20 + i * 22} y1={oy - ySurf * scale + 2}
                x2={ox - 30 + i * 22} y2={oy - ySurf * scale + 12}
                stroke="var(--danger)" strokeWidth="0.7" />
            ))}
            {/* Position target marker */}
            {(() => {
              const [tx, ty] = toS(xd, ySurf + 0.4);
              return (
                <g>
                  <line x1={tx} y1={oy - ySurf * scale - 60} x2={tx} y2={oy - ySurf * scale}
                    stroke="var(--plot-2)" strokeWidth="1" strokeDasharray="3 3" />
                  <text x={tx} y={oy - ySurf * scale - 64} textAnchor="middle"
                    fill="var(--plot-2)" fontSize="10" fontFamily="var(--mono)">xd</text>
                </g>
              );
            })()}
            {/* Peg */}
            {(() => {
              const [px, py] = toS(x_h, y_h);
              return (
                <g>
                  <rect x={px - 8} y={py - 30} width={16} height={30}
                    fill="var(--plot-1)" rx="2" />
                  <line x1={px} y1={py} x2={px} y2={py + (Fmeas_now > 0 ? 14 : 0)}
                    stroke="var(--warn)" strokeWidth="2"
                    markerEnd={Fmeas_now > 0 ? "url(#arr-h)" : ""} />
                  {Fmeas_now > 0 && (
                    <text x={px + 8} y={py + 12} fill="var(--warn)" fontSize="10" fontFamily="var(--mono)">
                      F={Fmeas_now.toFixed(1)}N
                    </text>
                  )}
                </g>
              );
            })()}
            <defs>
              <marker id="arr-h" viewBox="0 0 10 10" refX="8" refY="5"
                markerWidth="6" markerHeight="6" orient="auto-start-reverse">
                <path d="M2 1L8 5L2 9" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
              </marker>
            </defs>
            <text x={10} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
              t = {time.toFixed(2)}s | x = {x_h.toFixed(3)} | F = {Fmeas_now.toFixed(1)}N
            </text>
          </svg>
        )}

        {/* Plots */}
        <div style={{ marginTop: 12 }}>
          <Plot
            series={mode === 'hybrid'
              ? [xSeries, xdSeries]
              : [xSeries, xdSeries, xdvSeries]}
            xLabel={mode === 'hybrid' ? '' : 't (s)'}
            yLabel="x (m)"
            xRange={[0, tFinal]}
            width={640}
            height={mode === 'hybrid' ? 130 : 160}
            vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
          />
          {mode === 'hybrid' && (
            <Plot
              series={[FmeasSeries, FdSeries]}
              xLabel="t (s)"
              yLabel="F (N)"
              xRange={[0, tFinal]}
              width={640}
              height={130}
              vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
            />
          )}
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">x now</div><div className="value">{snap.x[0].toFixed(3)}</div></div>
          {mode === 'hybrid' && (
            <>
              <div className="metric"><div className="label">y now</div><div className="value">{snap.x[2].toFixed(3)}</div></div>
              <div className="metric"><div className="label">F meas</div><div className="value">{Fmeas_now.toFixed(1)}N</div></div>
              <div className="metric"><div className="label">F error</div><div className="value">{(Fd - Fmeas_now).toFixed(1)}</div></div>
            </>
          )}
          {mode === 'admittance' && (
            <>
              <div className="metric"><div className="label">xd_v</div><div className="value">{snap.x[2].toFixed(3)}</div></div>
              <div className="metric"><div className="label">deflection</div><div className="value">{(snap.x[2] - xRef).toFixed(3)}</div></div>
              <div className="metric"><div className="label">SS predict</div><div className="value">{(pushF / Ka).toFixed(3)}</div></div>
            </>
          )}
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Play'}
          </button>
        </div>
      </div>

      <div className="controls">
        <h3>Mode</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${mode === 'hybrid' ? 'primary' : ''}`}
            onClick={() => setMode('hybrid')}>Hybrid</button>
          <button className={`btn ${mode === 'admittance' ? 'primary' : ''}`}
            onClick={() => setMode('admittance')}>Admittance</button>
        </div>

        {mode === 'hybrid' && (
          <>
            <h3>Position axis (x)</h3>
            <Slider label="x_d target" value={xd} onChange={setXd} min={0.2} max={2.5} step={0.02} />
            <Slider label="K_p position" value={Kp} onChange={setKp} min={1} max={300} step={1} precision={0} />

            <h3 style={{ marginTop: 16 }}>Force axis (y)</h3>
            <Slider label="F_d desired (N)" value={Fd} onChange={setFd} min={0} max={20} step={0.2} precision={1} />
            <Slider label="K_f force gain" value={Kf} onChange={setKf} min={0.1} max={50} step={0.1} precision={2} />
            <Slider label="K_env surface" value={Kenv} onChange={setKenv} min={100} max={3000} step={20} precision={0} />

            <h3 style={{ marginTop: 16 }}>Damping</h3>
            <Slider label="K_d" value={Kd_} onChange={setKd_} min={0} max={50} step={0.5} precision={1} />
          </>
        )}

        {mode === 'admittance' && (
          <>
            <h3>Virtual impedance</h3>
            <Slider label="Mₐ virtual mass" value={Ma} onChange={setMa} min={0.1} max={10} step={0.1} />
            <Slider label="Dₐ damping" value={Da} onChange={setDa} min={1} max={50} step={0.5} precision={1} />
            <Slider label="Kₐ stiffness" value={Ka} onChange={setKa} min={5} max={200} step={1} precision={0} />

            <h3 style={{ marginTop: 16 }}>Reference</h3>
            <Slider label="x_ref" value={xRef} onChange={setXRef} min={0} max={2} step={0.02} />

            <h3 style={{ marginTop: 16 }}>External force</h3>
            <Slider label="push F (N)" value={pushF} onChange={setPushF} min={-30} max={30} step={0.5} precision={1} />
          </>
        )}
      </div>
    </div>
  );
}
