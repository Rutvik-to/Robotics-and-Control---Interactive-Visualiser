import React, { useState, useMemo, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Siciliano Ch. 13: Interaction tasks with constrained motion.
 *
 *  Task-space decomposition (eq. 13.3):
 *    When the end-effector touches a surface (a "contact constraint"), motion
 *    along the surface normal is FORBIDDEN, motion tangent is FREE.
 *
 *    Twist:  v = J·q̇  in ℝ²
 *    Decompose v into:
 *      v_n = (n̂·v) n̂        (along normal — must be zero in contact)
 *      v_t = v - v_n         (tangent — free)
 *
 *    Force decomposition:
 *      f_n = constrained force from surface (controlled, regulated)
 *      f_t = free force (zero in ideal frictionless case)
 *
 *  The selection logic:
 *     Σ_t (tangent twist subspace)     :  control velocity  (position-like)
 *     Σ_n (normal twist subspace)      :  control force     (constraint-like)
 *
 *  Visualization: peg on a tilted surface. User picks surface angle. Robot must
 *  slide along surface (tangent: position controlled) while pressing into it
 *  (normal: force controlled). The decomposition is shown live.
 */

export default function ConstrainedMotion() {
  const [surfaceAngle, setSurfaceAngle] = useState(20 * Math.PI / 180); // degrees → rad
  const [Fd, setFd] = useState(8);
  const [vt_d, setVtD] = useState(0.3); // tangential target speed
  const [Kp_t, setKpT] = useState(40);
  const [Kf_n, setKfN] = useState(8.0);
  const [Kenv, setKenv] = useState(800);

  const [time, setTime] = useState(0);
  const [running, setRunning] = useState(true);
  const animRef = useRef(null);

  const tFinal = 5.0;

  const sim = useMemo(() => {
    const dt = 0.002;
    const m = 1.0;
    // Surface passes through origin with normal n = (-sin(angle), cos(angle))
    // (tilted clockwise by surfaceAngle from horizontal)
    const nx = -Math.sin(surfaceAngle);
    const ny = Math.cos(surfaceAngle);
    const tx = Math.cos(surfaceAngle);  // tangent
    const ty = Math.sin(surfaceAngle);

    // State: [x, y, xdot, ydot, s] where s = arc-length along tangent (for tracking)
    const f = (t, st) => {
      const [x, y, xdot, ydot, s] = st;
      const v = [xdot, ydot];

      // Normal signed distance: d = n·p (since surface through origin)
      const d_n = nx * x + ny * y;
      // Penetration depth (positive when inside surface, d_n < 0)
      const pen = Math.max(0, -d_n);
      // Surface contact force (pushes along +n)
      const Fenv = Kenv * pen;
      const Fx_env = Fenv * nx;
      const Fy_env = Fenv * ny;

      // Tangent and normal velocity components
      const v_n = nx * xdot + ny * ydot;
      const v_t = tx * xdot + ty * ydot;

      // Force control along normal: we want to press INTO the surface (along -n).
      // When Fenv < Fd, push harder (more negative along n direction).
      // tau_n_scalar is the projection along +n; negative = push toward surface.
      const Ferr = Fd - Fenv;
      const tau_n_scalar = -Kf_n * Ferr - 5 * v_n;
      // Position/velocity control along tangent
      const tau_t_scalar = Kp_t * (vt_d - v_t);  // velocity control

      // Project back to Cartesian: tau = tau_n*n + tau_t*t
      const tau_x = tau_n_scalar * nx + tau_t_scalar * tx;
      const tau_y = tau_n_scalar * ny + tau_t_scalar * ty;

      const xddot = (tau_x + Fx_env) / m;
      const yddot = (tau_y + Fy_env) / m;
      const sdot = v_t;
      return [xdot, ydot, xddot, yddot, sdot];
    };
    // Start above surface, off to the left
    return simulate(f, [-1.0, 0.6, 0, 0, 0], 0, tFinal, dt);
  }, [surfaceAngle, Fd, vt_d, Kp_t, Kf_n, Kenv]);

  useEffect(() => {
    if (!running) return;
    let wallStart = performance.now();
    let tStart = time;
    const tick = () => {
      const e = (performance.now() - wallStart) / 1000;
      const nt = tStart + e;
      if (nt >= tFinal) {
        setTime(0); wallStart = performance.now(); tStart = 0;
      } else { setTime(nt); }
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  useEffect(() => { setTime(0); }, [surfaceAngle, Fd, vt_d, Kp_t, Kf_n]);

  const idx = Math.min(sim.length - 1, Math.floor(time / 0.002));
  const snap = sim[idx];
  const [x, y, xdot, ydot] = snap.x;

  const nx = -Math.sin(surfaceAngle);
  const ny = Math.cos(surfaceAngle);
  const tx = Math.cos(surfaceAngle);
  const ty = Math.sin(surfaceAngle);
  const d_n = nx * x + ny * y;
  const pen = Math.max(0, -d_n);
  const Fenv = Kenv * pen;
  const v_n_now = nx * xdot + ny * ydot;
  const v_t_now = tx * xdot + ty * ydot;

  // Plots
  const FnSeries = { name: 'F_normal', color: 'var(--warn)', width: 2,
    points: sim.map(s => {
      const [xs, ys] = [s.x[0], s.x[1]];
      const d = nx * xs + ny * ys;
      return [s.t, Math.max(0, -d) * Kenv];
    }) };
  const FdSeries = { name: 'Fd', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, Fd]) };
  const vtSeries = { name: 'v_tangent', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, tx * s.x[2] + ty * s.x[3]]) };
  const vtdSeries = { name: 'v_t target', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, vt_d]) };

  // SVG
  const W = 460, H = 280;
  const scale = 90;
  const ox = W / 2, oy = H / 2;
  const toS = (xx, yy) => [ox + xx * scale, oy - yy * scale];
  const [px, py] = toS(x, y);
  // Surface line endpoints
  const surfLen = 2.5;
  const [sax, say] = toS(-surfLen * tx, -surfLen * ty);
  const [sbx, sby] = toS(surfLen * tx, surfLen * ty);

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Constrained motion: sliding on a tilted surface</h3>
        <p className="eq">v = v_t + v_n &nbsp;·&nbsp; tangent: position controlled · normal: force controlled</p>

        <Explainer
          title="What is constrained motion?"
          plain="When a robot touches a surface, its world splits in two: directions ALONG the surface (it can slide freely) and the direction INTO the surface (it can't go through). A smart controller treats these separately. Along the surface: control position/velocity normally. Perpendicular: regulate the contact force. This is exactly what writing on a whiteboard feels like — you control where the pen moves (tangent) and how hard you press (normal). The selection matrix from Ch. 9 generalizes to ANY surface direction once you know its normal vector n̂."
          equation="v = (n̂·v)n̂ + (v − (n̂·v)n̂),    tangent: ‖, normal: ⊥"
          equationNote="Siciliano §13.2. The constraint manifold's tangent space carries position-controlled DOFs; the normal space carries force-controlled DOFs. This decomposition holds for any smooth contact constraint, not just flat surfaces."
          knobs={[
            { name: 'surface angle', what: 'tilts the surface — the normal direction changes accordingly' },
            { name: 'F_d normal', what: 'how hard to press into the surface' },
            { name: 'v_t target', what: 'how fast to slide along the surface' },
            { name: 'K_p tangent', what: 'gain for velocity tracking on the surface' },
            { name: 'K_f normal', what: 'gain for force regulation perpendicular to surface' }
          ]} />

        <svg width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
          {/* Surface line */}
          <line x1={sax} y1={say} x2={sbx} y2={sby}
            stroke="var(--danger)" strokeWidth="2" />
          {/* Hatching below */}
          {[...Array(24)].map((_, i) => {
            const u = -surfLen + (i / 24) * 2 * surfLen;
            const [bx, by] = toS(u * tx, u * ty);
            const [hx, hy] = toS(u * tx + 0.15 * nx * -1, u * ty + 0.15 * ny * -1);
            return <line key={i} x1={bx} y1={by} x2={hx} y2={hy}
              stroke="var(--danger)" strokeWidth="0.5" />;
          })}

          {/* Normal direction at contact (drawn from peg) */}
          {pen > 0.001 && (() => {
            const [tx2, ty2] = toS(x + 0.4 * nx, y + 0.4 * ny);
            return (
              <g>
                <line x1={px} y1={py} x2={tx2} y2={ty2}
                  stroke="var(--warn)" strokeWidth="1.5" markerEnd="url(#arr-n)" />
                <text x={tx2 + 6} y={ty2} fill="var(--warn)" fontSize="10" fontFamily="var(--mono)">
                  F_n = {Fenv.toFixed(1)}
                </text>
              </g>
            );
          })()}
          {/* Tangent direction */}
          {(() => {
            const [tex, tey] = toS(x + 0.4 * tx, y + 0.4 * ty);
            return (
              <g>
                <line x1={px} y1={py} x2={tex} y2={tey}
                  stroke="var(--accent-2)" strokeWidth="1.5" strokeDasharray="2 2" markerEnd="url(#arr-t)" />
                <text x={tex + 4} y={tey + 12} fill="var(--accent-2)" fontSize="10" fontFamily="var(--mono)">
                  v_t = {v_t_now.toFixed(2)}
                </text>
              </g>
            );
          })()}

          <defs>
            <marker id="arr-n" viewBox="0 0 10 10" refX="8" refY="5"
              markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M2 1L8 5L2 9" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
            </marker>
            <marker id="arr-t" viewBox="0 0 10 10" refX="8" refY="5"
              markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M2 1L8 5L2 9" fill="none" stroke="var(--accent-2)" strokeWidth="1.5" />
            </marker>
          </defs>

          {/* Peg */}
          <circle cx={px} cy={py} r="8" fill="var(--plot-1)" stroke="var(--text)" strokeWidth="1" />

          <text x={10} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            t = {time.toFixed(2)}s · contact = {pen > 0.001 ? 'YES' : 'no'}
          </text>
        </svg>

        <div style={{ marginTop: 12, display: 'grid', gap: 4 }}>
          <Plot
            series={[FnSeries, FdSeries]}
            xLabel=""
            yLabel="F_n (N)"
            xRange={[0, tFinal]}
            width={640}
            height={130}
            vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
          />
          <Plot
            series={[vtSeries, vtdSeries]}
            xLabel="t (s)"
            yLabel="v_t (m/s)"
            xRange={[0, tFinal]}
            width={640}
            height={130}
            vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
          />
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">F_n now</div><div className="value">{Fenv.toFixed(1)}N</div></div>
          <div className="metric"><div className="label">v_t now</div><div className="value">{v_t_now.toFixed(2)}</div></div>
          <div className="metric"><div className="label">v_n now</div><div className="value">{v_n_now.toFixed(3)}</div></div>
          <div className="metric"><div className="label">contact</div><div className="value">{pen > 0.001 ? 'yes' : 'no'}</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Play'}
          </button>
        </div>
      </div>

      <div className="controls">
        <h3>Surface</h3>
        <Slider label="angle (deg)" value={surfaceAngle * 180 / Math.PI}
          onChange={v => setSurfaceAngle(v * Math.PI / 180)}
          min={-45} max={45} step={1} precision={0} />
        <Slider label="K_env stiffness" value={Kenv} onChange={setKenv} min={100} max={3000} step={20} precision={0} />

        <h3 style={{ marginTop: 16 }}>Normal (force)</h3>
        <Slider label="F_d (N)" value={Fd} onChange={setFd} min={0} max={25} step={0.2} precision={1} />
        <Slider label="K_f gain" value={Kf_n} onChange={setKfN} min={0.1} max={50} step={0.1} precision={2} />

        <h3 style={{ marginTop: 16 }}>Tangent (motion)</h3>
        <Slider label="v_t target" value={vt_d} onChange={setVtD} min={-1} max={1} step={0.02} />
        <Slider label="K_p gain" value={Kp_t} onChange={setKpT} min={1} max={200} step={1} precision={0} />
      </div>
    </div>
  );
}
