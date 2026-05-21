import React, { useState, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano §11.2.1: Differential-drive mobile robot.
 *
 *  Two independently driven wheels separated by track width L.
 *  Wheel velocities (linear): v_R (right), v_L (left)
 *  Body twist:
 *    v = (v_R + v_L) / 2          (forward speed)
 *    ω = (v_R − v_L) / L          (angular speed)
 *
 *  Inverse:
 *    v_R = v + (ω L) / 2
 *    v_L = v − (ω L) / 2
 *
 *  Kinematics same as unicycle:
 *    ẋ = v cos(θ),  ẏ = v sin(θ),  θ̇ = ω
 *
 *  Pure-pursuit-style trajectory tracking (§11.5.2):
 *    Pick a look-ahead point on the reference; steer toward it.
 *    Generates feasible curvature for a desired path radius.
 */

export default function DiffDrive() {
  // Robot state
  const [pose, setPose] = useState({ x: -1, y: 0, th: 0.3 });
  const [vL, setVL] = useState(0);
  const [vR, setVR] = useState(0);
  const [L, setL] = useState(0.4);   // track width
  const [wheelR, setWheelR] = useState(0.08);

  // Manual mode vs tracking mode
  const [mode, setMode] = useState('manual'); // 'manual' | 'figure8'
  const [running, setRunning] = useState(false);
  const [Klook, setKlook] = useState(2.0);

  const [trail, setTrail] = useState([]);
  const animRef = useRef(null);
  const poseRef = useRef(pose);
  useEffect(() => { poseRef.current = pose; }, [pose]);
  const startTimeRef = useRef(0);

  useEffect(() => {
    if (!running) return;
    const dt = 0.04;
    startTimeRef.current = performance.now();
    const tick = () => {
      const p = poseRef.current;
      let v, w;

      if (mode === 'manual') {
        v = (vR + vL) / 2;
        w = (vR - vL) / L;
      } else {
        // Figure-8 tracking via pure pursuit
        const t = (performance.now() - startTimeRef.current) / 1000;
        // Reference path: lemniscate
        const a = 1.5;
        const phase = 0.4 * t;
        const refX = a * Math.cos(phase) / (1 + Math.sin(phase) ** 2);
        const refY = a * Math.cos(phase) * Math.sin(phase) / (1 + Math.sin(phase) ** 2);
        // Look-ahead: aim at point at small future phase
        const aheadPhase = phase + 0.3;
        const aheadX = a * Math.cos(aheadPhase) / (1 + Math.sin(aheadPhase) ** 2);
        const aheadY = a * Math.cos(aheadPhase) * Math.sin(aheadPhase) / (1 + Math.sin(aheadPhase) ** 2);

        const dx = aheadX - p.x;
        const dy = aheadY - p.y;
        let alpha = Math.atan2(dy, dx) - p.th;
        while (alpha > Math.PI) alpha -= 2 * Math.PI;
        while (alpha < -Math.PI) alpha += 2 * Math.PI;

        v = 0.7;
        w = Klook * alpha;
        // Update wheel velocities to match (for display)
        setVR(v + (w * L) / 2);
        setVL(v - (w * L) / 2);
      }

      const newPose = {
        x: p.x + v * Math.cos(p.th) * dt,
        y: p.y + v * Math.sin(p.th) * dt,
        th: p.th + w * dt
      };
      setPose(newPose);
      setTrail(t => {
        const next = [...t, [newPose.x, newPose.y]];
        if (next.length > 500) next.shift();
        return next;
      });

      animRef.current = setTimeout(tick, dt * 1000);
    };
    animRef.current = setTimeout(tick, 30);
    return () => clearTimeout(animRef.current);
  }, [running, mode, vL, vR, L, Klook]);

  // Visualization
  const W = 460, H = 360;
  const scale = 65;
  const ox = W / 2, oy = H / 2;
  const toS = (x, y) => [ox + x * scale, oy - y * scale];
  const [px, py] = toS(pose.x, pose.y);

  // Body rectangle (small car)
  const bodyL = 0.28, bodyW = L;
  const corners = [
    [-bodyL / 2, -bodyW / 2], [bodyL / 2, -bodyW / 2],
    [bodyL / 2, bodyW / 2], [-bodyL / 2, bodyW / 2]
  ].map(([lx, ly]) => {
    const c = Math.cos(pose.th), s = Math.sin(pose.th);
    return [pose.x + c * lx - s * ly, pose.y + s * lx + c * ly];
  });

  // Body-frame velocity arrow
  const arrowLen = 0.5;
  const v = (vR + vL) / 2;
  const w = (vR - vL) / L;
  const [ax, ay] = toS(pose.x + arrowLen * Math.cos(pose.th), pose.y + arrowLen * Math.sin(pose.th));

  // Reference path for figure-8 mode
  const refPathPoints = [];
  if (mode === 'figure8') {
    for (let i = 0; i <= 100; i++) {
      const phase = (i / 100) * 2 * Math.PI;
      const a = 1.5;
      const rx = a * Math.cos(phase) / (1 + Math.sin(phase) ** 2);
      const ry = a * Math.cos(phase) * Math.sin(phase) / (1 + Math.sin(phase) ** 2);
      refPathPoints.push([rx, ry]);
    }
  }

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Differential drive — two wheels, no caster needed</h3>
        <p className="eq">v = (v_R + v_L)/2,  ω = (v_R − v_L)/L</p>

        <Explainer
          title="How does two-wheel steering work?"
          plain="Picture a tank or a Roomba. Two driven wheels, one on each side. Spin both forward at the same speed: you go straight. Spin them in OPPOSITE directions: you turn in place. Spin one faster than the other: you curve. That's it. The math: average the wheel speeds to get how fast you're going forward; subtract them and divide by the wheel separation to get how fast you're turning. Wider wheelbase L means a slower turn rate for the same wheel-speed difference."
          equation="v = (v_R + v_L)/2,   ω = (v_R − v_L)/L"
          equationNote="Siciliano eq. 11.10. Inverse: v_R = v + ωL/2, v_L = v − ωL/2. The differential drive's instantaneous center of rotation is on the line through the two wheels, distance v/ω from the body center."
          knobs={[
            { name: 'v_R, v_L', what: 'left and right wheel linear speeds (manual mode)' },
            { name: 'L', what: 'track width (wheel separation) — wider = more stable, slower to turn' },
            { name: 'Mode', what: 'manual = drive wheels directly; figure-8 = track a lemniscate path' },
            { name: 'K_look', what: 'pure-pursuit gain — how aggressively to chase the look-ahead point' }
          ]} />

        <svg width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
          {/* Grid */}
          {[-3, -2, -1, 0, 1, 2, 3].map(g => (
            <g key={g}>
              <line x1={ox + g * scale} y1={0} x2={ox + g * scale} y2={H}
                stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={0} y1={oy + g * scale} x2={W} y2={oy + g * scale}
                stroke="var(--grid)" strokeWidth="0.5" />
            </g>
          ))}

          {/* Figure-8 reference */}
          {mode === 'figure8' && refPathPoints.length > 0 && (
            <path
              d={refPathPoints.map((p, i) => {
                const [sx, sy] = toS(p[0], p[1]);
                return `${i === 0 ? 'M' : 'L'} ${sx} ${sy}`;
              }).join(' ')}
              fill="none" stroke="var(--plot-2)" strokeWidth="1.5" strokeDasharray="4 4" />
          )}

          {/* Trail */}
          {trail.length > 1 && (
            <path
              d={trail.map((p, i) => {
                const [tx, ty] = toS(p[0], p[1]);
                return `${i === 0 ? 'M' : 'L'} ${tx} ${ty}`;
              }).join(' ')}
              fill="none" stroke="var(--accent-2)" strokeWidth="1" opacity="0.6" />
          )}

          {/* Robot body */}
          <polygon
            points={corners.map(c => toS(c[0], c[1]).join(',')).join(' ')}
            fill="var(--plot-1)" fillOpacity="0.85" stroke="var(--text)" strokeWidth="1" />

          {/* Wheels */}
          {(() => {
            const c = Math.cos(pose.th), s = Math.sin(pose.th);
            // Left wheel: body-frame (0, +L/2)
            const lwx = pose.x - s * (L / 2);
            const lwy = pose.y + c * (L / 2);
            const [lwsx, lwsy] = toS(lwx, lwy);
            const rwx = pose.x + s * (L / 2);
            const rwy = pose.y - c * (L / 2);
            const [rwsx, rwsy] = toS(rwx, rwy);
            const wheelColorL = Math.abs(vL) > 0.05 ? 'var(--accent-2)' : 'var(--text-faint)';
            const wheelColorR = Math.abs(vR) > 0.05 ? 'var(--accent-2)' : 'var(--text-faint)';
            return (
              <g>
                <rect x={lwsx - 8} y={lwsy - 4} width={16} height={8}
                  fill={wheelColorL} transform={`rotate(${-pose.th * 180 / Math.PI} ${lwsx} ${lwsy})`} />
                <rect x={rwsx - 8} y={rwsy - 4} width={16} height={8}
                  fill={wheelColorR} transform={`rotate(${-pose.th * 180 / Math.PI} ${rwsx} ${rwsy})`} />
              </g>
            );
          })()}

          {/* Velocity arrow */}
          {Math.abs(v) > 0.05 && (
            <line x1={px} y1={py} x2={ax} y2={ay}
              stroke="var(--warn)" strokeWidth="2" markerEnd="url(#arr-dd)" />
          )}
          <defs>
            <marker id="arr-dd" viewBox="0 0 10 10" refX="8" refY="5"
              markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M2 1L8 5L2 9" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
            </marker>
          </defs>

          <text x={10} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            v = {v.toFixed(2)} m/s | ω = {w.toFixed(2)} rad/s
          </text>
        </svg>

        <div className="metrics">
          <div className="metric"><div className="label">v_L</div><div className="value">{vL.toFixed(2)}</div></div>
          <div className="metric"><div className="label">v_R</div><div className="value">{vR.toFixed(2)}</div></div>
          <div className="metric"><div className="label">v</div><div className="value">{v.toFixed(2)}</div></div>
          <div className="metric"><div className="label">ω</div><div className="value">{w.toFixed(2)}</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Drive'}
          </button>
          <button className="btn" onClick={() => {
            setRunning(false);
            setPose({ x: -1, y: 0, th: 0.3 });
            setTrail([]);
          }}>Reset</button>
          <button className="btn" onClick={() => setTrail([])}>Clear trail</button>
        </div>
      </div>

      <div className="controls">
        <h3>Mode</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${mode === 'manual' ? 'primary' : ''}`}
            onClick={() => { setMode('manual'); setTrail([]); }}>Manual wheels</button>
          <button className={`btn ${mode === 'figure8' ? 'primary' : ''}`}
            onClick={() => { setMode('figure8'); setTrail([]); }}>Figure-8</button>
        </div>

        {mode === 'manual' && (
          <>
            <h3>Wheel velocities</h3>
            <Slider label="v_L (left)" value={vL} onChange={setVL} min={-1.5} max={1.5} step={0.02} />
            <Slider label="v_R (right)" value={vR} onChange={setVR} min={-1.5} max={1.5} step={0.02} />
            <div className="btn-row" style={{ marginTop: 8 }}>
              <button className="btn" onClick={() => { setVL(0.5); setVR(0.5); }}>Straight</button>
              <button className="btn" onClick={() => { setVL(-0.5); setVR(0.5); }}>Spin L</button>
              <button className="btn" onClick={() => { setVL(0.5); setVR(-0.5); }}>Spin R</button>
              <button className="btn" onClick={() => { setVL(0); setVR(0); }}>Stop</button>
            </div>
          </>
        )}

        {mode === 'figure8' && (
          <>
            <h3>Pure-pursuit</h3>
            <Slider label="K_look" value={Klook} onChange={setKlook} min={0.5} max={8} step={0.1} precision={1} />
          </>
        )}

        <h3 style={{ marginTop: 16 }}>Geometry</h3>
        <Slider label="L track width" value={L} onChange={setL} min={0.15} max={0.8} step={0.01} />
      </div>
    </div>
  );
}
