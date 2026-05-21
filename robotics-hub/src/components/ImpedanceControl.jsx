import React, { useState, useMemo, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Siciliano §9.5: Impedance control.
 *
 * Goal: make the end-effector behave like a virtual mass-spring-damper
 * relative to a desired position xd.
 *
 *  Desired impedance:   M_d (ẍ − ẍd) + D_d (ẋ − ẋd) + K_d (x − xd) = F_ext
 *
 * When you push the end-effector with external force F_ext, it deflects
 * by F_ext/K_d in steady state. Soft Kd = compliant (good for contact),
 * stiff Kd = positioning (good for free motion).
 *
 * Wall contact: a virtual wall at x = x_wall exerts F_ext = K_wall · max(0, x - x_wall).
 */

export default function ImpedanceControl() {
  const [Md, setMd] = useState(1.0);
  const [Dd, setDd] = useState(10);
  const [Kd, setKd] = useState(80);
  const [xd, setXd] = useState(1.0);   // desired position
  const [wallOn, setWallOn] = useState(true);
  const [xWall, setXWall] = useState(0.7);
  const [Kwall, setKwall] = useState(500);
  const [pushF, setPushF] = useState(0);   // manual external push
  const [time, setTime] = useState(0);
  const [running, setRunning] = useState(true);
  const animRef = useRef(null);

  const tFinal = 5.0;

  const sim = useMemo(() => {
    const dt = 0.002;
    // State: [x, xdot]
    const f = (t, x_) => {
      const [x, xdot] = x_;
      // External wall force (only when penetrating)
      let Fext = pushF;
      if (wallOn && x > xWall) {
        Fext -= Kwall * (x - xWall);
      }
      // Impedance dynamics: M_d ẍ + D_d ẋ + K_d (x − xd) = F_ext
      const xddot = (Fext - Dd * xdot - Kd * (x - xd)) / Md;
      return [xdot, xddot];
    };
    return simulate(f, [0, 0], 0, tFinal, dt);
  }, [Md, Dd, Kd, xd, wallOn, xWall, Kwall, pushF]);

  useEffect(() => {
    if (!running) return;
    let startWall = performance.now();
    let startT = time;
    const tick = () => {
      const elapsed = (performance.now() - startWall) / 1000;
      const newT = startT + elapsed;
      if (newT >= tFinal) {
        setTime(0);
        startWall = performance.now();
        startT = 0;
        animRef.current = requestAnimationFrame(tick);
        return;
      }
      setTime(newT);
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  useEffect(() => { setTime(0); }, [Md, Dd, Kd, xd, wallOn, xWall, Kwall, pushF]);

  // Snapshot
  const idx = Math.min(sim.length - 1, Math.floor(time / 0.002));
  const snap = sim[idx];
  const x_now = snap.x[0];
  const xdot_now = snap.x[1];

  // Wall contact force at current time
  const Fcontact = wallOn && x_now > xWall ? Kwall * (x_now - xWall) : 0;

  // Time series
  const xSeries = { name: 'x', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, s.x[0]]) };
  const xdSeries = { name: 'xd', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, xd]) };
  const wallSeries = wallOn ? { name: 'wall', color: 'var(--danger)', width: 1, dash: '2 3',
    points: sim.map(s => [s.t, xWall]) } : null;

  // Cartoon visualization
  const W = 460, H = 140;
  const xMin = -0.5, xMax = 2.0;
  const toPx = (x) => 30 + ((x - xMin) / (xMax - xMin)) * (W - 60);
  const xPx = toPx(x_now);
  const xdPx = toPx(xd);
  const wallPx = toPx(xWall);

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Impedance control — virtual spring-damper to setpoint</h3>
        <p className="eq">Mₐ ẍ + Dₐ ẋ + Kₐ (x − xd) = F_ext</p>

        <Explainer
          title="What is impedance control?"
          plain="Imagine the end-effector is connected to the goal position by a virtual rubber band (the spring Kₐ) and a virtual shock absorber (the damper Dₐ). Normal position control would say 'be exactly there or I'll fight you.' Impedance control says 'be there if nothing's in the way, but if you bump into something, give a little.' That's how robots can safely work near humans, polish surfaces, or insert pegs — they don't break things when they touch them."
          equation="Mₐ·ẍ + Dₐ·ẋ + Kₐ·(x − xd) = F_ext"
          equationNote="Siciliano eq. 9.30. Push the robot (F_ext) and it deflects by F_ext/Kₐ. Soft Kₐ = compliant. Stiff Kₐ = positioning."
          knobs={[
            { name: 'Kₐ stiffness', what: 'how hard the spring pulls back to xd — soft for contact, stiff for accuracy' },
            { name: 'Dₐ damping', what: 'how much it resists velocity — too low = bouncy, too high = sluggish' },
            { name: 'Mₐ virtual mass', what: 'apparent inertia — higher = feels heavier when you push it' },
            { name: 'Wall', what: 'turn on to simulate hitting a surface; contact force = K_wall × penetration' },
            { name: 'External push', what: 'apply a constant force — watch the steady-state deflection equal F/Kₐ' }
          ]} />

        {/* Cartoon */}
        <svg width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
          {/* Floor */}
          <line x1={20} y1={H - 30} x2={W - 20} y2={H - 30} stroke="var(--border)" strokeWidth="1" />

          {/* Spring from origin to ee */}
          {(() => {
            const x1 = toPx(0), x2 = xPx;
            const segs = 10;
            const amp = 8;
            let path = `M ${x1} ${H / 2}`;
            for (let i = 1; i <= segs; i++) {
              const sx = x1 + ((x2 - x1) * i) / segs;
              const sy = H / 2 + (i % 2 === 0 ? -amp : amp);
              path += ` L ${sx} ${sy}`;
            }
            path += ` L ${x2} ${H / 2}`;
            return <path d={path} stroke="var(--accent-2)" strokeWidth="1.5" fill="none" />;
          })()}

          {/* Desired position marker */}
          <line x1={xdPx} y1={H / 2 - 22} x2={xdPx} y2={H / 2 + 22}
            stroke="var(--plot-2)" strokeWidth="1" strokeDasharray="3 3" />
          <text x={xdPx} y={H / 2 - 26} textAnchor="middle" fill="var(--plot-2)"
            fontSize="10" fontFamily="var(--mono)">xd</text>

          {/* Wall */}
          {wallOn && (
            <g>
              <rect x={wallPx} y={H / 2 - 28} width={W - wallPx - 20} height={56}
                fill="var(--danger)" fillOpacity="0.12"
                stroke="var(--danger)" strokeWidth="1" strokeDasharray="2 3" />
              <text x={wallPx + 4} y={H / 2 - 12} fill="var(--danger)"
                fontSize="10" fontFamily="var(--mono)">wall</text>
            </g>
          )}

          {/* End-effector */}
          <rect x={xPx - 14} y={H / 2 - 12} width={28} height={24}
            fill="var(--plot-1)" rx="3" />
          <text x={xPx} y={H / 2 + 4} textAnchor="middle" fill="var(--bg)"
            fontSize="10" fontFamily="var(--mono)" fontWeight="500">EE</text>

          {/* External push arrow */}
          {Math.abs(pushF) > 0.5 && (
            <g>
              <line x1={xPx + (pushF > 0 ? -30 : 30)} y1={H / 2}
                x2={xPx + (pushF > 0 ? -15 : 15)} y2={H / 2}
                stroke="var(--warn)" strokeWidth="2"
                markerEnd="url(#arr-imp)" />
              <defs>
                <marker id="arr-imp" viewBox="0 0 10 10" refX="8" refY="5"
                  markerWidth="6" markerHeight="6" orient="auto-start-reverse">
                  <path d="M2 1L8 5L2 9" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
                </marker>
              </defs>
            </g>
          )}

          <text x={10} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            x = {x_now.toFixed(3)} | F_contact = {Fcontact.toFixed(1)} N
          </text>
        </svg>

        <Plot
          series={[xSeries, xdSeries, ...(wallSeries ? [wallSeries] : [])]}
          xLabel="t (s)"
          yLabel="x (m)"
          xRange={[0, tFinal]}
          width={640}
          height={170}
          vLines={[{ x: time, color: 'var(--accent)', dash: '4 3' }]}
        />

        <div className="metrics">
          <div className="metric"><div className="label">x now</div><div className="value">{x_now.toFixed(3)}</div></div>
          <div className="metric"><div className="label">deflection</div><div className="value">{(x_now - xd).toFixed(3)}</div></div>
          <div className="metric"><div className="label">F contact</div><div className="value">{Fcontact.toFixed(1)}N</div></div>
          <div className="metric"><div className="label">predicted SS</div><div className="value">{(pushF / Kd).toFixed(3)}</div></div>
        </div>
      </div>

      <div className="controls">
        <h3>Impedance</h3>
        <Slider label="Kₐ stiffness" value={Kd} onChange={setKd} min={1} max={500} step={1} precision={0} />
        <Slider label="Dₐ damping" value={Dd} onChange={setDd} min={0} max={80} step={0.5} precision={1} />
        <Slider label="Mₐ virtual mass" value={Md} onChange={setMd} min={0.1} max={5} step={0.05} />

        <h3 style={{ marginTop: 16 }}>Setpoint</h3>
        <Slider label="xd (m)" value={xd} onChange={setXd} min={-0.3} max={1.8} step={0.02} />

        <h3 style={{ marginTop: 16 }}>External force</h3>
        <Slider label="push F (N)" value={pushF} onChange={setPushF} min={-30} max={30} step={0.5} precision={1} />

        <h3 style={{ marginTop: 16 }}>Wall</h3>
        <div className="btn-row" style={{ marginBottom: 8 }}>
          <button className={`btn ${wallOn ? 'primary' : ''}`}
            onClick={() => setWallOn(!wallOn)}>Wall: {wallOn ? 'ON' : 'OFF'}</button>
        </div>
        {wallOn && (
          <>
            <Slider label="x_wall" value={xWall} onChange={setXWall} min={0.3} max={1.8} step={0.02} />
            <Slider label="K_wall" value={Kwall} onChange={setKwall} min={50} max={2000} step={10} precision={0} />
          </>
        )}

        <div className="btn-row" style={{ marginTop: 16 }}>
          <button className="btn" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Play'}
          </button>
        </div>
      </div>
    </div>
  );
}
