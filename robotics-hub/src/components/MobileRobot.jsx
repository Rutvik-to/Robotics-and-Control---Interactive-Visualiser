import React, { useState, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano Ch. 11: Wheeled mobile robots — unicycle model.
 *
 *  Kinematic model (eq. 11.13):
 *    ẋ = v cos(θ)
 *    ẏ = v sin(θ)
 *    θ̇ = ω
 *
 *  Nonholonomic constraint: ẋ sin(θ) − ẏ cos(θ) = 0  (can't slide sideways).
 *
 *  Posture regulation (drive to a target point) — Lyapunov approach (§11.6.1):
 *    Define:
 *      ρ = √((xd−x)² + (yd−y)²)     (distance to target)
 *      α = atan2(yd−y, xd−x) − θ    (heading error)
 *      β = -atan2(yd-y, xd-x)        (target orientation)
 *    Control law:
 *      v = Kρ · ρ · cos(α)
 *      ω = Kα · α + Kρ · sin(α) cos(α)   (simplified — drops β term for clarity)
 *
 *  Path following (§11.5.2): track a circular reference path.
 */

export default function MobileRobot() {
  const [mode, setMode] = useState('point'); // 'point' | 'circle'
  const [pose, setPose] = useState({ x: -2, y: -1, th: 0.5 });
  const [target, setTarget] = useState({ x: 1.5, y: 1.2 });
  const [Kr, setKr] = useState(0.6);   // distance gain
  const [Ka, setKa] = useState(2.0);   // heading gain
  const [vmax, setVmax] = useState(1.0);
  const [omax, setOmax] = useState(2.0);
  const [running, setRunning] = useState(false);
  const [trail, setTrail] = useState([]);
  const [circleR, setCircleR] = useState(1.5);
  const [pathV, setPathV] = useState(0.7);
  const animRef = useRef(null);
  const poseRef = useRef(pose);
  useEffect(() => { poseRef.current = pose; }, [pose]);

  const svgRef = useRef(null);

  // Animation
  useEffect(() => {
    if (!running) return;
    const dt = 0.04;
    const tick = () => {
      const p = poseRef.current;
      let v, w;

      if (mode === 'point') {
        // Posture regulation
        const dx = target.x - p.x;
        const dy = target.y - p.y;
        const rho = Math.sqrt(dx * dx + dy * dy);
        if (rho < 0.05) {
          v = 0; w = 0;
          setRunning(false);
        } else {
          let alpha = Math.atan2(dy, dx) - p.th;
          // Wrap to [-π, π]
          while (alpha > Math.PI) alpha -= 2 * Math.PI;
          while (alpha < -Math.PI) alpha += 2 * Math.PI;
          v = Kr * rho * Math.cos(alpha);
          w = Ka * alpha + Kr * Math.sin(alpha) * Math.cos(alpha);
        }
      } else {
        // Circular path following: target point ahead on circle
        // Find closest point on circle, then aim slightly ahead
        const r = Math.sqrt(p.x * p.x + p.y * p.y);
        const phi = Math.atan2(p.y, p.x);
        // Closest point on circle
        const closestX = circleR * Math.cos(phi);
        const closestY = circleR * Math.sin(phi);
        // Look-ahead: 0.3 rad along circle (counter-clockwise)
        const aheadPhi = phi + 0.3;
        const tgtX = circleR * Math.cos(aheadPhi);
        const tgtY = circleR * Math.sin(aheadPhi);
        const dx = tgtX - p.x, dy = tgtY - p.y;
        const rho = Math.sqrt(dx * dx + dy * dy);
        let alpha = Math.atan2(dy, dx) - p.th;
        while (alpha > Math.PI) alpha -= 2 * Math.PI;
        while (alpha < -Math.PI) alpha += 2 * Math.PI;
        v = pathV;
        w = Ka * alpha;
      }

      // Saturate
      v = Math.max(-vmax, Math.min(vmax, v));
      w = Math.max(-omax, Math.min(omax, w));

      // Integrate unicycle
      const newPose = {
        x: p.x + v * Math.cos(p.th) * dt,
        y: p.y + v * Math.sin(p.th) * dt,
        th: p.th + w * dt
      };
      setPose(newPose);
      setTrail(t => {
        const next = [...t, [newPose.x, newPose.y]];
        if (next.length > 400) next.shift();
        return next;
      });

      animRef.current = setTimeout(tick, dt * 1000);
    };
    animRef.current = setTimeout(tick, 30);
    return () => clearTimeout(animRef.current);
  }, [running, mode, target, Kr, Ka, vmax, omax, circleR, pathV]);

  // Click to set target (point mode)
  const handleClick = (e) => {
    if (mode !== 'point') return;
    const rect = svgRef.current.getBoundingClientRect();
    const W = 460, H = 360;
    const scale = 60;
    const ox = W / 2, oy = H / 2;
    const px = ((e.clientX - rect.left) / rect.width) * W;
    const py = ((e.clientY - rect.top) / rect.height) * H;
    const wx = (px - ox) / scale;
    const wy = -(py - oy) / scale;
    setTarget({ x: wx, y: wy });
  };

  // SVG
  const W = 460, H = 360;
  const ox = W / 2, oy = H / 2;
  const scale = 60;
  const toS = (x, y) => [ox + x * scale, oy - y * scale];
  const [px, py] = toS(pose.x, pose.y);
  const [tx, ty] = toS(target.x, target.y);

  // Heading arrow
  const ahead = 0.35;
  const [ax, ay] = toS(pose.x + ahead * Math.cos(pose.th), pose.y + ahead * Math.sin(pose.th));

  // Distance / heading error displayed
  const dx = target.x - pose.x, dy = target.y - pose.y;
  const rho = Math.sqrt(dx * dx + dy * dy);
  let alpha = Math.atan2(dy, dx) - pose.th;
  while (alpha > Math.PI) alpha -= 2 * Math.PI;
  while (alpha < -Math.PI) alpha += 2 * Math.PI;

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Unicycle mobile robot — drive to target / follow path</h3>
        <p className="eq">ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = ω  &nbsp;|&nbsp; nonholonomic: no sideways slip</p>

        <Explainer
          title="What's a unicycle robot?"
          plain="Think of a Roomba or a tank: it can move forward/backward at speed v and turn at rate ω, but it CANNOT slide sideways. To reach a point off to the right, it must first turn, then drive. That 'can't move sideways' rule is called a nonholonomic constraint, and it's why parallel parking is hard. The controller looks at the distance ρ and angle α to the target, then decides how much to drive vs how much to turn."
          equation="v = Kρ · ρ · cos(α),    ω = Kα · α"
          equationNote="Siciliano §11.6.1. The robot drives toward the target with effort proportional to distance, while simultaneously rotating to face it. cos(α) ensures it doesn't drive backward when facing away — it turns around first."
          knobs={[
            { name: 'Click', what: 'sets a new target on the workspace (point mode)' },
            { name: 'Kρ', what: 'forward-drive aggressiveness — higher = faster approach' },
            { name: 'Kα', what: 'turning aggressiveness — higher = sharper steering' },
            { name: 'vmax / ωmax', what: 'physical limits on the robot\'s motors' },
            { name: 'Mode', what: 'point: drive to a fixed target; circle: track a circular path forever' }
          ]} />

        <svg ref={svgRef} width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)', cursor: mode === 'point' ? 'crosshair' : 'default' }}
          onClick={handleClick}>
          {/* Grid */}
          {[-3, -2, -1, 0, 1, 2, 3].map(g => (
            <g key={g}>
              <line x1={ox + g * scale} y1={0} x2={ox + g * scale} y2={H} stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={0} y1={oy + g * scale} x2={W} y2={oy + g * scale} stroke="var(--grid)" strokeWidth="0.5" />
            </g>
          ))}

          {/* Circular path */}
          {mode === 'circle' && (
            <circle cx={ox} cy={oy} r={circleR * scale}
              fill="none" stroke="var(--plot-2)" strokeWidth="1.5" strokeDasharray="6 4" />
          )}

          {/* Trail */}
          {trail.length > 1 && (
            <path
              d={trail.map((p, i) => {
                const [tx2, ty2] = toS(p[0], p[1]);
                return `${i === 0 ? 'M' : 'L'} ${tx2} ${ty2}`;
              }).join(' ')}
              fill="none" stroke="var(--accent-2)" strokeWidth="1" opacity="0.7" />
          )}

          {/* Target (point mode) */}
          {mode === 'point' && (
            <g>
              <circle cx={tx} cy={ty} r="8" fill="none" stroke="var(--warn)" strokeWidth="1.5" strokeDasharray="3 2" />
              <line x1={tx - 6} y1={ty} x2={tx + 6} y2={ty} stroke="var(--warn)" strokeWidth="1.5" />
              <line x1={tx} y1={ty - 6} x2={tx} y2={ty + 6} stroke="var(--warn)" strokeWidth="1.5" />
            </g>
          )}

          {/* Robot body */}
          <g>
            <circle cx={px} cy={py} r="10" fill="var(--plot-1)" />
            <line x1={px} y1={py} x2={ax} y2={ay} stroke="var(--text)" strokeWidth="2" />
            <circle cx={ax} cy={ay} r="3" fill="var(--text)" />
          </g>

          <text x={10} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            pose: ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}, {(pose.th * 180 / Math.PI).toFixed(0)}°)
          </text>
          {mode === 'point' && (
            <text x={10} y={28} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
              click to retarget
            </text>
          )}
        </svg>

        <div className="metrics">
          <div className="metric"><div className="label">distance ρ</div><div className="value">{rho.toFixed(2)}</div></div>
          <div className="metric"><div className="label">heading α</div><div className="value">{(alpha * 180 / Math.PI).toFixed(0)}°</div></div>
          <div className="metric"><div className="label">θ</div><div className="value">{(pose.th * 180 / Math.PI).toFixed(0)}°</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Drive'}
          </button>
          <button className="btn" onClick={() => {
            setRunning(false);
            setPose({ x: -2, y: -1, th: 0.5 });
            setTrail([]);
          }}>Reset</button>
          <button className="btn" onClick={() => setTrail([])}>Clear trail</button>
        </div>
      </div>

      <div className="controls">
        <h3>Mode</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${mode === 'point' ? 'primary' : ''}`}
            onClick={() => { setMode('point'); setTrail([]); }}>To point</button>
          <button className={`btn ${mode === 'circle' ? 'primary' : ''}`}
            onClick={() => { setMode('circle'); setTrail([]); }}>Follow circle</button>
        </div>

        <h3>Control gains</h3>
        <Slider label="Kρ (drive)" value={Kr} onChange={setKr} min={0.1} max={3} step={0.05} />
        <Slider label="Kα (turn)" value={Ka} onChange={setKa} min={0.5} max={8} step={0.1} precision={1} />

        <h3 style={{ marginTop: 16 }}>Limits</h3>
        <Slider label="v_max" value={vmax} onChange={setVmax} min={0.2} max={3} step={0.05} />
        <Slider label="ω_max" value={omax} onChange={setOmax} min={0.5} max={5} step={0.05} />

        {mode === 'circle' && (
          <>
            <h3 style={{ marginTop: 16 }}>Path</h3>
            <Slider label="radius" value={circleR} onChange={setCircleR} min={0.5} max={2.8} step={0.05} />
            <Slider label="speed" value={pathV} onChange={setPathV} min={0.2} max={2} step={0.05} />
          </>
        )}
      </div>
    </div>
  );
}
