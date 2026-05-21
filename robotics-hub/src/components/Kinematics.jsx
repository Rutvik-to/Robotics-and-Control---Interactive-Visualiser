import React, { useState, useMemo, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * 2-link planar arm kinematics (Siciliano Ch. 2 & 3).
 *
 * Forward:  pe = [L1 cos(q1) + L2 cos(q1+q2), L1 sin(q1) + L2 sin(q1+q2)]
 *
 * Jacobian: J = [[-L1 s1 - L2 s12, -L2 s12],
 *                [ L1 c1 + L2 c12,  L2 c12]]
 *
 * Inverse: q2 = atan2(±√(1-c2²), c2),  c2 = (px² + py² − L1² − L2²)/(2 L1 L2)
 *
 * Manipulability ellipse: SVD of J — axes = singular values, directions = U columns.
 */
export default function Kinematics() {
  const [L1, setL1] = useState(1.2);
  const [L2, setL2] = useState(1.0);
  const [q1, setQ1] = useState(0.5);
  const [q2, setQ2] = useState(1.0);
  const [mode, setMode] = useState('fk'); // 'fk' | 'ik'
  const [elbow, setElbow] = useState('up'); // 'up' | 'down' (IK branch)
  const [target, setTarget] = useState({ x: 1.5, y: 0.8 });
  const [showEllipse, setShowEllipse] = useState(true);
  const svgRef = useRef(null);

  // Forward kinematics
  const fk = (a, b) => {
    const x1 = L1 * Math.cos(a);
    const y1 = L1 * Math.sin(a);
    const xe = x1 + L2 * Math.cos(a + b);
    const ye = y1 + L2 * Math.sin(a + b);
    return { x1, y1, xe, ye };
  };

  // Inverse kinematics (returns {q1, q2} or null)
  const ik = (px, py, elb) => {
    const r2 = px * px + py * py;
    const c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (c2 > 1 || c2 < -1) return null; // unreachable
    const s2 = elb === 'up' ? Math.sqrt(1 - c2 * c2) : -Math.sqrt(1 - c2 * c2);
    const qq2 = Math.atan2(s2, c2);
    const qq1 = Math.atan2(py, px) - Math.atan2(L2 * s2, L1 + L2 * c2);
    return { q1: qq1, q2: qq2 };
  };

  // If in IK mode, derive q1/q2 from target
  const angles = useMemo(() => {
    if (mode === 'ik') {
      const sol = ik(target.x, target.y, elbow);
      if (sol) return sol;
      return { q1, q2 }; // fall back
    }
    return { q1, q2 };
  }, [mode, target, elbow, L1, L2, q1, q2]);

  const reachable = mode === 'ik' ? ik(target.x, target.y, elbow) !== null : true;
  const pose = fk(angles.q1, angles.q2);

  // Jacobian at current pose
  const s1 = Math.sin(angles.q1), c1 = Math.cos(angles.q1);
  const s12 = Math.sin(angles.q1 + angles.q2), c12 = Math.cos(angles.q1 + angles.q2);
  const J = [
    [-L1 * s1 - L2 * s12, -L2 * s12],
    [L1 * c1 + L2 * c12, L2 * c12]
  ];
  const detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0];
  const manip = Math.abs(detJ); // scalar manipulability index
  const nearSingular = manip < 0.05;

  // Eigen-decomp of J*J^T for manipulability ellipse
  const JJT = [
    [J[0][0] ** 2 + J[0][1] ** 2, J[0][0] * J[1][0] + J[0][1] * J[1][1]],
    [J[0][0] * J[1][0] + J[0][1] * J[1][1], J[1][0] ** 2 + J[1][1] ** 2]
  ];
  const tr = JJT[0][0] + JJT[1][1];
  const det = JJT[0][0] * JJT[1][1] - JJT[0][1] * JJT[1][0];
  const disc = Math.max(0, tr * tr - 4 * det);
  const lam1 = (tr + Math.sqrt(disc)) / 2;
  const lam2 = (tr - Math.sqrt(disc)) / 2;
  const sigma1 = Math.sqrt(Math.max(0, lam1));
  const sigma2 = Math.sqrt(Math.max(0, lam2));
  // Eigenvector for lam1
  let ang;
  if (Math.abs(JJT[0][1]) > 1e-9) {
    ang = Math.atan2(lam1 - JJT[0][0], JJT[0][1]);
  } else {
    ang = lam1 >= JJT[0][0] ? 0 : Math.PI / 2;
  }

  // SVG scaling
  const maxReach = L1 + L2;
  const W = 460, H = 400;
  const ox = W / 2, oy = H / 2;
  const scale = Math.min(W, H) / (2 * maxReach * 1.15);
  const toSvg = (x, y) => [ox + x * scale, oy - y * scale];

  const handleClick = (e) => {
    if (mode !== 'ik') return;
    const rect = svgRef.current.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * W;
    const py = ((e.clientY - rect.top) / rect.height) * H;
    const wx = (px - ox) / scale;
    const wy = -(py - oy) / scale;
    setTarget({ x: wx, y: wy });
  };

  const [baseX, baseY] = toSvg(0, 0);
  const [j1X, j1Y] = toSvg(pose.x1, pose.y1);
  const [eX, eY] = toSvg(pose.xe, pose.ye);
  const [tX, tY] = toSvg(target.x, target.y);

  // Reach circles
  const reachOuter = maxReach * scale;
  const reachInner = Math.abs(L1 - L2) * scale;

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Planar 2-link manipulator</h3>
        <p className="eq">pe = [L₁c₁ + L₂c₁₂, L₁s₁ + L₂s₁₂] &nbsp;·&nbsp; J(q) ∈ ℝ²ˣ² &nbsp;·&nbsp; v = J(q) q̇</p>
        <Explainer
          title="Forward vs inverse kinematics"
          plain="Think of your arm: forward kinematics is 'if I bend my shoulder 30° and elbow 60°, where does my hand end up?' Easy — just add up where each link goes. Inverse kinematics is the reverse: 'I want my hand HERE, what angles do I need?' Harder — usually there are two solutions (elbow up or elbow down, like reaching over vs under). And sometimes the target is unreachable (your arm isn't long enough). The teal ellipse shows where the arm can move easily — long axis = fast motion direction, short axis = sluggish."
          equation="pe = [L₁cos(q₁) + L₂cos(q₁+q₂), L₁sin(q₁) + L₂sin(q₁+q₂)]"
          equationNote="Siciliano eq. 2.65. The Jacobian J(q) maps joint velocities q̇ to end-effector velocity v: v = J·q̇."
          knobs={[
            { name: 'Forward / Inverse', what: 'pick which mode; in IK, click anywhere to set a target' },
            { name: 'q₁, q₂', what: 'joint angles in FK mode — drives the arm directly' },
            { name: 'Elbow up / down', what: 'two valid IK solutions for the same target' },
            { name: 'L₁, L₂', what: 'link lengths — controls reach and singularity location' }
          ]} />
        <svg ref={svgRef} width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)', cursor: mode === 'ik' ? 'crosshair' : 'default' }}
          onClick={handleClick}>
          {/* Reachable workspace */}
          <circle cx={ox} cy={oy} r={reachOuter} fill="none"
            stroke="var(--border)" strokeWidth="0.5" strokeDasharray="3 4" />
          {reachInner > 1 && (
            <circle cx={ox} cy={oy} r={reachInner} fill="none"
              stroke="var(--border)" strokeWidth="0.5" strokeDasharray="3 4" />
          )}
          {/* Axes */}
          <line x1={0} y1={oy} x2={W} y2={oy} stroke="var(--grid)" strokeWidth="0.5" />
          <line x1={ox} y1={0} x2={ox} y2={H} stroke="var(--grid)" strokeWidth="0.5" />

          {/* Manipulability ellipse at end-effector */}
          {showEllipse && !nearSingular && (
            <ellipse cx={eX} cy={eY}
              rx={sigma1 * scale * 0.35}
              ry={sigma2 * scale * 0.35}
              fill="var(--plot-3)" fillOpacity="0.12"
              stroke="var(--plot-3)" strokeWidth="1" strokeDasharray="3 3"
              transform={`rotate(${-ang * 180 / Math.PI} ${eX} ${eY})`} />
          )}

          {/* Link 1 */}
          <line x1={baseX} y1={baseY} x2={j1X} y2={j1Y}
            stroke={nearSingular ? 'var(--danger)' : 'var(--plot-1)'} strokeWidth="5" strokeLinecap="round" />
          {/* Link 2 */}
          <line x1={j1X} y1={j1Y} x2={eX} y2={eY}
            stroke={nearSingular ? 'var(--danger)' : 'var(--accent-2)'} strokeWidth="5" strokeLinecap="round" />

          {/* Joints */}
          <circle cx={baseX} cy={baseY} r="7" fill="var(--bg)" stroke="var(--text)" strokeWidth="1.5" />
          <circle cx={j1X} cy={j1Y} r="6" fill="var(--bg-elev)" stroke="var(--text-dim)" strokeWidth="1.5" />
          <circle cx={eX} cy={eY} r="5" fill="var(--plot-2)" />

          {/* Target marker (IK mode) */}
          {mode === 'ik' && (
            <g>
              <circle cx={tX} cy={tY} r="9" fill="none"
                stroke={reachable ? 'var(--warn)' : 'var(--danger)'} strokeWidth="1.5" strokeDasharray="3 2" />
              <line x1={tX - 7} y1={tY} x2={tX + 7} y2={tY}
                stroke={reachable ? 'var(--warn)' : 'var(--danger)'} strokeWidth="1.5" />
              <line x1={tX} y1={tY - 7} x2={tX} y2={tY + 7}
                stroke={reachable ? 'var(--warn)' : 'var(--danger)'} strokeWidth="1.5" />
            </g>
          )}

          {/* Labels */}
          <text x={baseX + 10} y={baseY + 16} fill="var(--text-dim)" fontSize="11" fontFamily="var(--mono)">base</text>
          <text x={eX + 10} y={eY - 8} fill="var(--text)" fontSize="11" fontFamily="var(--mono)">
            ({pose.xe.toFixed(2)}, {pose.ye.toFixed(2)})
          </text>
        </svg>

        <div className="metrics">
          <div className="metric"><div className="label">q₁</div><div className="value">{(angles.q1 * 180 / Math.PI).toFixed(1)}°</div></div>
          <div className="metric"><div className="label">q₂</div><div className="value">{(angles.q2 * 180 / Math.PI).toFixed(1)}°</div></div>
          <div className="metric"><div className="label">det J</div><div className="value" style={{ color: nearSingular ? 'var(--danger)' : 'var(--text)' }}>{detJ.toFixed(3)}</div></div>
          <div className="metric"><div className="label">σ₁ / σ₂</div><div className="value">{(sigma1 / (sigma2 || 1e-9)).toFixed(2)}</div></div>
        </div>
        <p className="note">
          {mode === 'fk'
            ? <>Forward mode: drive joints, watch the end-effector trace through workspace. The teal ellipse is the
              <code>manipulability</code> shape — long axis is direction of easy motion, short axis is direction of difficulty.</>
            : <>Inverse mode: <strong>click anywhere</strong> in the workspace to set a target. The arm solves
              q₁, q₂ analytically. Switch elbow branch to see the two solutions. {!reachable && <span style={{ color: 'var(--danger)' }}>Target unreachable.</span>}</>}
          {nearSingular && <span style={{ color: 'var(--danger)' }}> Singular: J loses rank — straight-arm pose.</span>}
        </p>
      </div>

      <div className="controls">
        <h3>Mode</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${mode === 'fk' ? 'primary' : ''}`} onClick={() => setMode('fk')}>Forward</button>
          <button className={`btn ${mode === 'ik' ? 'primary' : ''}`} onClick={() => setMode('ik')}>Inverse</button>
        </div>

        {mode === 'fk' && (
          <>
            <Slider label="q₁ (rad)" value={q1} onChange={setQ1} min={-Math.PI} max={Math.PI} step={0.01} />
            <Slider label="q₂ (rad)" value={q2} onChange={setQ2} min={-Math.PI} max={Math.PI} step={0.01} />
          </>
        )}
        {mode === 'ik' && (
          <>
            <Slider label="target x" value={target.x} onChange={x => setTarget(t => ({ ...t, x }))} min={-3} max={3} step={0.01} />
            <Slider label="target y" value={target.y} onChange={y => setTarget(t => ({ ...t, y }))} min={-3} max={3} step={0.01} />
            <div className="btn-row">
              <button className={`btn ${elbow === 'up' ? 'primary' : ''}`} onClick={() => setElbow('up')}>Elbow up</button>
              <button className={`btn ${elbow === 'down' ? 'primary' : ''}`} onClick={() => setElbow('down')}>Elbow down</button>
            </div>
          </>
        )}

        <h3 style={{ marginTop: 20 }}>Links</h3>
        <Slider label="L₁" value={L1} onChange={setL1} min={0.3} max={2.0} step={0.05} />
        <Slider label="L₂" value={L2} onChange={setL2} min={0.3} max={2.0} step={0.05} />

        <div className="btn-row" style={{ marginTop: 16 }}>
          <button className={`btn ${showEllipse ? 'primary' : ''}`}
            onClick={() => setShowEllipse(!showEllipse)}>
            Manipulability: {showEllipse ? 'ON' : 'OFF'}
          </button>
        </div>
      </div>
    </div>
  );
}
