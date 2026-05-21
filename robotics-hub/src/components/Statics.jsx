import React, { useState, useMemo, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano Ch. 5: Differential kinematics & statics.
 *
 *  STATICS (§5.7):
 *    Duality between velocity and force domains.
 *      Velocity:   v_e = J(q) q̇
 *      Force:      τ = J(q)ᵀ F_e
 *    A wrench F_e applied at the end-effector maps to joint torques τ.
 *    Equivalently: a kinematic singularity (rank-deficient J) means there
 *    are directions in which the arm cannot resist external force.
 *
 *  FORCE ELLIPSOID:
 *    The set { F : |J⁻ᵀ τ| ≤ 1 with |τ| ≤ 1 } is an ellipsoid;
 *    its principal axes are reciprocal to the velocity manipulability ellipsoid.
 *    So directions of high velocity capability are directions of LOW force capability.
 *
 *  NEWTON-EULER (§7.5) — not implemented here as a recursive algorithm, but the
 *    underlying balance is: for each link, gravity + applied wrench − inertial
 *    reaction = sum of joint reactions. We expose the static balance at the
 *    end effector: τ = Jᵀ ( F_external + m_payload · g · ẑ ).
 */

export default function Statics() {
  const [L1, setL1] = useState(1.0);
  const [L2, setL2] = useState(0.8);
  const [q1, setQ1] = useState(0.6);
  const [q2, setQ2] = useState(1.0);
  const [Fx, setFx] = useState(0);
  const [Fy, setFy] = useState(-5);
  const [payload, setPayload] = useState(2.0);
  const [showForceEll, setShowForceEll] = useState(true);
  const [showVelEll, setShowVelEll] = useState(false);
  const svgRef = useRef(null);

  // Forward kinematics & Jacobian
  const c1 = Math.cos(q1), s1 = Math.sin(q1);
  const c12 = Math.cos(q1 + q2), s12 = Math.sin(q1 + q2);
  const x1 = L1 * c1, y1 = L1 * s1;
  const xe = x1 + L2 * c12, ye = y1 + L2 * s12;

  const J = [
    [-L1 * s1 - L2 * s12, -L2 * s12],
    [L1 * c1 + L2 * c12, L2 * c12]
  ];
  const detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0];

  // Total external force (user-applied + payload weight along -y)
  const Ftotal = [Fx, Fy - payload * 9.81];

  // Joint torques τ = Jᵀ F
  const tau = [
    J[0][0] * Ftotal[0] + J[1][0] * Ftotal[1],
    J[0][1] * Ftotal[0] + J[1][1] * Ftotal[1]
  ];

  // Velocity ellipsoid (J Jᵀ eigenvalues)
  const JJT = [
    [J[0][0] ** 2 + J[0][1] ** 2, J[0][0] * J[1][0] + J[0][1] * J[1][1]],
    [J[0][0] * J[1][0] + J[0][1] * J[1][1], J[1][0] ** 2 + J[1][1] ** 2]
  ];
  const trV = JJT[0][0] + JJT[1][1];
  const detV = JJT[0][0] * JJT[1][1] - JJT[0][1] * JJT[1][0];
  const discV = Math.max(0, trV * trV - 4 * detV);
  const lamV1 = (trV + Math.sqrt(discV)) / 2;
  const lamV2 = (trV - Math.sqrt(discV)) / 2;
  const sigV1 = Math.sqrt(Math.max(0, lamV1));
  const sigV2 = Math.sqrt(Math.max(0, lamV2));
  // Eigenvector angle for lamV1
  let velAng;
  if (Math.abs(JJT[0][1]) > 1e-9) {
    velAng = Math.atan2(lamV1 - JJT[0][0], JJT[0][1]);
  } else {
    velAng = lamV1 >= JJT[0][0] ? 0 : Math.PI / 2;
  }

  // Force ellipsoid: axes are reciprocal of velocity axes, principal directions same
  const sigF1 = sigV2 > 1e-6 ? 1 / sigV2 : 100;
  const sigF2 = sigV1 > 1e-6 ? 1 / sigV1 : 100;
  // Force ellipse rotates 90° relative to vel (axes swap)
  const forceAng = velAng + Math.PI / 2;

  const nearSingular = Math.abs(detJ) < 0.05;

  // SVG
  const W = 460, H = 380;
  const ox = W / 2, oy = H / 2;
  const scale = 80;
  const toSvg = (x, y) => [ox + x * scale, oy - y * scale];
  const [bx, by] = toSvg(0, 0);
  const [jx, jy] = toSvg(x1, y1);
  const [ex, ey] = toSvg(xe, ye);

  // Force vector visualization
  const fMag = Math.sqrt(Ftotal[0] ** 2 + Ftotal[1] ** 2);
  const fScale = 0.02; // m per Newton
  const [fendX, fendY] = toSvg(xe + Ftotal[0] * fScale, ye + Ftotal[1] * fScale);

  // Click to apply force at end-effector
  const handleClick = (e) => {
    const rect = svgRef.current.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * W;
    const py = ((e.clientY - rect.top) / rect.height) * H;
    const wx = (px - ox) / scale;
    const wy = -(py - oy) / scale;
    // Force vector = (click - ee) / fScale
    setFx((wx - xe) / fScale);
    setFy(((wy - ye) / fScale) + payload * 9.81); // user-applied component (we add gravity back so display sums correctly)
  };

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Statics: Jᵀ maps wrenches to joint torques</h3>
        <p className="eq">τ = J(q)ᵀ F_e &nbsp;·&nbsp; rank-deficient J ⇒ direction of zero resistance</p>

        <Explainer
          title="Holding stuff = motor torque math"
          plain="Stick out your arm horizontally and hold a heavy book. Your shoulder works hard, your elbow works less. Why? Because moving your shoulder a tiny bit lifts the book a LOT (long lever); moving your elbow lifts it less. The Jacobian-transpose Jᵀ does this bookkeeping automatically: tell it the force at the end-effector and it tells you exactly how much torque each motor has to produce. It's the same matrix that moves velocities the other way — that 'two for one' is duality. Near a singular pose (arm fully extended), one direction needs nearly zero torque — that's why a locked elbow can support so much weight."
          equation="τ = J(q)ᵀ · F_e"
          equationNote="Siciliano eq. 5.106. F_e is the wrench (force + moment) at the end-effector; τ is the vector of joint torques to balance it. The principle of virtual work guarantees Jᵀ is the right matrix."
          knobs={[
            { name: 'q₁, q₂', what: 'arm pose — changes how forces project onto joints' },
            { name: 'F_x, F_y', what: 'external force at the end-effector — try pushing in different directions' },
            { name: 'payload', what: 'mass hanging at the end-effector — adds gravity force automatically' },
            { name: 'Click on canvas', what: 'apply force toward the click point — interactive!' }
          ]} />

        <svg ref={svgRef} width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)', cursor: 'crosshair' }}
          onClick={handleClick}>
          {/* Axes */}
          <line x1={0} y1={oy} x2={W} y2={oy} stroke="var(--grid)" strokeWidth="0.5" />
          <line x1={ox} y1={0} x2={ox} y2={H} stroke="var(--grid)" strokeWidth="0.5" />

          {/* Velocity ellipsoid */}
          {showVelEll && !nearSingular && (
            <ellipse cx={ex} cy={ey}
              rx={sigV1 * scale * 0.4}
              ry={sigV2 * scale * 0.4}
              fill="var(--plot-1)" fillOpacity="0.08"
              stroke="var(--plot-1)" strokeWidth="1" strokeDasharray="3 3"
              transform={`rotate(${-velAng * 180 / Math.PI} ${ex} ${ey})`} />
          )}

          {/* Force ellipsoid */}
          {showForceEll && !nearSingular && (
            <ellipse cx={ex} cy={ey}
              rx={Math.min(sigF1 * scale * 0.4, 120)}
              ry={Math.min(sigF2 * scale * 0.4, 120)}
              fill="var(--plot-2)" fillOpacity="0.10"
              stroke="var(--plot-2)" strokeWidth="1" strokeDasharray="3 3"
              transform={`rotate(${-forceAng * 180 / Math.PI} ${ex} ${ey})`} />
          )}

          {/* Links */}
          <line x1={bx} y1={by} x2={jx} y2={jy}
            stroke={nearSingular ? 'var(--danger)' : 'var(--plot-1)'}
            strokeWidth="5" strokeLinecap="round" />
          <line x1={jx} y1={jy} x2={ex} y2={ey}
            stroke={nearSingular ? 'var(--danger)' : 'var(--accent-2)'}
            strokeWidth="5" strokeLinecap="round" />

          {/* Joints */}
          <circle cx={bx} cy={by} r="6" fill="var(--bg)" stroke="var(--text)" strokeWidth="1.5" />
          <circle cx={jx} cy={jy} r="5" fill="var(--bg-elev)" stroke="var(--text-dim)" strokeWidth="1.5" />
          <circle cx={ex} cy={ey} r="4" fill="var(--plot-2)" />

          {/* Force arrow at EE */}
          {fMag > 0.1 && (
            <g>
              <line x1={ex} y1={ey} x2={fendX} y2={fendY}
                stroke="var(--warn)" strokeWidth="2.5" markerEnd="url(#arrow-fe)" />
              <defs>
                <marker id="arrow-fe" viewBox="0 0 10 10" refX="8" refY="5"
                  markerWidth="6" markerHeight="6" orient="auto-start-reverse">
                  <path d="M2 1L8 5L2 9" fill="none" stroke="var(--warn)" strokeWidth="1.5"
                    strokeLinecap="round" strokeLinejoin="round" />
                </marker>
              </defs>
              <text x={fendX + 6} y={fendY - 4} fill="var(--warn)" fontSize="11" fontFamily="var(--mono)">
                F = {fMag.toFixed(1)}N
              </text>
            </g>
          )}

          {/* Payload weight */}
          {payload > 0 && (
            <g>
              <circle cx={ex} cy={ey + 16} r="4" fill="var(--text-dim)" />
              <text x={ex + 8} y={ey + 20} fill="var(--text-dim)" fontSize="10" fontFamily="var(--mono)">
                {payload.toFixed(1)} kg
              </text>
            </g>
          )}

          <text x={10} y={16} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            click to apply force at end-effector
          </text>
        </svg>

        <div className="metrics">
          <div className="metric"><div className="label">τ₁ (N·m)</div><div className="value">{tau[0].toFixed(2)}</div></div>
          <div className="metric"><div className="label">τ₂ (N·m)</div><div className="value">{tau[1].toFixed(2)}</div></div>
          <div className="metric"><div className="label">|F|</div><div className="value">{fMag.toFixed(1)}N</div></div>
          <div className="metric"><div className="label">det J</div>
            <div className="value" style={{ color: nearSingular ? 'var(--danger)' : 'var(--text)' }}>{detJ.toFixed(3)}</div></div>
        </div>

        <p className="note">
          The yellow ellipse is the <strong>force manipulability</strong> ellipsoid — its long axis is the direction in which the
          arm <strong>resists external force best</strong>. The blue ellipse (toggle on) is the velocity ellipsoid: its long axis
          is the direction of <strong>easiest motion</strong>. They're orthogonal. This is the velocity/force
          <em> duality </em>(Siciliano §5.7). Near a singularity the force ellipse stretches to infinity along one axis — the
          arm can apply unbounded force in that direction with tiny joint torques (e.g. pushing a wall with a fully extended arm).
        </p>
      </div>

      <div className="controls">
        <h3>Pose</h3>
        <Slider label="q₁ (rad)" value={q1} onChange={setQ1} min={-Math.PI} max={Math.PI} step={0.01} />
        <Slider label="q₂ (rad)" value={q2} onChange={setQ2} min={-Math.PI} max={Math.PI} step={0.01} />
        <Slider label="L₁" value={L1} onChange={setL1} min={0.3} max={2} step={0.05} />
        <Slider label="L₂" value={L2} onChange={setL2} min={0.3} max={2} step={0.05} />

        <h3 style={{ marginTop: 16 }}>External wrench</h3>
        <Slider label="F_x (N)" value={Fx} onChange={setFx} min={-30} max={30} step={0.5} precision={1} />
        <Slider label="F_y (N)" value={Fy} onChange={setFy} min={-30} max={30} step={0.5} precision={1} />
        <Slider label="payload (kg)" value={payload} onChange={setPayload} min={0} max={10} step={0.1} precision={1} />

        <div className="btn-row" style={{ marginTop: 16 }}>
          <button className={`btn ${showForceEll ? 'primary' : ''}`}
            onClick={() => setShowForceEll(!showForceEll)}>Force ellipse</button>
          <button className={`btn ${showVelEll ? 'primary' : ''}`}
            onClick={() => setShowVelEll(!showVelEll)}>Velocity ellipse</button>
        </div>
      </div>
    </div>
  );
}
