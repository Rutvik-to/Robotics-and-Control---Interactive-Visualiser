import React, { useState, useMemo } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano §2.8-2.9: Denavit-Hartenberg parameters
 *
 * Each link i has 4 DH parameters (a_i, alpha_i, d_i, theta_i):
 *   a_i     = link length (along x_i)
 *   alpha_i = link twist (about x_i)
 *   d_i     = link offset (along z_{i-1})
 *   theta_i = joint angle (about z_{i-1})
 *
 * Homogeneous transform from frame i-1 to frame i:
 *   A_i = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
 *
 *   [c_theta  -s_theta*c_alpha   s_theta*s_alpha   a*c_theta]
 *   [s_theta   c_theta*c_alpha  -c_theta*s_alpha   a*s_theta]
 *   [   0          s_alpha           c_alpha            d   ]
 *   [   0             0                 0                1   ]
 *
 * We project the 3D frame chain onto 2D using an isometric-style projection.
 * Also shows the end-effector orientation as Euler ZYZ angles (§2.4.1).
 */

// Build 4x4 homogeneous transform from DH params
function dhTransform(a, alpha, d, theta) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1]
  ];
}

// 4x4 matrix multiply
function matMul4(A, B) {
  const R = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 4; k++)
        R[i][j] += A[i][k] * B[k][j];
  return R;
}

// Extract ZYZ Euler angles from 3x3 rotation matrix top-left of 4x4
// R = Rz(phi) Ry(theta) Rz(psi)
function rotToZYZ(M) {
  const r33 = M[2][2];
  if (Math.abs(r33) < 1 - 1e-6) {
    const theta = Math.acos(r33);
    const phi = Math.atan2(M[1][2], M[0][2]);
    const psi = Math.atan2(M[2][1], -M[2][0]);
    return { phi, theta, psi };
  } else {
    // Singular: theta = 0 or π
    return { phi: Math.atan2(M[1][0], M[0][0]), theta: r33 > 0 ? 0 : Math.PI, psi: 0 };
  }
}

// Isometric-style projection: 3D point -> 2D
function project([x, y, z], yaw = 0.6, pitch = 0.4) {
  // Rotate world by yaw around Z, then pitch around X
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  // Rz(yaw)
  const x1 = cy * x - sy * y;
  const y1 = sy * x + cy * y;
  const z1 = z;
  // Rx(pitch)
  const x2 = x1;
  const y2 = cp * y1 - sp * z1;
  const z2 = sp * y1 + cp * z1;
  return [x2, -y2]; // y flipped for SVG
}

export default function DHKinematics() {
  // Anthropomorphic-style 3-DOF arm by default (shoulder pan, shoulder pitch, elbow)
  // DH params from Siciliano Ex 2.7 (anthropomorphic arm), simplified
  const [links, setLinks] = useState([
    { a: 0, alpha: Math.PI / 2, d: 0.5, theta: 0, label: 'shoulder pan' },
    { a: 1.0, alpha: 0, d: 0, theta: -Math.PI / 4, label: 'shoulder pitch' },
    { a: 0.8, alpha: 0, d: 0, theta: Math.PI / 4, label: 'elbow' }
  ]);
  const [yaw, setYaw] = useState(0.6);
  const [pitch, setPitch] = useState(0.4);
  const [showFrames, setShowFrames] = useState(true);

  // Compute forward kinematics chain
  const { frames, T_end } = useMemo(() => {
    const I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    const frames = [I];
    let T = I;
    for (const link of links) {
      const A = dhTransform(link.a, link.alpha, link.d, link.theta);
      T = matMul4(T, A);
      frames.push(T);
    }
    return { frames, T_end: T };
  }, [links]);

  const eulerZYZ = rotToZYZ(T_end);
  const eePos = [T_end[0][3], T_end[1][3], T_end[2][3]];

  // SVG setup
  const W = 480, H = 380;
  const cx = W / 2, cy = H / 2 + 40;
  const scale = 90;

  const toSvg = (p3) => {
    const [px, py] = project(p3, yaw, pitch);
    return [cx + px * scale, cy + py * scale];
  };

  // Extract origin from a 4x4 transform
  const originOf = (T) => [T[0][3], T[1][3], T[2][3]];
  // Extract axis (column) — column 0=x, 1=y, 2=z
  const axisOf = (T, col, len = 0.25) => {
    const o = originOf(T);
    return [o[0] + len * T[0][col], o[1] + len * T[1][col], o[2] + len * T[2][col]];
  };

  // Update a single DH parameter
  const updateLink = (i, key, val) => {
    setLinks(ls => ls.map((l, idx) => idx === i ? { ...l, [key]: val } : l));
  };

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Denavit-Hartenberg chain (3-link, 3D)</h3>
        <p className="eq">A_i = Rot_z(θᵢ)·Trans_z(dᵢ)·Trans_x(aᵢ)·Rot_x(αᵢ) &nbsp;·&nbsp; T_n⁰ = ∏ Aᵢ</p>

        <Explainer
          title="What are DH parameters?"
          plain="Every robot arm — no matter how weird — can be described by just 4 numbers per joint: a (link length), α (link twist), d (link offset), θ (joint angle). It's a recipe: stand at the previous joint, rotate by θ, slide along z by d, slide along x by a, twist by α — now you're at the next joint. Chain these together and you've built any robot in the world. The Denavit-Hartenberg convention is the universal language industrial robotics uses to describe geometry."
          equation="Aᵢ = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ) · Rot_x(αᵢ)"
          equationNote="Siciliano eq. 2.52. Multiply A₁·A₂·A₃·… to get the end-effector pose relative to the base. The 3×3 rotation block tells you which way the gripper faces; the column on the right gives position."
          knobs={[
            { name: 'θᵢ', what: 'joint angle — this is what motors control' },
            { name: 'aᵢ', what: 'link length — physical distance between joints' },
            { name: 'dᵢ', what: 'offset along the joint axis (z) — common in wrist mechanisms' },
            { name: 'αᵢ', what: 'twist between consecutive axes; α=90° turns the plane of motion' },
            { name: 'yaw / pitch', what: 'just rotates the view — doesn\'t affect the robot' }
          ]} />

        <svg width="100%" viewBox={`0 0 ${W} ${H}`}
          style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
          {/* Floor grid */}
          {[-2, -1, 0, 1, 2].map(g => {
            const [a, b] = toSvg([g * 0.5, -1, 0]);
            const [c, d] = toSvg([g * 0.5, 1, 0]);
            return <line key={`gx${g}`} x1={a} y1={b} x2={c} y2={d} stroke="var(--grid)" strokeWidth="0.5" />;
          })}
          {[-2, -1, 0, 1, 2].map(g => {
            const [a, b] = toSvg([-1, g * 0.5, 0]);
            const [c, d] = toSvg([1, g * 0.5, 0]);
            return <line key={`gy${g}`} x1={a} y1={b} x2={c} y2={d} stroke="var(--grid)" strokeWidth="0.5" />;
          })}

          {/* Links */}
          {frames.slice(0, -1).map((F, i) => {
            const o1 = originOf(F);
            const o2 = originOf(frames[i + 1]);
            const [x1, y1] = toSvg(o1);
            const [x2, y2] = toSvg(o2);
            const color = i === 0 ? 'var(--plot-1)' : i === 1 ? 'var(--accent-2)' : 'var(--plot-2)';
            return (
              <line key={`L${i}`} x1={x1} y1={y1} x2={x2} y2={y2}
                stroke={color} strokeWidth="5" strokeLinecap="round" />
            );
          })}

          {/* Frame axes */}
          {showFrames && frames.map((F, i) => {
            const [ox, oy] = toSvg(originOf(F));
            const [xx, xy] = toSvg(axisOf(F, 0));
            const [yx, yy] = toSvg(axisOf(F, 1));
            const [zx, zy] = toSvg(axisOf(F, 2));
            return (
              <g key={`F${i}`}>
                <line x1={ox} y1={oy} x2={xx} y2={xy} stroke="#e24b4a" strokeWidth="1.5" />
                <line x1={ox} y1={oy} x2={yx} y2={yy} stroke="#5dcaa5" strokeWidth="1.5" />
                <line x1={ox} y1={oy} x2={zx} y2={zy} stroke="#7c9eff" strokeWidth="1.5" />
                <text x={ox + 6} y={oy - 6} fill="var(--text-dim)" fontSize="10" fontFamily="var(--mono)">
                  {i === 0 ? '0' : i === frames.length - 1 ? 'e' : i}
                </text>
              </g>
            );
          })}

          {/* Joints */}
          {frames.map((F, i) => {
            const [px, py] = toSvg(originOf(F));
            return (
              <circle key={`J${i}`} cx={px} cy={py} r={i === 0 ? 6 : 4}
                fill={i === frames.length - 1 ? 'var(--plot-2)' : 'var(--bg-elev)'}
                stroke="var(--text)" strokeWidth="1.5" />
            );
          })}

          <text x={10} y={16} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
            x = red · y = green · z = blue
          </text>
        </svg>

        <div className="metrics">
          <div className="metric"><div className="label">x_e</div><div className="value">{eePos[0].toFixed(3)}</div></div>
          <div className="metric"><div className="label">y_e</div><div className="value">{eePos[1].toFixed(3)}</div></div>
          <div className="metric"><div className="label">z_e</div><div className="value">{eePos[2].toFixed(3)}</div></div>
        </div>
        <div className="metrics">
          <div className="metric"><div className="label">φ (ZYZ)</div><div className="value">{(eulerZYZ.phi * 180 / Math.PI).toFixed(1)}°</div></div>
          <div className="metric"><div className="label">θ</div><div className="value">{(eulerZYZ.theta * 180 / Math.PI).toFixed(1)}°</div></div>
          <div className="metric"><div className="label">ψ</div><div className="value">{(eulerZYZ.psi * 180 / Math.PI).toFixed(1)}°</div></div>
        </div>
        <p className="note">
          Each frame is positioned by its DH transform. The base frame is at the origin; each subsequent frame
          applies <code>Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)</code> to the previous. Notice <code>α₁ = 90°</code> twists
          the first joint axis so the next two operate in a plane perpendicular to the floor — classic shoulder pan + pitch geometry.
          End-effector orientation reported as ZYZ Euler angles (Siciliano §2.4.1).
        </p>
      </div>

      <div className="controls">
        <h3>View</h3>
        <Slider label="yaw" value={yaw} onChange={setYaw} min={-Math.PI} max={Math.PI} step={0.02} />
        <Slider label="pitch" value={pitch} onChange={setPitch} min={-1.2} max={1.2} step={0.02} />
        <div className="btn-row" style={{ marginTop: 8 }}>
          <button className={`btn ${showFrames ? 'primary' : ''}`}
            onClick={() => setShowFrames(!showFrames)}>
            Frames: {showFrames ? 'ON' : 'OFF'}
          </button>
        </div>

        {links.map((link, i) => (
          <div key={i} style={{ marginTop: 16, paddingTop: 12, borderTop: '0.5px solid var(--border)' }}>
            <h3 style={{ marginBottom: 8 }}>Link {i + 1} — {link.label}</h3>
            <Slider label={`θ${i + 1} (rad)`} value={link.theta}
              onChange={v => updateLink(i, 'theta', v)} min={-Math.PI} max={Math.PI} step={0.02} />
            <Slider label={`a${i + 1}`} value={link.a}
              onChange={v => updateLink(i, 'a', v)} min={0} max={1.5} step={0.02} />
            <Slider label={`d${i + 1}`} value={link.d}
              onChange={v => updateLink(i, 'd', v)} min={0} max={1.5} step={0.02} />
            <Slider label={`α${i + 1} (rad)`} value={link.alpha}
              onChange={v => updateLink(i, 'alpha', v)} min={-Math.PI} max={Math.PI} step={0.02} />
          </div>
        ))}
      </div>
    </div>
  );
}
