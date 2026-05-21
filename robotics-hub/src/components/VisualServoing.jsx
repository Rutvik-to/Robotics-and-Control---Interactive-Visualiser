import React, { useState, useEffect, useRef, useMemo } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano Ch. 10: Image-based visual servoing (IBVS).
 *
 * Pinhole camera projection: s = (u, v) = f · (X/Z, Y/Z)
 *
 * Interaction matrix L_s (Siciliano eq. 10.31):
 *   ṡ = L_s(s, Z) · v_c    where v_c = [vx, vy, vz, ωx, ωy, ωz]ᵀ is camera twist.
 *
 *   For point feature (u, v) at depth Z:
 *     L_s = [ -f/Z   0     u/Z   uv/f       -(f+u²/f)  v  ]
 *           [  0    -f/Z   v/Z   f+v²/f     -uv/f      -u ]
 *
 * Control: v_c = -λ · L_s⁺ · (s − s*)  (Siciliano eq. 10.51).
 *
 * Simplified planar IBVS here:
 *   - 4 image features (corners of a square)
 *   - Camera moves in (X, Y, Z) and rotates about optical axis (theta)
 *   - World target = square; camera projects it; controller minimizes feature error.
 *
 * We restrict camera motion to (X, Y, Z, θ_z) for visualization tractability,
 * but the full interaction matrix structure is preserved.
 */

const f_cam = 200; // focal length (px)

// World square (4 corners) at z = 1 default
function projectPoint([X, Y, Z], camPose) {
  // Camera pose: [cx, cy, cz, theta]; world->camera = rotate by -theta around z, then translate
  const [cx, cy, cz, th] = camPose;
  const dx = X - cx, dy = Y - cy, dz = Z - cz;
  const ct = Math.cos(-th), st = Math.sin(-th);
  const Xc = ct * dx - st * dy;
  const Yc = st * dx + ct * dy;
  const Zc = dz;
  if (Zc < 0.05) return { u: 0, v: 0, Z: Zc, valid: false };
  const u = f_cam * Xc / Zc;
  const v = f_cam * Yc / Zc;
  return { u, v, Z: Zc, valid: true };
}

// 2x4 interaction matrix for one point: maps (vx, vy, vz, omega_z) → (u̇, v̇)
function interactionRow(u, v, Z) {
  const f = f_cam;
  return [
    [-f / Z, 0, u / Z, v],
    [0, -f / Z, v / Z, -u]
  ];
}

// Pseudo-inverse of 8x4 matrix (4 points × 2 = 8 rows): A⁺ = (AᵀA)⁻¹ Aᵀ
function pinv8x4(A) {
  // AᵀA is 4x4
  const AtA = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 8; k++)
        AtA[i][j] += A[k][i] * A[k][j];

  // 4x4 inverse via cofactor expansion (small, manageable)
  const inv = matInv4(AtA);
  if (!inv) return null;

  // pinv = (AᵀA)⁻¹ Aᵀ : 4x8
  const result = [[0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]];
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 8; j++)
      for (let k = 0; k < 4; k++)
        result[i][j] += inv[i][k] * A[j][k];
  return result;
}

// 4x4 matrix inverse via adjugate
function matInv4(M) {
  // Compute determinant via cofactor expansion along first row
  const m = M;
  const det4 = (
    m[0][0] * det3(sub(m, 0, 0)) -
    m[0][1] * det3(sub(m, 0, 1)) +
    m[0][2] * det3(sub(m, 0, 2)) -
    m[0][3] * det3(sub(m, 0, 3))
  );
  if (Math.abs(det4) < 1e-12) return null;
  const inv = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      const sign = ((i + j) % 2 === 0) ? 1 : -1;
      inv[j][i] = sign * det3(sub(m, i, j)) / det4;
    }
  }
  return inv;
}
function sub(M, ri, rj) {
  const out = [];
  for (let i = 0; i < 4; i++) {
    if (i === ri) continue;
    const row = [];
    for (let j = 0; j < 4; j++) {
      if (j === rj) continue;
      row.push(M[i][j]);
    }
    out.push(row);
  }
  return out;
}
function det3(M) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

export default function VisualServoing() {
  const [lam, setLam] = useState(1.5);
  const [running, setRunning] = useState(false);
  // Camera state: x, y, z (along optical axis), theta_z
  const [cam, setCam] = useState({ x: 0.7, y: -0.4, z: 0, th: 0.4 });
  const animRef = useRef(null);
  const camRef = useRef(cam);
  useEffect(() => { camRef.current = cam; }, [cam]);

  // World feature points (a square at Z = 1.5 by default)
  const targetZ = 1.5;
  const halfSide = 0.3;
  const worldPts = [
    [-halfSide, -halfSide, targetZ],
    [halfSide, -halfSide, targetZ],
    [halfSide, halfSide, targetZ],
    [-halfSide, halfSide, targetZ]
  ];

  // Desired image features (when camera is at origin pointing at z = 1.5)
  const desiredImg = worldPts.map(p => projectPoint(p, [0, 0, 0, 0]));

  // Current projections
  const currentImg = worldPts.map(p => projectPoint(p, [cam.x, cam.y, cam.z, cam.th]));

  // Animate IBVS
  useEffect(() => {
    if (!running) return;
    const dt = 0.04;
    const tick = () => {
      const c = camRef.current;
      const proj = worldPts.map(p => projectPoint(p, [c.x, c.y, c.z, c.th]));
      if (proj.some(p => !p.valid)) {
        setRunning(false);
        return;
      }

      // Build 8x4 interaction stack: rows for (u1,v1, u2,v2, u3,v3, u4,v4)
      // columns: vx, vy, vz, omega_z
      const L = [];
      const e = [];
      for (let i = 0; i < 4; i++) {
        const rows = interactionRow(proj[i].u, proj[i].v, proj[i].Z);
        L.push(rows[0]);
        L.push(rows[1]);
        e.push(proj[i].u - desiredImg[i].u);
        e.push(proj[i].v - desiredImg[i].v);
      }

      const Lpinv = pinv8x4(L);
      if (!Lpinv) {
        setRunning(false);
        return;
      }
      // v_c = -λ · L⁺ · e
      const vc = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 8; j++) {
          vc[i] += Lpinv[i][j] * e[j];
        }
        vc[i] *= -lam;
      }

      // Update camera pose
      // vc is in CAMERA frame; convert to world by rotating about z by th
      const ct = Math.cos(c.th), st = Math.sin(c.th);
      const newCam = {
        x: c.x + (ct * vc[0] - st * vc[1]) * dt,
        y: c.y + (st * vc[0] + ct * vc[1]) * dt,
        z: c.z + vc[2] * dt,
        th: c.th + vc[3] * dt
      };
      setCam(newCam);

      // Stop if converged
      const errNorm = Math.sqrt(e.reduce((s, v) => s + v * v, 0));
      if (errNorm < 1.0) {
        setRunning(false);
        return;
      }

      animRef.current = setTimeout(tick, dt * 1000);
    };
    animRef.current = setTimeout(tick, 30);
    return () => clearTimeout(animRef.current);
  }, [running, lam]);

  // Error metric
  const errVec = currentImg.map((p, i) => [p.u - desiredImg[i].u, p.v - desiredImg[i].v]);
  const errNorm = Math.sqrt(errVec.reduce((s, [a, b]) => s + a * a + b * b, 0));

  // Image plane render (320x320)
  const IW = 320, IH = 320;
  const toImg = (u, v) => [IW / 2 + u * 0.5, IH / 2 + v * 0.5];

  // World top-down render (220x220)
  const WW = 220, WH = 220;
  const wScale = 80;
  const toW = (x, y) => [WW / 2 + x * wScale, WH / 2 - y * wScale];

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Image-based visual servoing (IBVS)</h3>
        <p className="eq">v_c = −λ · L_s⁺ · (s − s*),  ṡ = L_s · v_c</p>

        <Explainer
          title="What is visual servoing?"
          plain="The robot has a camera and sees four corners of a target square. It compares where each corner CURRENTLY appears in the image vs where it SHOULD appear when the robot is in the right pose. The interaction matrix L_s is the cheat sheet that says 'if I move the camera this way, that's how the dots move on the screen.' We invert it to plan the move. Result: the dots slide across the screen until they hit their target positions, and the camera ends up where we wanted."
          equation="v_c = −λ · L_s⁺ · (s − s*)"
          equationNote="Siciliano eq. 10.51. s = current image features (4 corners × 2 coords = 8 numbers). s* = desired image features. v_c = camera velocity (we use 4 DOF here: 3 translation + 1 rotation). λ controls how fast we close the gap."
          knobs={[
            { name: 'λ gain', what: 'closing speed — too small is slow, too big is jittery' },
            { name: 'Play', what: 'starts the loop; camera moves until image features overlap their targets' },
            { name: 'Manual sliders', what: 'reposition the camera by hand to see how the image distorts' }
          ]} />

        <div style={{ display: 'grid', gridTemplateColumns: '320px 1fr', gap: 16 }}>
          {/* Image plane */}
          <div>
            <p style={{ fontSize: 11, color: 'var(--text-faint)', margin: '0 0 4px', fontFamily: 'var(--mono)' }}>
              IMAGE PLANE (what camera sees)
            </p>
            <svg width={IW} height={IH} viewBox={`0 0 ${IW} ${IH}`}
              style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
              {/* Crosshair */}
              <line x1={0} y1={IH / 2} x2={IW} y2={IH / 2} stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={IW / 2} y1={0} x2={IW / 2} y2={IH} stroke="var(--grid)" strokeWidth="0.5" />
              {/* Desired square (outline) */}
              <polygon
                points={desiredImg.map(p => toImg(p.u, p.v).join(',')).join(' ')}
                fill="none" stroke="var(--plot-2)" strokeWidth="1.5" strokeDasharray="4 3" />
              {desiredImg.map((p, i) => {
                const [px, py] = toImg(p.u, p.v);
                return <circle key={`d${i}`} cx={px} cy={py} r="4" fill="none" stroke="var(--plot-2)" strokeWidth="1.5" />;
              })}
              {/* Current square */}
              {currentImg.every(p => p.valid) && (
                <polygon
                  points={currentImg.map(p => toImg(p.u, p.v).join(',')).join(' ')}
                  fill="var(--plot-1)" fillOpacity="0.15"
                  stroke="var(--plot-1)" strokeWidth="2" />
              )}
              {currentImg.map((p, i) => {
                if (!p.valid) return null;
                const [px, py] = toImg(p.u, p.v);
                return <circle key={`c${i}`} cx={px} cy={py} r="5" fill="var(--plot-1)" />;
              })}
              {/* Error vectors */}
              {currentImg.map((p, i) => {
                if (!p.valid) return null;
                const [px, py] = toImg(p.u, p.v);
                const [tx, ty] = toImg(desiredImg[i].u, desiredImg[i].v);
                return <line key={`e${i}`} x1={px} y1={py} x2={tx} y2={ty}
                  stroke="var(--warn)" strokeWidth="0.8" opacity="0.6" />;
              })}
              <text x={6} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
                desired (yellow) · current (blue)
              </text>
            </svg>
          </div>

          {/* World top-down */}
          <div>
            <p style={{ fontSize: 11, color: 'var(--text-faint)', margin: '0 0 4px', fontFamily: 'var(--mono)' }}>
              WORLD (top-down)
            </p>
            <svg width={WW} height={WH} viewBox={`0 0 ${WW} ${WH}`}
              style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
              <line x1={0} y1={WH / 2} x2={WW} y2={WH / 2} stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={WW / 2} y1={0} x2={WW / 2} y2={WH} stroke="var(--grid)" strokeWidth="0.5" />
              {/* Target square (top-down, just shows x-y extent) */}
              <rect x={WW / 2 - halfSide * wScale} y={WH / 2 - halfSide * wScale}
                width={halfSide * 2 * wScale} height={halfSide * 2 * wScale}
                fill="var(--plot-2)" fillOpacity="0.2" stroke="var(--plot-2)" strokeWidth="1" />
              <text x={WW / 2} y={WH / 2 + halfSide * wScale + 14} textAnchor="middle"
                fill="var(--plot-2)" fontSize="9" fontFamily="var(--mono)">target</text>
              {/* Camera */}
              {(() => {
                const [cx, cy] = toW(cam.x, cam.y);
                const dx = Math.sin(cam.th) * 18, dy = -Math.cos(cam.th) * 18;
                return (
                  <g>
                    <line x1={cx} y1={cy} x2={cx + dx} y2={cy + dy}
                      stroke="var(--plot-1)" strokeWidth="2" />
                    <circle cx={cx} cy={cy} r="6" fill="var(--plot-1)" />
                  </g>
                );
              })()}
              <text x={6} y={14} fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">
                camera pose
              </text>
            </svg>
          </div>
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">image error</div><div className="value">{errNorm.toFixed(1)}px</div></div>
          <div className="metric"><div className="label">cam x</div><div className="value">{cam.x.toFixed(2)}</div></div>
          <div className="metric"><div className="label">cam y</div><div className="value">{cam.y.toFixed(2)}</div></div>
          <div className="metric"><div className="label">cam θ</div><div className="value">{(cam.th * 180 / Math.PI).toFixed(0)}°</div></div>
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Run servo'}
          </button>
          <button className="btn" onClick={() => {
            setRunning(false);
            setCam({ x: 0.7, y: -0.4, z: 0, th: 0.4 });
          }}>Reset pose</button>
          <button className="btn" onClick={() => {
            setRunning(false);
            const r1 = (Math.random() - 0.5) * 1.5;
            const r2 = (Math.random() - 0.5) * 1.5;
            const r3 = (Math.random() - 0.5) * 1.2;
            setCam({ x: r1, y: r2, z: 0, th: r3 });
          }}>Random start</button>
        </div>
      </div>

      <div className="controls">
        <h3>Servo gain</h3>
        <Slider label="λ" value={lam} onChange={setLam} min={0.2} max={5} step={0.1} precision={1} />

        <h3 style={{ marginTop: 16 }}>Camera pose</h3>
        <Slider label="x" value={cam.x} onChange={v => setCam(c => ({ ...c, x: v }))} min={-1.5} max={1.5} step={0.02} />
        <Slider label="y" value={cam.y} onChange={v => setCam(c => ({ ...c, y: v }))} min={-1.5} max={1.5} step={0.02} />
        <Slider label="z (along view)" value={cam.z} onChange={v => setCam(c => ({ ...c, z: v }))} min={-1} max={1} step={0.02} />
        <Slider label="θ (rad)" value={cam.th} onChange={v => setCam(c => ({ ...c, th: v }))} min={-1.5} max={1.5} step={0.02} />
      </div>
    </div>
  );
}
