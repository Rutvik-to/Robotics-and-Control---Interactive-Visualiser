import React, { useState, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano §10.7: Position-Based Visual Servoing (PBVS).
 *
 *  Workflow:
 *    1. Camera sees target features.
 *    2. From image features + known target geometry, ESTIMATE the 3D pose
 *       (Tcam_target) — this is the "PnP" problem in computer vision.
 *    3. Compute pose error in Cartesian space: e = T_desired - T_estimated
 *    4. Apply standard Cartesian control: v_c = -λ · e
 *
 *  Difference from IBVS (Ch. 10):
 *    - IBVS minimizes ERROR IN THE IMAGE (pixels) — robust to depth errors but
 *      can take weird Cartesian paths.
 *    - PBVS minimizes ERROR IN 3D SPACE (meters) — straight-line Cartesian path,
 *      but requires good camera calibration and an accurate target model.
 *
 *  In this widget, we "cheat" by directly using the true 3D pose for estimation
 *  (perfect estimator), so the user can see the clean Cartesian convergence.
 *  We add an OPTIONAL pose-estimation noise slider so they can see degradation.
 */

export default function PBVS() {
  const [lam, setLam] = useState(2.0);
  const [noise, setNoise] = useState(0.0);
  const [running, setRunning] = useState(false);
  const [cam, setCam] = useState({ x: 0.8, y: -0.4, z: 0, th: 0.5 });
  const animRef = useRef(null);
  const camRef = useRef(cam);
  useEffect(() => { camRef.current = cam; }, [cam]);

  // Desired camera pose: at origin, no rotation
  const desired = { x: 0, y: 0, z: 0, th: 0 };

  // Animate PBVS
  useEffect(() => {
    if (!running) return;
    const dt = 0.04;
    const tick = () => {
      const c = camRef.current;
      // Add gaussian-ish noise to "estimated" pose
      const n = () => noise * (Math.random() - 0.5) * 2;
      const est = {
        x: c.x + n() * 0.1,
        y: c.y + n() * 0.1,
        z: c.z + n() * 0.05,
        th: c.th + n() * 0.1
      };
      // Pose error
      const ex = desired.x - est.x;
      const ey = desired.y - est.y;
      const ez = desired.z - est.z;
      const eth = desired.th - est.th;

      const errNorm = Math.sqrt(ex * ex + ey * ey + ez * ez + eth * eth);
      if (errNorm < 0.02 && noise < 0.001) {
        setRunning(false);
        return;
      }

      // PBVS control law: v_c = -λ · e_pose (in world frame here for simplicity)
      const dx = lam * ex * dt;
      const dy = lam * ey * dt;
      const dz = lam * ez * dt;
      const dth = lam * eth * dt;

      setCam({ x: c.x + dx, y: c.y + dy, z: c.z + dz, th: c.th + dth });
      animRef.current = setTimeout(tick, dt * 1000);
    };
    animRef.current = setTimeout(tick, 30);
    return () => clearTimeout(animRef.current);
  }, [running, lam, noise]);

  // 4 corner target features (for visualization)
  const targetZ = 1.5;
  const half = 0.3;
  const worldPts = [
    [-half, -half, targetZ], [half, -half, targetZ],
    [half, half, targetZ], [-half, half, targetZ]
  ];

  const f_cam = 200;
  const project = (p, cp) => {
    const [X, Y, Z] = p;
    const dx = X - cp.x, dy = Y - cp.y, dz = Z - cp.z;
    const ct = Math.cos(-cp.th), st = Math.sin(-cp.th);
    const Xc = ct * dx - st * dy;
    const Yc = st * dx + ct * dy;
    const Zc = dz;
    if (Zc < 0.05) return { u: 0, v: 0, valid: false };
    return { u: f_cam * Xc / Zc, v: f_cam * Yc / Zc, valid: true };
  };

  const currentImg = worldPts.map(p => project(p, cam));
  const desiredImg = worldPts.map(p => project(p, desired));

  const poseErr = Math.sqrt(
    (cam.x - desired.x) ** 2 +
    (cam.y - desired.y) ** 2 +
    (cam.z - desired.z) ** 2 +
    (cam.th - desired.th) ** 2
  );

  const IW = 320, IH = 320;
  const toImg = (u, v) => [IW / 2 + u * 0.5, IH / 2 + v * 0.5];

  const WW = 220, WH = 220;
  const wScale = 80;
  const toW = (x, y) => [WW / 2 + x * wScale, WH / 2 - y * wScale];

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Position-based visual servoing (PBVS)</h3>
        <p className="eq">e_pose = T_desired − T_estimated &nbsp;·&nbsp; v_c = −λ · e_pose</p>

        <Explainer
          title="PBVS vs IBVS — what's different?"
          plain="Image-based (IBVS) tracks DOTS ON THE SCREEN — it doesn't care where the camera actually is in 3D, just that the dots end up where they should on the image. Robust but the camera can take weird paths through space. Position-based (PBVS) does the opposite: from the dots, it RECONSTRUCTS the camera's 3D pose, then commands the camera to move in a straight Cartesian line to the goal pose. Cleaner trajectories, but it lives or dies by how accurately you can recover 3D from 2D (camera calibration, target model). Bump the noise slider — small pose-estimation errors can throw it off."
          equation="v_c = −λ · (T_desired ⊖ T_estimated)"
          equationNote="Siciliano §10.7. The error is computed in Cartesian (3D) space rather than image space. Pose is typically extracted via Perspective-n-Point (PnP) algorithms."
          knobs={[
            { name: 'λ gain', what: 'how fast to close the pose gap — same role as in IBVS' },
            { name: 'noise', what: 'add Gaussian error to the 3D pose estimate — see PBVS\'s sensitivity to calibration' },
            { name: 'Run servo', what: 'starts the control loop; watch the camera move in a straight 3D line' }
          ]} />

        <div style={{ display: 'grid', gridTemplateColumns: '320px 1fr', gap: 16 }}>
          <div>
            <p style={{ fontSize: 11, color: 'var(--text-faint)', margin: '0 0 4px', fontFamily: 'var(--mono)' }}>
              IMAGE PLANE
            </p>
            <svg width={IW} height={IH} viewBox={`0 0 ${IW} ${IH}`}
              style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
              <line x1={0} y1={IH / 2} x2={IW} y2={IH / 2} stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={IW / 2} y1={0} x2={IW / 2} y2={IH} stroke="var(--grid)" strokeWidth="0.5" />
              <polygon
                points={desiredImg.map(p => toImg(p.u, p.v).join(',')).join(' ')}
                fill="none" stroke="var(--plot-2)" strokeWidth="1.5" strokeDasharray="4 3" />
              {currentImg.every(p => p.valid) && (
                <polygon
                  points={currentImg.map(p => toImg(p.u, p.v).join(',')).join(' ')}
                  fill="var(--plot-1)" fillOpacity="0.15"
                  stroke="var(--plot-1)" strokeWidth="2" />
              )}
              {currentImg.map((p, i) => {
                if (!p.valid) return null;
                const [px, py] = toImg(p.u, p.v);
                return <circle key={i} cx={px} cy={py} r="5" fill="var(--plot-1)" />;
              })}
            </svg>
          </div>

          <div>
            <p style={{ fontSize: 11, color: 'var(--text-faint)', margin: '0 0 4px', fontFamily: 'var(--mono)' }}>
              WORLD (Cartesian path)
            </p>
            <svg width={WW} height={WH} viewBox={`0 0 ${WW} ${WH}`}
              style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
              <line x1={0} y1={WH / 2} x2={WW} y2={WH / 2} stroke="var(--grid)" strokeWidth="0.5" />
              <line x1={WW / 2} y1={0} x2={WW / 2} y2={WH} stroke="var(--grid)" strokeWidth="0.5" />
              {/* Target square top-down */}
              <rect x={WW / 2 - half * wScale} y={WH / 2 - half * wScale}
                width={half * 2 * wScale} height={half * 2 * wScale}
                fill="var(--plot-2)" fillOpacity="0.2" stroke="var(--plot-2)" strokeWidth="1" />
              {/* Straight-line ideal path */}
              <line
                x1={toW(cam.x, cam.y)[0]} y1={toW(cam.x, cam.y)[1]}
                x2={toW(0, 0)[0]} y2={toW(0, 0)[1]}
                stroke="var(--accent-2)" strokeWidth="0.5" strokeDasharray="3 3" />
              {/* Goal marker */}
              {(() => {
                const [gx, gy] = toW(0, 0);
                return (
                  <g>
                    <circle cx={gx} cy={gy} r="6" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
                    <line x1={gx - 4} y1={gy} x2={gx + 4} y2={gy} stroke="var(--warn)" strokeWidth="1.5" />
                    <line x1={gx} y1={gy - 4} x2={gx} y2={gy + 4} stroke="var(--warn)" strokeWidth="1.5" />
                  </g>
                );
              })()}
              {/* Camera */}
              {(() => {
                const [cx, cy] = toW(cam.x, cam.y);
                const dx = Math.sin(cam.th) * 18, dy = -Math.cos(cam.th) * 18;
                return (
                  <g>
                    <line x1={cx} y1={cy} x2={cx + dx} y2={cy + dy} stroke="var(--plot-1)" strokeWidth="2" />
                    <circle cx={cx} cy={cy} r="6" fill="var(--plot-1)" />
                  </g>
                );
              })()}
            </svg>
          </div>
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">pose err</div><div className="value">{poseErr.toFixed(3)}</div></div>
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
            setCam({ x: 0.8, y: -0.4, z: 0, th: 0.5 });
          }}>Reset</button>
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
        <h3>Servo</h3>
        <Slider label="λ" value={lam} onChange={setLam} min={0.2} max={5} step={0.1} precision={1} />
        <Slider label="pose noise" value={noise} onChange={setNoise} min={0} max={0.5} step={0.01} precision={2} />

        <h3 style={{ marginTop: 16 }}>Camera pose</h3>
        <Slider label="x" value={cam.x} onChange={v => setCam(c => ({ ...c, x: v }))} min={-1.5} max={1.5} step={0.02} />
        <Slider label="y" value={cam.y} onChange={v => setCam(c => ({ ...c, y: v }))} min={-1.5} max={1.5} step={0.02} />
        <Slider label="z (along view)" value={cam.z} onChange={v => setCam(c => ({ ...c, z: v }))} min={-1} max={1} step={0.02} />
        <Slider label="θ" value={cam.th} onChange={v => setCam(c => ({ ...c, th: v }))} min={-1.5} max={1.5} step={0.02} />
      </div>
    </div>
  );
}
