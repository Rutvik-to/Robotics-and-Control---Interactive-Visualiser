import React, { useState, useEffect, useRef } from 'react';
import Slider from './Slider.jsx';
import Explainer from './Explainer.jsx';

/**
 * Siciliano Ch. 12: Motion planning.
 *
 *  ARTIFICIAL POTENTIAL FIELDS (§12.4):
 *    U_total(q) = U_attractive(q) + U_repulsive(q)
 *    U_att = 0.5 * ka * ||q - qf||²
 *    U_rep = if d(q) <= rho0:  0.5 * kr * (1/d - 1/rho0)²
 *            else:             0
 *    Gradient descent: q̇ = -∇U
 *
 *    Pitfall: local minima where ∇U = 0 but q ≠ qf.
 *
 *  RRT (§12.5.3):
 *    Tree of random samples connecting start to goal.
 *    1. Sample random configuration q_rand
 *    2. Find nearest node q_near in tree
 *    3. Extend toward q_rand by step ε → q_new
 *    4. If q_new is collision-free, add edge (q_near, q_new)
 *    5. Repeat until goal reached
 */

// Simple obstacle: circle with center and radius
function distToObstacle(p, obs) {
  return Math.sqrt((p[0] - obs.x) ** 2 + (p[1] - obs.y) ** 2) - obs.r;
}

function inCollision(p, obstacles) {
  return obstacles.some(o => distToObstacle(p, o) < 0);
}

// Line segment in collision?
function segCollision(a, b, obstacles, steps = 10) {
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
    if (inCollision(p, obstacles)) return true;
  }
  return false;
}

export default function MotionPlanning() {
  const [method, setMethod] = useState('apf'); // 'apf' | 'rrt'
  const [ka, setKa] = useState(2.0);
  const [kr, setKr] = useState(80.0);
  const [rho0, setRho0] = useState(0.5);
  const [stepRRT, setStepRRT] = useState(0.2);
  const [running, setRunning] = useState(false);
  const [pos, setPos] = useState([-2.0, -1.5]);
  const [trail, setTrail] = useState([]);
  const [rrtNodes, setRrtNodes] = useState([{ p: [-2.0, -1.5], parent: -1 }]);
  const [rrtFound, setRrtFound] = useState(false);
  const animRef = useRef(null);
  const posRef = useRef(pos);
  const nodesRef = useRef(rrtNodes);
  useEffect(() => { posRef.current = pos; }, [pos]);
  useEffect(() => { nodesRef.current = rrtNodes; }, [rrtNodes]);

  const goal = [2.0, 1.5];
  const start = [-2.0, -1.5];
  const obstacles = [
    { x: -0.5, y: 0.5, r: 0.6 },
    { x: 1.0, y: -0.3, r: 0.5 },
    { x: 0.2, y: -1.2, r: 0.4 }
  ];

  // APF animation
  useEffect(() => {
    if (!running || method !== 'apf') return;
    const dt = 0.03;
    const tick = () => {
      const p = posRef.current;
      // Attractive gradient
      const ax = ka * (p[0] - goal[0]);
      const ay = ka * (p[1] - goal[1]);
      // Repulsive gradient
      let rx = 0, ry = 0;
      obstacles.forEach(o => {
        const dx = p[0] - o.x;
        const dy = p[1] - o.y;
        const d = Math.sqrt(dx * dx + dy * dy) - o.r;
        if (d > 0 && d <= rho0) {
          const factor = kr * (1 / d - 1 / rho0) / (d * d);
          const norm = Math.sqrt(dx * dx + dy * dy);
          rx += factor * dx / (norm || 1);
          ry += factor * dy / (norm || 1);
        }
      });
      // Descend total gradient
      const gx = ax + rx, gy = ay + ry;
      const mag = Math.sqrt(gx * gx + gy * gy);
      if (mag < 0.05) {
        setRunning(false);
        return;
      }
      const newP = [p[0] - 0.4 * gx * dt / Math.max(1, mag),
      p[1] - 0.4 * gy * dt / Math.max(1, mag)];

      // Reached goal?
      const dg = Math.sqrt((newP[0] - goal[0]) ** 2 + (newP[1] - goal[1]) ** 2);
      if (dg < 0.1) {
        setPos(newP);
        setRunning(false);
        return;
      }

      setPos(newP);
      setTrail(t => {
        const next = [...t, newP];
        if (next.length > 600) next.shift();
        return next;
      });
      animRef.current = setTimeout(tick, dt * 1000);
    };
    animRef.current = setTimeout(tick, 30);
    return () => clearTimeout(animRef.current);
  }, [running, method, ka, kr, rho0]);

  // RRT incremental
  useEffect(() => {
    if (!running || method !== 'rrt') return;
    let iterations = 0;
    const tick = () => {
      if (rrtFound || iterations > 200) {
        setRunning(false);
        return;
      }
      iterations++;
      // Sample
      let qRand;
      // Bias toward goal 10% of the time
      if (Math.random() < 0.1) {
        qRand = goal;
      } else {
        qRand = [(Math.random() - 0.5) * 6, (Math.random() - 0.5) * 4];
      }
      const nodes = nodesRef.current;
      // Find nearest
      let nearestIdx = 0;
      let nearestD = Infinity;
      for (let i = 0; i < nodes.length; i++) {
        const d = (nodes[i].p[0] - qRand[0]) ** 2 + (nodes[i].p[1] - qRand[1]) ** 2;
        if (d < nearestD) { nearestD = d; nearestIdx = i; }
      }
      const qNear = nodes[nearestIdx].p;
      // Extend
      const dx = qRand[0] - qNear[0], dy = qRand[1] - qNear[1];
      const norm = Math.sqrt(dx * dx + dy * dy);
      const qNew = [qNear[0] + (dx / norm) * stepRRT, qNear[1] + (dy / norm) * stepRRT];
      // Collision check
      if (!segCollision(qNear, qNew, obstacles)) {
        const newNodes = [...nodes, { p: qNew, parent: nearestIdx }];
        setRrtNodes(newNodes);
        // Goal reached?
        const dg = Math.sqrt((qNew[0] - goal[0]) ** 2 + (qNew[1] - goal[1]) ** 2);
        if (dg < stepRRT) {
          setRrtFound(true);
          setRunning(false);
          return;
        }
      }
      animRef.current = setTimeout(tick, 25);
    };
    animRef.current = setTimeout(tick, 25);
    return () => clearTimeout(animRef.current);
  }, [running, method, stepRRT, rrtFound]);

  // Build path through RRT from goal back to start
  const rrtPath = (() => {
    if (rrtNodes.length === 0) return [];
    // Find node closest to goal
    let bestIdx = -1, bestD = Infinity;
    for (let i = 0; i < rrtNodes.length; i++) {
      const d = (rrtNodes[i].p[0] - goal[0]) ** 2 + (rrtNodes[i].p[1] - goal[1]) ** 2;
      if (d < bestD) { bestD = d; bestIdx = i; }
    }
    if (bestD > stepRRT * stepRRT) return [];
    // Walk parents
    const path = [];
    let i = bestIdx;
    while (i >= 0) {
      path.push(rrtNodes[i].p);
      i = rrtNodes[i].parent;
    }
    return path.reverse();
  })();

  // SVG
  const W = 500, H = 360;
  const scale = 70;
  const ox = W / 2, oy = H / 2;
  const toS = (x, y) => [ox + x * scale, oy - y * scale];

  const resetAll = () => {
    setRunning(false);
    setPos([...start]);
    setTrail([]);
    setRrtNodes([{ p: [...start], parent: -1 }]);
    setRrtFound(false);
  };

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Motion planning: {method === 'apf' ? 'artificial potential fields' : 'RRT'}</h3>
        <p className="eq">
          {method === 'apf'
            ? 'U = ½ kₐ ‖q − q_f‖² + ½ kᵣ (1/d − 1/ρ₀)²  ·  q̇ = −∇U'
            : 'q_new = q_near + ε · (q_rand − q_near)/‖·‖   (collision-checked)'}
        </p>

        <Explainer
          title={method === 'apf' ? 'Potential fields explained' : 'How does RRT explore?'}
          plain={method === 'apf'
            ? "Treat the goal like a magnet (attractive) and obstacles like other magnets that REPEL you. The robot just rolls 'downhill' on the combined potential energy landscape. Simple, fast, reactive — but the killer flaw is LOCAL MINIMA: spots where attractive and repulsive forces cancel and the robot gets stuck even though the goal isn't reached. Try putting yourself in a U-shape of obstacles and watch."
            : "Throw a dart at the configuration space. Find the nearest point in your tree. Take a small step from that point toward the dart. If the step is collision-free, add it to the tree. Repeat thousands of times. Eventually a branch reaches the goal. RRT is RANDOMIZED — different every run — and PROBABILISTICALLY COMPLETE: given enough samples, it WILL find a path if one exists. Used by autonomous cars, drug-molecule docking, and many movie-CG robots."}
          equation={method === 'apf'
            ? "U_att = ½kₐ‖q − q_f‖²,  U_rep = ½kᵣ(1/d − 1/ρ₀)²  if d ≤ ρ₀"
            : "Sample → find nearest → extend by ε → check collision → repeat"}
          equationNote={method === 'apf'
            ? "Siciliano §12.4. ρ₀ is the obstacle's 'influence radius'; outside it, the obstacle doesn't push. ka and kr trade off goal-seeking vs obstacle-avoiding."
            : "Siciliano §12.5.3. Bias the sampling toward the goal (we use 10%) to speed convergence. RRT* refines the tree to find shorter paths over time."}
          knobs={method === 'apf' ? [
            { name: 'kₐ', what: 'attractive strength — pull toward goal' },
            { name: 'kᵣ', what: 'repulsive strength — push from obstacles' },
            { name: 'ρ₀', what: 'how far obstacles can be "felt"; bigger = more cautious' }
          ] : [
            { name: 'ε step', what: 'how far to extend per iteration; smaller = denser tree but slower' }
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

          {/* Obstacles */}
          {obstacles.map((o, i) => {
            const [cx, cy] = toS(o.x, o.y);
            return (
              <g key={i}>
                <circle cx={cx} cy={cy} r={o.r * scale}
                  fill="var(--danger)" fillOpacity="0.18"
                  stroke="var(--danger)" strokeWidth="1" />
                {method === 'apf' && (
                  <circle cx={cx} cy={cy} r={(o.r + rho0) * scale}
                    fill="none" stroke="var(--danger)" strokeWidth="0.5"
                    strokeDasharray="3 3" opacity="0.5" />
                )}
              </g>
            );
          })}

          {/* RRT tree */}
          {method === 'rrt' && rrtNodes.map((n, i) => {
            if (n.parent < 0) return null;
            const [ax, ay] = toS(rrtNodes[n.parent].p[0], rrtNodes[n.parent].p[1]);
            const [bx, by] = toS(n.p[0], n.p[1]);
            return <line key={i} x1={ax} y1={ay} x2={bx} y2={by}
              stroke="var(--text-faint)" strokeWidth="0.7" opacity="0.7" />;
          })}
          {method === 'rrt' && rrtNodes.map((n, i) => {
            const [px2, py2] = toS(n.p[0], n.p[1]);
            return <circle key={i} cx={px2} cy={py2} r="1.5" fill="var(--accent-2)" />;
          })}

          {/* RRT path */}
          {method === 'rrt' && rrtFound && rrtPath.length > 1 && (
            <path
              d={rrtPath.map((p, i) => {
                const [sx, sy] = toS(p[0], p[1]);
                return `${i === 0 ? 'M' : 'L'} ${sx} ${sy}`;
              }).join(' ')}
              fill="none" stroke="var(--warn)" strokeWidth="2.5" />
          )}

          {/* APF trail */}
          {method === 'apf' && trail.length > 1 && (
            <path
              d={trail.map((p, i) => {
                const [tx, ty] = toS(p[0], p[1]);
                return `${i === 0 ? 'M' : 'L'} ${tx} ${ty}`;
              }).join(' ')}
              fill="none" stroke="var(--accent-2)" strokeWidth="1.5" />
          )}

          {/* Goal */}
          {(() => {
            const [gx, gy] = toS(goal[0], goal[1]);
            return (
              <g>
                <circle cx={gx} cy={gy} r="10" fill="none" stroke="var(--warn)" strokeWidth="1.5" />
                <circle cx={gx} cy={gy} r="3" fill="var(--warn)" />
                <text x={gx + 12} y={gy + 4} fill="var(--warn)" fontSize="10" fontFamily="var(--mono)">goal</text>
              </g>
            );
          })()}

          {/* Start */}
          {(() => {
            const [sx, sy] = toS(start[0], start[1]);
            return (
              <g>
                <circle cx={sx} cy={sy} r="5" fill="var(--plot-2)" />
                <text x={sx + 8} y={sy + 4} fill="var(--plot-2)" fontSize="10" fontFamily="var(--mono)">start</text>
              </g>
            );
          })()}

          {/* APF current position */}
          {method === 'apf' && (() => {
            const [pxs, pys] = toS(pos[0], pos[1]);
            return <circle cx={pxs} cy={pys} r="7" fill="var(--plot-1)" stroke="var(--text)" strokeWidth="1" />;
          })()}
        </svg>

        <div className="metrics">
          {method === 'apf' && (
            <>
              <div className="metric"><div className="label">pos</div><div className="value">{pos[0].toFixed(2)},{pos[1].toFixed(2)}</div></div>
              <div className="metric"><div className="label">to goal</div><div className="value">{Math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2).toFixed(2)}</div></div>
              <div className="metric"><div className="label">trail pts</div><div className="value">{trail.length}</div></div>
            </>
          )}
          {method === 'rrt' && (
            <>
              <div className="metric"><div className="label">nodes</div><div className="value">{rrtNodes.length}</div></div>
              <div className="metric"><div className="label">found</div>
                <div className="value" style={{ color: rrtFound ? 'var(--accent-2)' : 'var(--text-faint)' }}>
                  {rrtFound ? 'YES' : 'no'}
                </div>
              </div>
              {rrtFound && <div className="metric"><div className="label">path len</div><div className="value">{rrtPath.length}</div></div>}
            </>
          )}
        </div>

        <div className="btn-row" style={{ marginTop: 12 }}>
          <button className="btn primary" onClick={() => setRunning(r => !r)}>
            {running ? 'Pause' : 'Plan'}
          </button>
          <button className="btn" onClick={resetAll}>Reset</button>
        </div>
      </div>

      <div className="controls">
        <h3>Method</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${method === 'apf' ? 'primary' : ''}`}
            onClick={() => { setMethod('apf'); resetAll(); }}>Potential field</button>
          <button className={`btn ${method === 'rrt' ? 'primary' : ''}`}
            onClick={() => { setMethod('rrt'); resetAll(); }}>RRT</button>
        </div>

        {method === 'apf' && (
          <>
            <h3>Potential gains</h3>
            <Slider label="kₐ attractive" value={ka} onChange={setKa} min={0.5} max={10} step={0.1} precision={1} />
            <Slider label="kᵣ repulsive" value={kr} onChange={setKr} min={5} max={300} step={1} precision={0} />
            <Slider label="ρ₀ influence" value={rho0} onChange={setRho0} min={0.1} max={1.2} step={0.02} />
          </>
        )}
        {method === 'rrt' && (
          <>
            <h3>RRT step</h3>
            <Slider label="ε step size" value={stepRRT} onChange={setStepRRT} min={0.05} max={0.6} step={0.01} />
          </>
        )}
      </div>
    </div>
  );
}
