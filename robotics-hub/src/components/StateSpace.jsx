import React, { useState, useMemo } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate, eig2 } from '../utils/math.js';

/**
 * Second-order linear system in state-space form:
 *   ẋ = A x + B u
 *   A = [[0, 1], [-ωn², -2ζωn]]   (closed-loop with full-state feedback that places poles at -ζωn ± jωn√(1-ζ²))
 *
 * This mirrors the canonical second-order analysis in Siciliano Appendix B and §5.6
 * (used throughout joint-space and operational-space control design).
 *
 * We let the user pick (ωn, ζ) → eigenvalues live on s-plane;
 * unit step response shows the time-domain behaviour.
 */
export default function StateSpace() {
  const [wn, setWn] = useState(3.0);
  const [zeta, setZeta] = useState(0.7);
  const [showSecond, setShowSecond] = useState(false);
  const [wn2, setWn2] = useState(5.0);
  const [zeta2, setZeta2] = useState(0.3);

  const A = (wn, z) => [[0, 1], [-wn * wn, -2 * z * wn]];
  const eigs1 = useMemo(() => eig2(A(wn, zeta)), [wn, zeta]);
  const eigs2 = useMemo(() => eig2(A(wn2, zeta2)), [wn2, zeta2]);

  // Simulate step response: ẋ = Ax + B·1, output y = x[0]
  const simStep = (wn, z) => {
    const f = (t, x) => {
      const u = 1.0;
      const ax = [x[1], -wn * wn * x[0] - 2 * z * wn * x[1] + wn * wn * u];
      // We use B = [0, ωn²] so steady-state y = 1
      return ax;
    };
    return simulate(f, [0, 0], 0, 8.0, 0.01);
  };

  const sim1 = useMemo(() => simStep(wn, zeta), [wn, zeta]);
  const sim2 = useMemo(() => simStep(wn2, zeta2), [wn2, zeta2]);

  const series = [
    { name: 'sys 1', color: 'var(--plot-1)', width: 2, points: sim1.map(s => [s.t, s.x[0]]) },
    { name: 'ref', color: 'var(--text-faint)', width: 1, dash: '4 3', points: [[0, 1], [8, 1]] }
  ];
  if (showSecond) {
    series.push({ name: 'sys 2', color: 'var(--plot-4)', width: 2, points: sim2.map(s => [s.t, s.x[0]]) });
  }

  // Predicted analytical metrics
  const overshoot = zeta < 1 ? Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) * 100 : 0;
  const tSettle = 4 / (zeta * wn); // 2% settling time approx
  const tPeak = zeta < 1 ? Math.PI / (wn * Math.sqrt(1 - zeta * zeta)) : Infinity;

  // s-plane plot
  const PW = 320, PH = 280;
  const sMax = 8;
  const sx = (re) => PW / 2 + (re / sMax) * (PW / 2 - 20);
  const sy = (im) => PH / 2 - (im / sMax) * (PH / 2 - 20);

  // damping ratio lines (cones at constant ζ)
  const dampLines = [0.2, 0.4, 0.6, 0.8].map(z => {
    const ang = Math.acos(z); // angle from negative real axis
    return { z, x: -sMax * Math.cos(ang), y: sMax * Math.sin(ang) };
  });

  const stable1 = eigs1.every(e => e.re < 0);
  const stable2 = eigs2.every(e => e.re < 0);

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Pole placement & step response</h3>
        <p className="eq">ẋ = Ax + Bu &nbsp;·&nbsp; A = [[0, 1], [−ωₙ², −2ζωₙ]] &nbsp;·&nbsp; λ = −ζωₙ ± jωₙ√(1−ζ²)</p>

        <Explainer
          title="Why poles matter"
          plain="Every linear system has 'poles' (eigenvalues) — they're the system's fingerprint. If both poles are in the left half of the s-plane (negative real part), the system is stable: small disturbances die out. Right half = unstable: errors grow forever. The natural frequency ωₙ controls how fast the system responds; the damping ratio ζ controls oscillation. ζ < 1 = bouncy (underdamped), ζ = 1 = critically damped (fastest non-oscillating), ζ > 1 = sluggish (overdamped). Control engineers 'place' poles by choosing gains."
          equation="λ = −ζωₙ ± jωₙ√(1−ζ²)"
          equationNote="Siciliano App. B. A pole's real part is the decay rate; imaginary part is the oscillation frequency. Pure real poles = no oscillation. Complex pair = oscillatory."
          knobs={[
            { name: 'ωₙ', what: 'natural frequency — how fast the system responds; radius in s-plane' },
            { name: 'ζ', what: 'damping ratio — controls overshoot; ζ ≈ 0.7 is the engineering sweet spot' },
            { name: 'Compare', what: 'overlay a second system to see how pole positions translate to time-domain shape' }
          ]} />

        <div style={{ display: 'grid', gridTemplateColumns: '320px 1fr', gap: 16, alignItems: 'start' }}>
          {/* s-plane */}
          <svg width={PW} height={PH} viewBox={`0 0 ${PW} ${PH}`}
            style={{ background: 'var(--bg-soft)', borderRadius: 'var(--radius)' }}>
            {/* Stable region shading */}
            <rect x="0" y="0" width={PW / 2} height={PH} fill="var(--plot-3)" fillOpacity="0.04" />
            <text x={PW / 4} y={20} textAnchor="middle" fill="var(--plot-3)" fontSize="10" fontFamily="var(--mono)">stable</text>
            <text x={3 * PW / 4} y={20} textAnchor="middle" fill="var(--danger)" fontSize="10" fontFamily="var(--mono)">unstable</text>

            {/* Damping lines */}
            {dampLines.map((d, i) => (
              <g key={i}>
                <line x1={PW / 2} y1={PH / 2} x2={sx(d.x)} y2={sy(d.y)}
                  stroke="var(--grid)" strokeWidth="0.5" strokeDasharray="2 3" />
                <line x1={PW / 2} y1={PH / 2} x2={sx(d.x)} y2={sy(-d.y)}
                  stroke="var(--grid)" strokeWidth="0.5" strokeDasharray="2 3" />
                <text x={sx(d.x) - 2} y={sy(d.y) - 4} fill="var(--text-faint)" fontSize="9" fontFamily="var(--mono)" textAnchor="end">
                  ζ={d.z}
                </text>
              </g>
            ))}

            {/* Natural-freq circle (current) */}
            <circle cx={PW / 2} cy={PH / 2} r={(wn / sMax) * (PW / 2 - 20)}
              fill="none" stroke="var(--plot-1)" strokeWidth="0.5" strokeDasharray="3 3" opacity="0.5" />

            {/* Axes */}
            <line x1={0} y1={PH / 2} x2={PW} y2={PH / 2} stroke="var(--border)" strokeWidth="1" />
            <line x1={PW / 2} y1={0} x2={PW / 2} y2={PH} stroke="var(--border)" strokeWidth="1" />
            <text x={PW - 8} y={PH / 2 - 4} textAnchor="end" fill="var(--text-dim)" fontSize="10" fontFamily="var(--mono)">Re</text>
            <text x={PW / 2 + 6} y={12} fill="var(--text-dim)" fontSize="10" fontFamily="var(--mono)">Im</text>

            {/* Eigenvalues — system 1 */}
            {eigs1.map((e, i) => (
              <g key={`e1${i}`}>
                <line x1={sx(e.re) - 7} y1={sy(e.im) - 7} x2={sx(e.re) + 7} y2={sy(e.im) + 7}
                  stroke="var(--plot-1)" strokeWidth="2" />
                <line x1={sx(e.re) - 7} y1={sy(e.im) + 7} x2={sx(e.re) + 7} y2={sy(e.im) - 7}
                  stroke="var(--plot-1)" strokeWidth="2" />
              </g>
            ))}

            {/* Eigenvalues — system 2 */}
            {showSecond && eigs2.map((e, i) => (
              <g key={`e2${i}`}>
                <line x1={sx(e.re) - 7} y1={sy(e.im) - 7} x2={sx(e.re) + 7} y2={sy(e.im) + 7}
                  stroke="var(--plot-4)" strokeWidth="2" />
                <line x1={sx(e.re) - 7} y1={sy(e.im) + 7} x2={sx(e.re) + 7} y2={sy(e.im) - 7}
                  stroke="var(--plot-4)" strokeWidth="2" />
              </g>
            ))}
          </svg>

          {/* Step response */}
          <Plot
            series={series}
            xLabel="t (s)"
            yLabel="y(t)"
            xRange={[0, 8]}
            width={420}
            height={PH}
          />
        </div>

        <div className="legend" style={{ marginTop: 12 }}>
          <span><span className="swatch" style={{ background: 'var(--plot-1)' }} />sys 1: ωₙ={wn.toFixed(2)}, ζ={zeta.toFixed(2)}</span>
          {showSecond && <span><span className="swatch" style={{ background: 'var(--plot-4)' }} />sys 2: ωₙ={wn2.toFixed(2)}, ζ={zeta2.toFixed(2)}</span>}
          <span style={{ color: stable1 ? 'var(--plot-3)' : 'var(--danger)' }}>
            sys 1: {stable1 ? 'stable' : 'UNSTABLE'}
          </span>
        </div>

        <div className="metrics">
          <div className="metric"><div className="label">overshoot</div><div className="value">{overshoot.toFixed(1)}%</div></div>
          <div className="metric"><div className="label">t peak</div><div className="value">{isFinite(tPeak) ? tPeak.toFixed(2) + 's' : '—'}</div></div>
          <div className="metric"><div className="label">t settle 2%</div><div className="value">{tSettle.toFixed(2)}s</div></div>
          <div className="metric"><div className="label">λ real</div><div className="value">{eigs1[0].re.toFixed(2)}</div></div>
        </div>

        <p className="note">
          Sliders move the eigenvalues on the s-plane. <code>ζ &lt; 1</code> → complex conjugate pair → oscillatory step
          response. <code>ζ = 1</code> → critically damped (fastest non-oscillating). <code>ζ &gt; 1</code> → two real
          poles → overdamped. <code>ωₙ</code> sets the radial distance from origin: bigger = faster but more control effort.
          Lyapunov-stable iff every <code>Re(λ) &lt; 0</code> — the left half plane.
        </p>
      </div>

      <div className="controls">
        <h3>System 1</h3>
        <Slider label="ωₙ natural freq" value={wn} onChange={setWn} min={0.5} max={8} step={0.05} />
        <Slider label="ζ damping" value={zeta} onChange={setZeta} min={-0.3} max={2.0} step={0.01} />

        <h3 style={{ marginTop: 20 }}>Compare</h3>
        <div className="btn-row" style={{ marginBottom: 12 }}>
          <button className={`btn ${showSecond ? 'primary' : ''}`}
            onClick={() => setShowSecond(!showSecond)}>
            System 2: {showSecond ? 'ON' : 'OFF'}
          </button>
        </div>
        {showSecond && (
          <>
            <Slider label="ωₙ₂" value={wn2} onChange={setWn2} min={0.5} max={8} step={0.05} />
            <Slider label="ζ₂" value={zeta2} onChange={setZeta2} min={-0.3} max={2.0} step={0.01} />
          </>
        )}

        <div className="btn-row" style={{ marginTop: 16 }}>
          <button className="btn" onClick={() => { setWn(3); setZeta(0.7); }}>Critically tuned</button>
          <button className="btn" onClick={() => { setWn(5); setZeta(0.1); }}>Oscillatory</button>
          <button className="btn" onClick={() => { setWn(2); setZeta(-0.1); }}>Unstable</button>
        </div>
      </div>
    </div>
  );
}
