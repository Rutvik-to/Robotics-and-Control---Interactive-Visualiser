import React, { useState, useMemo } from 'react';
import Slider from './Slider.jsx';
import Plot from './Plot.jsx';
import Explainer from './Explainer.jsx';
import { simulate } from '../utils/math.js';

/**
 * Siciliano §8.5.3 (Robust) and §8.5.4 (Adaptive) control.
 *
 *  TRUE plant:   I_true q̈ + b q̇ + m g lc sin(q) = τ
 *  MODEL used:   I_hat (parameter we think is the inertia)
 *
 *  Robust (sliding-mode style, simplified):
 *    s = ė + λe                         (sliding surface)
 *    τ = I_hat (q̈d + λ ė) + g_hat(q) + K_s · sat(s/φ)
 *    where sat is saturation (boundary-layer to avoid chatter).
 *
 *  Adaptive:
 *    τ = θ̂ · Y(q, q̇, q̈d)               (regressor form, Siciliano eq. 8.93)
 *    θ̂̇ = −Γ Yᵀ s                       (Slotine-Li update law)
 *    Y here = [q̈d + λė, sin(q)]ᵀ  parameterizing inertia and gravity gain.
 */

function plant(q, qdot, I_true, b, mglc) {
  // Returns q̈ given torque
  return (tau) => (tau - b * qdot - mglc * Math.sin(q)) / I_true;
}

export default function RobustAdaptive() {
  const [mode, setMode] = useState('robust'); // 'robust' | 'adaptive'
  const [I_true, setI_true] = useState(1.5);
  const [I_hat, setI_hat] = useState(0.8);   // model mismatch
  const [Ks, setKs] = useState(5);
  const [phi, setPhi] = useState(0.1);
  const [lam, setLam] = useState(8);
  const [gam, setGam] = useState(20);
  const [qd, setQd] = useState(1.2);
  const [trackSine, setTrackSine] = useState(true);

  const b = 0.3, mglc = 4.0; // true gravity gain m·g·lc

  const sim = useMemo(() => {
    const tFinal = 5.0, dt = 0.002;

    // Reference trajectory: constant qd OR a sine wave (adaptive needs excitation)
    const ref = (t) => {
      if (mode === 'adaptive' && trackSine) {
        return {
          q: qd + 0.5 * Math.sin(1.5 * t),
          qd: 0.75 * Math.cos(1.5 * t),
          qdd: -1.125 * Math.sin(1.5 * t)
        };
      }
      return { q: qd, qd: 0, qdd: 0 };
    };

    // State: [q, qdot, theta1_hat (inertia), theta2_hat (grav gain)]
    const f = (t, x) => {
      const [q, qdot, t1, t2] = x;
      const r = ref(t);
      const e = r.q - q;
      const edot = r.qd - qdot;
      const s = edot + lam * e;

      // Regressor: Y = [q̈d + λė, sin(q)]
      const Y = [r.qdd + lam * edot, Math.sin(q)];

      let tau;
      let theta_dot = [0, 0];

      if (mode === 'robust') {
        const tauNom = I_hat * (r.qdd + lam * edot) + mglc * Math.sin(q);
        const sat = Math.abs(s) <= phi ? s / phi : Math.sign(s);
        tau = tauNom + Ks * sat;
      } else {
        tau = t1 * Y[0] + t2 * Y[1];
        theta_dot = [gam * Y[0] * s, gam * Y[1] * s];
      }

      const qddot = (tau - b * qdot - mglc * Math.sin(q)) / I_true;
      return [qdot, qddot, theta_dot[0], theta_dot[1]];
    };

    const x0 = mode === 'adaptive' ? [0.3, 0, 0.2, 1.0] : [0, 0, I_hat, mglc];
    return simulate(f, x0, 0, tFinal, dt);
  }, [mode, I_true, I_hat, Ks, phi, lam, gam, qd, trackSine]);

  // Reference for plotting
  const refSeriesData = useMemo(() => {
    return sim.map(s => {
      if (mode === 'adaptive' && trackSine) {
        return [s.t, qd + 0.5 * Math.sin(1.5 * s.t)];
      }
      return [s.t, qd];
    });
  }, [sim, mode, trackSine, qd]);

  const qSeries = { name: 'q', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, s.x[0]]) };
  const refSeries = { name: 'qd', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: refSeriesData };

  const t1Series = { name: 'θ̂₁', color: 'var(--plot-1)', width: 2,
    points: sim.map(s => [s.t, s.x[2]]) };
  const t1True = { name: 'I_true', color: 'var(--plot-1)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, I_true]) };
  const t2Series = { name: 'θ̂₂', color: 'var(--plot-2)', width: 2,
    points: sim.map(s => [s.t, s.x[3]]) };
  const t2True = { name: 'mglc', color: 'var(--plot-2)', width: 1, dash: '5 4',
    points: sim.map(s => [s.t, mglc]) };

  const fin = sim[sim.length - 1].x;
  const err = Math.abs(qd - fin[0]);

  return (
    <div className="panel two-col">
      <div className="viz-card">
        <h3>Robust vs adaptive control under model mismatch</h3>
        <p className="eq">
          {mode === 'robust'
            ? 'τ = Î(q̈d + λė) + ĝ(q) + Kₛ·sat(s/φ),  s = ė + λe'
            : 'τ = θ̂ᵀ Y(q,q̇,q̈d),  θ̂̇ = Γ Yᵀ s'}
        </p>

        <Explainer
          title={mode === 'robust' ? 'Robust control — fight uncertainty by force' : 'Adaptive control — learn the right parameters'}
          plain={mode === 'robust'
            ? "Imagine pushing a box but you don't know if it weighs 1kg or 5kg. A robust controller assumes worst-case and uses a big extra 'kick' (Kₛ) to overpower any uncertainty. It works, but wastes energy and can vibrate (chatter). The saturation boundary φ smooths out the kick to avoid chattering."
            : "Same box, but now the controller measures how it's responding and updates its guess of the weight in real time. The estimates θ̂₁ (inertia) and θ̂₂ (gravity load) start wrong, but climb toward the true values as the arm moves — like learning by doing."}
          equation={mode === 'robust'
            ? "τ = Î(q̈d + λė) + ĝ(q) + Kₛ·sat(s/φ)"
            : "τ = θ̂₁·(q̈d + λė) + θ̂₂·sin(q),   θ̂̇ = Γ·Yᵀs"}
          equationNote={mode === 'robust'
            ? "Siciliano eq. 8.72 — first two terms are the best-guess model torque; the last term is the 'just in case' switching action."
            : "Siciliano eq. 8.93 — θ̂ is updated continuously to drive the sliding variable s → 0."}
          knobs={mode === 'robust' ? [
            { name: 'I_true', what: 'the real inertia (the controller doesn\'t know this)' },
            { name: 'Î', what: 'what the controller thinks the inertia is — mismatch creates error' },
            { name: 'Kₛ', what: 'how hard the robust kick is — bigger = faster but more chatter' },
            { name: 'φ', what: 'softening band around s=0 — wider = smoother but slightly imprecise' },
            { name: 'λ', what: 'how quickly e is forced to zero once on the sliding surface' }
          ] : [
            { name: 'Γ (gain)', what: 'learning rate — bigger means estimates update faster' },
            { name: 'I_true', what: 'real inertia — watch θ̂₁ converge to this value' },
            { name: 'λ', what: 'desired closed-loop error decay rate' }
          ]} />

        <Plot
          series={[qSeries, refSeries]}
          xLabel="t (s)"
          yLabel="q (rad)"
          xRange={[0, 5]}
          width={640}
          height={180}
        />
        {mode === 'adaptive' && (
          <>
            <p style={{ fontSize: 12, color: 'var(--text-faint)', margin: '8px 0 4px', fontFamily: 'var(--mono)' }}>
              parameter estimates converging to truth
            </p>
            <Plot
              series={[t1Series, t1True, t2Series, t2True]}
              xLabel="t (s)"
              yLabel="θ̂"
              xRange={[0, 5]}
              width={640}
              height={150}
            />
          </>
        )}

        <div className="metrics">
          <div className="metric"><div className="label">|e| final</div><div className="value">{err.toFixed(3)}</div></div>
          {mode === 'adaptive' && (
            <>
              <div className="metric"><div className="label">θ̂₁ → I</div><div className="value">{fin[2].toFixed(2)}/{I_true.toFixed(2)}</div></div>
              <div className="metric"><div className="label">θ̂₂ → mglc</div><div className="value">{fin[3].toFixed(2)}/{mglc.toFixed(2)}</div></div>
            </>
          )}
          {mode === 'robust' && (
            <div className="metric"><div className="label">model err</div><div className="value">{Math.abs(I_true - I_hat).toFixed(2)}</div></div>
          )}
        </div>
      </div>

      <div className="controls">
        <h3>Mode</h3>
        <div className="btn-row" style={{ marginBottom: 16 }}>
          <button className={`btn ${mode === 'robust' ? 'primary' : ''}`} onClick={() => setMode('robust')}>Robust</button>
          <button className={`btn ${mode === 'adaptive' ? 'primary' : ''}`} onClick={() => setMode('adaptive')}>Adaptive</button>
        </div>

        <h3>Common</h3>
        <Slider label="qd setpoint" value={qd} onChange={setQd} min={-2.5} max={2.5} step={0.05} />
        <Slider label="I_true" value={I_true} onChange={setI_true} min={0.3} max={3.0} step={0.05} />
        <Slider label="λ (slope)" value={lam} onChange={setLam} min={1} max={20} step={0.5} precision={1} />

        {mode === 'robust' && (
          <>
            <h3 style={{ marginTop: 16 }}>Robust gains</h3>
            <Slider label="Î (model)" value={I_hat} onChange={setI_hat} min={0.1} max={3.0} step={0.05} />
            <Slider label="Kₛ" value={Ks} onChange={setKs} min={0} max={30} step={0.5} precision={1} />
            <Slider label="φ boundary" value={phi} onChange={setPhi} min={0.001} max={0.5} step={0.005} precision={3} />
          </>
        )}
        {mode === 'adaptive' && (
          <>
            <h3 style={{ marginTop: 16 }}>Adaptation</h3>
            <Slider label="Γ learning rate" value={gam} onChange={setGam} min={1} max={200} step={1} precision={0} />
            <div className="btn-row" style={{ marginTop: 12 }}>
              <button className={`btn ${trackSine ? 'primary' : ''}`}
                onClick={() => setTrackSine(!trackSine)}>
                Sine reference: {trackSine ? 'ON' : 'OFF'}
              </button>
            </div>
            <p style={{ fontSize: 11, color: 'var(--text-faint)', marginTop: 8, lineHeight: 1.4 }}>
              Sine reference provides persistent excitation — without it, parameters track but don't converge to true values (Siciliano §8.5.4).
            </p>
          </>
        )}
      </div>
    </div>
  );
}
