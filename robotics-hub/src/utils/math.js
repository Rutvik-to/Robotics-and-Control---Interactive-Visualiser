// Runge-Kutta 4 integrator for state-space ODEs
// f: (t, x) => dx/dt (arrays)
export function rk4Step(f, t, x, dt) {
  const k1 = f(t, x);
  const x2 = x.map((xi, i) => xi + 0.5 * dt * k1[i]);
  const k2 = f(t + 0.5 * dt, x2);
  const x3 = x.map((xi, i) => xi + 0.5 * dt * k2[i]);
  const k3 = f(t + 0.5 * dt, x3);
  const x4 = x.map((xi, i) => xi + dt * k3[i]);
  const k4 = f(t + dt, x4);
  return x.map((xi, i) => xi + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

// Simulate a system from t0 to t1, returning array of {t, x}
export function simulate(f, x0, t0, t1, dt) {
  const out = [];
  let x = [...x0];
  let t = t0;
  out.push({ t, x: [...x] });
  while (t < t1) {
    const step = Math.min(dt, t1 - t);
    x = rk4Step(f, t, x, step);
    t += step;
    out.push({ t, x: [...x] });
  }
  return out;
}

// 2x2 matrix inverse
export function inv2(M) {
  const [[a, b], [c, d]] = M;
  const det = a * d - b * c;
  if (Math.abs(det) < 1e-12) return null;
  return [[d / det, -b / det], [-c / det, a / det]];
}

// 2x2 matrix-vector multiply
export function matVec2(M, v) {
  return [M[0][0] * v[0] + M[0][1] * v[1], M[1][0] * v[0] + M[1][1] * v[1]];
}

// 2x2 matrix-matrix multiply
export function matMul2(A, B) {
  return [
    [A[0][0] * B[0][0] + A[0][1] * B[1][0], A[0][0] * B[0][1] + A[0][1] * B[1][1]],
    [A[1][0] * B[0][0] + A[1][1] * B[1][0], A[1][0] * B[0][1] + A[1][1] * B[1][1]]
  ];
}

// Eigenvalues of 2x2 matrix
export function eig2(M) {
  const [[a, b], [c, d]] = M;
  const tr = a + d;
  const det = a * d - b * c;
  const disc = tr * tr - 4 * det;
  if (disc >= 0) {
    const r = Math.sqrt(disc);
    return [{ re: (tr + r) / 2, im: 0 }, { re: (tr - r) / 2, im: 0 }];
  } else {
    const r = Math.sqrt(-disc);
    return [{ re: tr / 2, im: r / 2 }, { re: tr / 2, im: -r / 2 }];
  }
}

export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export const fmt = (v, n = 2) => Number.isFinite(v) ? v.toFixed(n) : '—';
