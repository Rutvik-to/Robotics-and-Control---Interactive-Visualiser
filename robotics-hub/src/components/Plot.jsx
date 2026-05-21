import React, { useMemo } from 'react';

/**
 * Series-based SVG line plot.
 * props:
 *   series: [{ name, color, points: [[x,y], ...] }]
 *   xLabel, yLabel: strings
 *   width, height: numbers
 *   xRange, yRange: optional [min, max] (auto if omitted)
 *   markers: optional [{ x, y, color, label }]  (horizontal/vertical reference)
 *   hLines: optional [{ y, color, dash, label }]
 *   vLines: optional [{ x, color, dash, label }]
 */
export default function Plot({
  series, xLabel = '', yLabel = '', width = 640, height = 280,
  xRange, yRange, hLines = [], vLines = []
}) {
  const padL = 48, padR = 16, padT = 12, padB = 32;
  const plotW = width - padL - padR;
  const plotH = height - padT - padB;

  const { xMin, xMax, yMin, yMax } = useMemo(() => {
    if (xRange && yRange) {
      return { xMin: xRange[0], xMax: xRange[1], yMin: yRange[0], yMax: yRange[1] };
    }
    let xMn = Infinity, xMx = -Infinity, yMn = Infinity, yMx = -Infinity;
    series.forEach(s => s.points.forEach(([x, y]) => {
      if (x < xMn) xMn = x; if (x > xMx) xMx = x;
      if (y < yMn) yMn = y; if (y > yMx) yMx = y;
    }));
    if (!isFinite(xMn)) { xMn = 0; xMx = 1; }
    if (!isFinite(yMn)) { yMn = -1; yMx = 1; }
    const yPad = (yMx - yMn) * 0.1 || 0.5;
    return {
      xMin: xRange ? xRange[0] : xMn,
      xMax: xRange ? xRange[1] : xMx,
      yMin: yRange ? yRange[0] : yMn - yPad,
      yMax: yRange ? yRange[1] : yMx + yPad
    };
  }, [series, xRange, yRange]);

  const sx = x => padL + ((x - xMin) / (xMax - xMin || 1)) * plotW;
  const sy = y => padT + plotH - ((y - yMin) / (yMax - yMin || 1)) * plotH;

  // Generate gridlines
  const xTicks = niceTicks(xMin, xMax, 6);
  const yTicks = niceTicks(yMin, yMax, 5);

  return (
    <svg width="100%" viewBox={`0 0 ${width} ${height}`} style={{ display: 'block' }}>
      {/* Grid */}
      {xTicks.map((t, i) => (
        <line key={`xg${i}`} x1={sx(t)} y1={padT} x2={sx(t)} y2={padT + plotH}
          stroke="var(--grid)" strokeWidth="0.5" />
      ))}
      {yTicks.map((t, i) => (
        <line key={`yg${i}`} x1={padL} y1={sy(t)} x2={padL + plotW} y2={sy(t)}
          stroke="var(--grid)" strokeWidth="0.5" />
      ))}

      {/* Axes */}
      <line x1={padL} y1={padT} x2={padL} y2={padT + plotH} stroke="var(--border)" strokeWidth="1" />
      <line x1={padL} y1={padT + plotH} x2={padL + plotW} y2={padT + plotH} stroke="var(--border)" strokeWidth="1" />

      {/* Tick labels */}
      {xTicks.map((t, i) => (
        <text key={`xl${i}`} x={sx(t)} y={padT + plotH + 14} textAnchor="middle"
          fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">{fmtTick(t)}</text>
      ))}
      {yTicks.map((t, i) => (
        <text key={`yl${i}`} x={padL - 6} y={sy(t) + 3} textAnchor="end"
          fill="var(--text-faint)" fontSize="10" fontFamily="var(--mono)">{fmtTick(t)}</text>
      ))}

      {/* Reference lines */}
      {hLines.map((h, i) => (
        <g key={`h${i}`}>
          <line x1={padL} y1={sy(h.y)} x2={padL + plotW} y2={sy(h.y)}
            stroke={h.color || 'var(--text-faint)'} strokeWidth="1"
            strokeDasharray={h.dash || '4 4'} />
          {h.label && (
            <text x={padL + plotW - 4} y={sy(h.y) - 4} textAnchor="end"
              fill={h.color || 'var(--text-faint)'} fontSize="10" fontFamily="var(--mono)">
              {h.label}
            </text>
          )}
        </g>
      ))}
      {vLines.map((v, i) => (
        <line key={`v${i}`} x1={sx(v.x)} y1={padT} x2={sx(v.x)} y2={padT + plotH}
          stroke={v.color || 'var(--text-faint)'} strokeWidth="1"
          strokeDasharray={v.dash || '4 4'} />
      ))}

      {/* Series */}
      {series.map((s, i) => {
        if (s.points.length === 0) return null;
        const d = s.points.map((p, j) => `${j === 0 ? 'M' : 'L'} ${sx(p[0])} ${sy(p[1])}`).join(' ');
        return (
          <path key={`s${i}`} d={d} stroke={s.color} strokeWidth={s.width || 1.5}
            fill="none" strokeDasharray={s.dash || ''} strokeLinejoin="round" strokeLinecap="round" />
        );
      })}

      {/* Axis labels */}
      {xLabel && (
        <text x={padL + plotW / 2} y={height - 6} textAnchor="middle"
          fill="var(--text-dim)" fontSize="11" fontFamily="var(--mono)">{xLabel}</text>
      )}
      {yLabel && (
        <text x={12} y={padT + plotH / 2} textAnchor="middle"
          fill="var(--text-dim)" fontSize="11" fontFamily="var(--mono)"
          transform={`rotate(-90 12 ${padT + plotH / 2})`}>{yLabel}</text>
      )}
    </svg>
  );
}

function niceTicks(min, max, count) {
  const range = max - min;
  if (range <= 0) return [min];
  const rough = range / count;
  const mag = Math.pow(10, Math.floor(Math.log10(rough)));
  const norm = rough / mag;
  let step;
  if (norm < 1.5) step = mag;
  else if (norm < 3) step = 2 * mag;
  else if (norm < 7) step = 5 * mag;
  else step = 10 * mag;
  const ticks = [];
  const start = Math.ceil(min / step) * step;
  for (let v = start; v <= max + step * 0.001; v += step) ticks.push(v);
  return ticks;
}

function fmtTick(v) {
  if (v === 0) return '0';
  const a = Math.abs(v);
  if (a >= 1000 || a < 0.01) return v.toExponential(1);
  if (a >= 10) return v.toFixed(0);
  if (a >= 1) return v.toFixed(1);
  return v.toFixed(2);
}
