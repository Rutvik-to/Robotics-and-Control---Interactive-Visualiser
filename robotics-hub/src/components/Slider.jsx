import React from 'react';

export default function Slider({ label, value, onChange, min, max, step = 0.01, unit = '', precision = 2 }) {
  return (
    <div className="control">
      <div className="row">
        <label>{label}</label>
        <span className="val">{Number(value).toFixed(precision)}{unit}</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={e => onChange(parseFloat(e.target.value))}
      />
    </div>
  );
}
