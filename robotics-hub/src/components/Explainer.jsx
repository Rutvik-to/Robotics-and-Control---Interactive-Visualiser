import React, { useState } from 'react';

/**
 * Collapsible "What is this?" panel.
 * - Opens to plain-language explanation, the equation, and what each slider does.
 * - Stays collapsed by default so the interactive view dominates.
 */
export default function Explainer({ title = 'What is this?', plain, equation, equationNote, knobs = [] }) {
  const [open, setOpen] = useState(false);

  return (
    <div style={{
      background: 'var(--bg-soft)',
      border: '0.5px solid var(--border)',
      borderRadius: 'var(--radius)',
      marginBottom: 16,
      overflow: 'hidden'
    }}>
      <button
        onClick={() => setOpen(o => !o)}
        style={{
          width: '100%',
          background: 'transparent',
          border: 'none',
          color: 'var(--text)',
          padding: '10px 14px',
          fontSize: 13,
          fontFamily: 'inherit',
          cursor: 'pointer',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          textAlign: 'left'
        }}>
        <span><span style={{ color: 'var(--accent)', marginRight: 8 }}>{open ? '▾' : '▸'}</span>{title}</span>
        <span style={{ fontSize: 11, color: 'var(--text-faint)', fontFamily: 'var(--mono)' }}>
          {open ? 'hide' : 'show'}
        </span>
      </button>

      {open && (
        <div style={{ padding: '4px 14px 14px', borderTop: '0.5px solid var(--border)' }}>
          <p style={{ fontSize: 13, color: 'var(--text-dim)', lineHeight: 1.6, margin: '12px 0 10px' }}>
            {plain}
          </p>
          {equation && (
            <div style={{
              background: 'var(--bg)',
              border: '0.5px solid var(--border)',
              borderRadius: 'var(--radius)',
              padding: '10px 12px',
              marginBottom: 10
            }}>
              <div style={{
                fontFamily: 'var(--mono)',
                fontSize: 13,
                color: 'var(--accent-2)',
                marginBottom: equationNote ? 4 : 0
              }}>{equation}</div>
              {equationNote && (
                <div style={{ fontSize: 12, color: 'var(--text-faint)', lineHeight: 1.5 }}>
                  {equationNote}
                </div>
              )}
            </div>
          )}
          {knobs.length > 0 && (
            <div>
              <div style={{
                fontSize: 11,
                color: 'var(--text-faint)',
                textTransform: 'uppercase',
                letterSpacing: '0.05em',
                fontFamily: 'var(--mono)',
                marginBottom: 6
              }}>
                What the sliders do
              </div>
              <ul style={{
                margin: 0,
                paddingLeft: 18,
                fontSize: 12.5,
                color: 'var(--text-dim)',
                lineHeight: 1.6
              }}>
                {knobs.map((k, i) => (
                  <li key={i}>
                    <code style={{
                      fontFamily: 'var(--mono)',
                      color: 'var(--accent)',
                      fontSize: 12
                    }}>{k.name}</code> — {k.what}
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
