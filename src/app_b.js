
  /* ============================== graphs ================================= */
  const PLOT_COLORS = ['#5cc4d8', '#eeb25a', '#8fd68a', '#e98aa8', '#a897f0', '#f0d46a', '#6aa8f5', '#f08a5c', '#c4cfda', '#7fe0c8', '#d58ae9', '#b0d26a'];
  const PLOT_KINDS = [['q', 'Joint position vs time'], ['qd', 'Joint velocity vs time'], ['qdd', 'Joint acceleration vs time'], ['tau', 'Joint torque / force vs time'], ['eeXYZ', 'End-effector X, Y, Z vs time'],
    ['pathXY', 'End-effector path, X–Y plane'], ['pathXZ', 'End-effector path, X–Z plane'], ['pathYZ', 'End-effector path, Y–Z plane'], ['ikErr', 'IK error vs iteration'],
    ['wsXY', 'Workspace samples, X–Y projection'], ['wsXZ', 'Workspace samples, X–Z projection']];
  const PL = { spec: null, hover: null, legendKey: '', drag: null, lastDraw: 0 };
  const activeSeries = () => S.active === 'sim' && S.sim ? S.sim : S.traj;
  function ensureEE(d) { if (!d.ee) d.ee = d.q.map(q => RL.fkPos(S.robot, q)); return d.ee; }
  function ensureTau(d) {
    if (!d.tau) { try { d.tau = RL.trajectoryTorques(S.robot, d, gravityOf()); } catch (e) { d.tau = null; d.tauError = e.message; } }
    return d.tau;
  }
  function buildPlotSpec() {
    const kind = S.plot.kind, robot = S.robot, J = robot.joints, mixed = J.some(j => j.type === 'P') && J.some(j => j.type === 'R');
    const unitWord = (r, p) => J.every(j => j.type === 'R') ? r : J.every(j => j.type === 'P') ? p : r + ' (revolute) / ' + p + ' (prismatic)';
    const nameOf = (j, i, u) => 'q' + (i + 1) + ' ' + j.name + ' [' + u(j).trim() + ']';
    if (kind === 'ikErr') {
      const r = S.ik.result; if (!r) return { empty: 'No inverse-kinematics result yet — solve a target in the Inverse Kinematics tab.' };
      const x = r.history.map((_, i) => i), lg = S.plot.logy, tf = (v) => lg ? Math.log10(Math.max(v, 1e-16)) : v;
      const series = [{ name: 'Position error [m]', y: r.history.map(h => tf(h.pos)), x }]; if (r.useOri) series.push({ name: 'Orientation error [rad]', y: r.history.map(h => tf(h.ori)), x });
      return { title: 'IK convergence — ' + (r.converged ? 'converged' : 'NOT converged'), xLabel: 'Iteration', yLabel: lg ? 'log₁₀ error' : 'Error', series };
    }
    if (kind === 'wsXY' || kind === 'wsXZ') {
      if (!S.ws.pts || !S.ws.count) return { empty: 'No workspace samples yet — run the Workspace Analysis.' };
      const a = 0, b = kind === 'wsXY' ? 1 : 2, stride = Math.max(1, Math.ceil(S.ws.count / 8000)), x = [], y = [];
      for (let i = 0; i < S.ws.count; i += stride) { x.push(S.ws.pts[3 * i + a]); y.push(S.ws.pts[3 * i + b]); }
      return { title: 'Workspace samples (' + x.length + ' of ' + S.ws.count + ' shown)', xLabel: 'X [m]', yLabel: (kind === 'wsXY' ? 'Y' : 'Z') + ' [m]', series: [{ name: 'Reachable samples', x, y, mode: 'points', color: '#5cc4d8' }], equal: true };
    }
    const d = activeSeries();
    if (!d) return { empty: 'No time series yet — generate a trajectory (Trajectory Planning) or run a forward-dynamics simulation (Dynamics).' };
    const t = d.t;
    if (kind === 'q' || kind === 'qd' || kind === 'qdd') {
      const u = kind === 'q' ? uPos : kind === 'qd' ? uVel : uAcc;
      return { title: { q: 'Joint positions', qd: 'Joint velocities', qdd: 'Joint accelerations' }[kind] + ' — ' + (d.type || ''), xLabel: 'Time [s]',
        yLabel: kind === 'q' ? unitWord('Angle [°]', 'Displacement [m]') : kind === 'qd' ? unitWord('Angular velocity [°/s]', 'Velocity [m/s]') : unitWord('Angular acceleration [°/s²]', 'Acceleration [m/s²]'),
        series: J.map((j, i) => ({ name: nameOf(j, i, u), x: t, y: d[kind].map(r => toDisp(j, r[i])) })), time: true, note: d.velocityNote };
    }
    if (kind === 'tau') {
      const tau = ensureTau(d); if (!tau) return { empty: 'Could not compute torques: ' + (d.tauError || 'unknown error') };
      return { title: 'Joint torques / forces (inverse dynamics)', xLabel: 'Time [s]', yLabel: unitWord('Torque [N·m]', 'Force [N]'), series: J.map((j, i) => ({ name: nameOf(j, i, uTau), x: t, y: tau.map(r => r[i]) })), time: true };
    }
    if (kind === 'eeXYZ') { const ee = ensureEE(d); return { title: 'End-effector position', xLabel: 'Time [s]', yLabel: 'Position [m]', series: ['X', 'Y', 'Z'].map((n, k) => ({ name: n + ' [m]', x: t, y: ee.map(p => p[k]) })), time: true }; }
    if (kind.startsWith('path')) {
      const ee = ensureEE(d), ax = { pathXY: [0, 1], pathXZ: [0, 2], pathYZ: [1, 2] }[kind], nm = 'XYZ';
      const x = ee.map(p => p[ax[0]]), y = ee.map(p => p[ax[1]]);
      return { title: 'End-effector path', xLabel: nm[ax[0]] + ' [m]', yLabel: nm[ax[1]] + ' [m]', equal: true,
        series: [{ name: 'Path', x, y }, { name: 'Start', x: [x[0]], y: [y[0]], mode: 'points', color: '#77d3a0' }, { name: 'End', x: [x[x.length - 1]], y: [y[y.length - 1]], mode: 'points', color: '#f2807b' }] };
    }
    return { empty: 'Unknown plot.' };
  }
  function niceTicks(lo, hi, target) {
    if (!(hi > lo)) { hi = lo + 1; } const step = niceStep((hi - lo) / Math.max(2, target)), out = []; let v = Math.ceil(lo / step - 1e-9) * step;
    for (let k = 0; v <= hi + step * 1e-9 && k < 200; k++, v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v); return { ticks: out, step };
  }
  const tickLabel = (v, step) => { const dec = Math.max(0, Math.min(8, -Math.floor(Math.log10(step) + 1e-9))); return Math.abs(v) >= 1e5 || (Math.abs(v) < 1e-3 && v !== 0) ? v.toExponential(1) : v.toFixed(dec); };
  const cssVar = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || '#ccc';

  function plotBounds(spec) {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const s of spec.series) { if (S.plot.hidden[s.name]) continue; for (let i = 0; i < s.x.length; i++) { const x = s.x[i], y = s.y[i]; if (!Number.isFinite(x) || !Number.isFinite(y)) continue; if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; } }
    if (!Number.isFinite(x0)) { x0 = 0; x1 = 1; y0 = -1; y1 = 1; }
    if (x1 - x0 < 1e-12) { x0 -= 0.5; x1 += 0.5; } if (y1 - y0 < 1e-12) { y0 -= 0.5; y1 += 0.5; }
    const py = (y1 - y0) * 0.06; y0 -= py; y1 += py; if (spec.equal) { const px = (x1 - x0) * 0.06; x0 -= px; x1 += px; }
    return { x0, x1, y0, y1 };
  }
  function plotGeom(cv) { return { L: 64, R: 14, T: 24, B: 36, w: cv.clientWidth, h: cv.clientHeight }; }
  function plotView(spec, cv) {
    let v = S.plot.view || plotBounds(spec); const g = plotGeom(cv);
    if (spec.equal) { const pw = g.w - g.L - g.R, ph = g.h - g.T - g.B, sx = (v.x1 - v.x0) / pw, sy = (v.y1 - v.y0) / ph, s = Math.max(sx, sy), cx = (v.x0 + v.x1) / 2, cy = (v.y0 + v.y1) / 2; v = { x0: cx - s * pw / 2, x1: cx + s * pw / 2, y0: cy - s * ph / 2, y1: cy + s * ph / 2 }; }
    return v;
  }
  function renderLegend(spec) {
    const key = S.plot.kind + '|' + spec.series.map(s => s.name + (S.plot.hidden[s.name] ? '0' : '1')).join('|');
    if (key === PL.legendKey) return; PL.legendKey = key;
    $('#legend').innerHTML = spec.series.map((s, i) => '<span class="chip' + (S.plot.hidden[s.name] ? ' off' : '') + '" role="button" tabindex="0" data-series="' + esc(s.name) + '" title="Toggle series"><i style="background:' + (s.color || PLOT_COLORS[i % PLOT_COLORS.length]) + '"></i>' + esc(s.name) + '</span>').join('');
  }
  function drawPlot() {
    const cv = $('#plot'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'); if (!ctx) return;
    const w = cv.clientWidth, h = cv.clientHeight; if (w < 20 || h < 20) return; const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
    const text = cssVar('--text'), muted = cssVar('--muted'), line = cssVar('--line'); let spec; try { spec = buildPlotSpec(); } catch (e) { spec = { empty: 'Plot error: ' + e.message }; }
    PL.spec = spec; ctx.font = '12px ' + cssVar('--sans');
    $('#logy-wrap').hidden = S.plot.kind !== 'ikErr';
    if (spec.empty) { ctx.fillStyle = muted; ctx.textAlign = 'center'; ctx.fillText(spec.empty, w / 2, h / 2); ctx.textAlign = 'left'; $('#legend').innerHTML = ''; PL.legendKey = ''; return; }
    renderLegend(spec);
    const g = plotGeom(cv), v = plotView(spec, cv), pw = w - g.L - g.R, ph = h - g.T - g.B, X = (x) => g.L + (x - v.x0) / (v.x1 - v.x0) * pw, Y = (y) => g.T + (1 - (y - v.y0) / (v.y1 - v.y0)) * ph;
    PL.map = { g, v, pw, ph };
    const tx = niceTicks(v.x0, v.x1, pw / 80), ty = niceTicks(v.y0, v.y1, ph / 34);
    ctx.lineWidth = 1; ctx.strokeStyle = line; ctx.fillStyle = muted; ctx.textAlign = 'center';
    ctx.beginPath(); tx.ticks.forEach(t => { const x = Math.round(X(t)) + 0.5; ctx.moveTo(x, g.T); ctx.lineTo(x, g.T + ph); }); ty.ticks.forEach(t => { const y = Math.round(Y(t)) + 0.5; ctx.moveTo(g.L, y); ctx.lineTo(g.L + pw, y); }); ctx.stroke();
    tx.ticks.forEach(t => ctx.fillText(tickLabel(t, tx.step), X(t), g.T + ph + 14)); ctx.textAlign = 'right'; ty.ticks.forEach(t => ctx.fillText(tickLabel(t, ty.step), g.L - 6, Y(t) + 4));
    ctx.strokeStyle = muted; ctx.strokeRect(g.L + 0.5, g.T + 0.5, pw, ph);
    ctx.fillStyle = text; ctx.textAlign = 'center'; ctx.fillText(spec.xLabel, g.L + pw / 2, h - 6); ctx.fillText(spec.title, g.L + pw / 2, 14);
    ctx.save(); ctx.translate(13, g.T + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(spec.yLabel, 0, 0); ctx.restore();
    ctx.save(); ctx.beginPath(); ctx.rect(g.L, g.T, pw, ph); ctx.clip();
    spec.series.forEach((s, si) => {
      if (S.plot.hidden[s.name]) return; const col = s.color || PLOT_COLORS[si % PLOT_COLORS.length]; ctx.strokeStyle = col; ctx.fillStyle = col;
      if (s.mode === 'points') { const big = s.x.length < 5, rad = big ? 5 : 1.6; for (let i = 0; i < s.x.length; i++) { const x = X(s.x[i]), y = Y(s.y[i]); if (x < g.L - 6 || x > g.L + pw + 6) continue; ctx.beginPath(); ctx.arc(x, y, rad, 0, 6.2832); ctx.fill(); } return; }
      ctx.lineWidth = 1.8; ctx.beginPath(); let pen = false; for (let i = 0; i < s.x.length; i++) { if (!Number.isFinite(s.y[i])) { pen = false; continue; } const x = X(s.x[i]), y = Y(s.y[i]); if (pen) ctx.lineTo(x, y); else ctx.moveTo(x, y); pen = true; } ctx.stroke();
    });
    if (spec.time && S.play.data && (S.play.data === activeSeries())) { const x = X(S.play.t); ctx.strokeStyle = cssVar('--accent'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(x, g.T); ctx.lineTo(x, g.T + ph); ctx.stroke(); ctx.setLineDash([]); }
    ctx.restore();
    if (PL.hover && PL.hover.x >= g.L && PL.hover.x <= g.L + pw && PL.hover.y >= g.T && PL.hover.y <= g.T + ph) drawHover(ctx, spec, v, X, Y, g, pw, ph);
  }
  function drawHover(ctx, spec, v, X, Y, g, pw, ph) {
    const hx = PL.hover.x, hy = PL.hover.y, dx = v.x0 + (hx - g.L) / pw * (v.x1 - v.x0), rows = []; let ax = null;
    spec.series.forEach((s, si) => {
      if (S.plot.hidden[s.name] || !s.x.length) return; let best = -1, bd = Infinity;
      if (spec.time) { let lo = 0, hi = s.x.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (s.x[m] < dx) lo = m; else hi = m; } best = Math.abs(s.x[lo] - dx) < Math.abs(s.x[hi] - dx) ? lo : hi; }
      else { for (let i = 0; i < s.x.length; i++) { const d = (X(s.x[i]) - hx) ** 2 + (Y(s.y[i]) - hy) ** 2; if (d < bd) { bd = d; best = i; } } if (bd > 400) return; }
      if (best >= 0) { rows.push({ s, i: best, col: s.color || PLOT_COLORS[si % PLOT_COLORS.length] }); if (ax === null) ax = s.x[best]; }
    });
    if (!rows.length) return; const sx = X(ax);
    if (spec.time) { ctx.strokeStyle = cssVar('--muted'); ctx.beginPath(); ctx.moveTo(sx + 0.5, g.T); ctx.lineTo(sx + 0.5, g.T + ph); ctx.stroke(); }
    rows.forEach(r => { ctx.fillStyle = r.col; ctx.beginPath(); ctx.arc(X(r.s.x[r.i]), Y(r.s.y[r.i]), 3.5, 0, 6.2832); ctx.fill(); });
    const lines = [spec.xLabel.split(' [')[0] + ' = ' + fx(ax, 4)].concat(rows.map(r => r.s.name.split(' [')[0] + ' = ' + fx(r.s.y[r.i], 4))), bw = Math.max(...lines.map(l => ctx.measureText(l).width)) + 14, bh = lines.length * 15 + 8;
    let bx = sx + 10; if (bx + bw > g.L + pw) bx = sx - bw - 10; const by = Math.min(g.T + 4, g.T + ph - bh);
    ctx.fillStyle = cssVar('--panel2'); ctx.strokeStyle = cssVar('--line2'); ctx.fillRect(bx, by, bw, bh); ctx.strokeRect(bx + 0.5, by + 0.5, bw, bh); ctx.fillStyle = cssVar('--text'); ctx.textAlign = 'left';
    lines.forEach((l, i) => ctx.fillText(l, bx + 7, by + 16 + i * 15));
  }
  function initPlotInteraction() {
    const cv = $('#plot');
    const dataAt = (px, py) => { const m = PL.map; return m ? { x: m.v.x0 + (px - m.g.L) / m.pw * (m.v.x1 - m.v.x0), y: m.v.y0 + (1 - (py - m.g.T) / m.ph) * (m.v.y1 - m.v.y0) } : null; };
    cv.addEventListener('wheel', e => {
      if (!PL.map || !PL.spec || PL.spec.empty) return; e.preventDefault(); const rc = cv.getBoundingClientRect(), p = dataAt(e.clientX - rc.left, e.clientY - rc.top), v = PL.map.v, f = Math.exp(RL.clamp(e.deltaY, -200, 200) * 0.0015);
      const nv = { x0: v.x0, x1: v.x1, y0: v.y0, y1: v.y1 }; const zx = !e.shiftKey || PL.spec.equal, zy = e.shiftKey || PL.spec.equal || !PL.spec.time;
      if (zx) { nv.x0 = p.x - (p.x - v.x0) * f; nv.x1 = p.x + (v.x1 - p.x) * f; } if (zy) { nv.y0 = p.y - (p.y - v.y0) * f; nv.y1 = p.y + (v.y1 - p.y) * f; }
      S.plot.view = nv; dirty.plot = true;
    }, { passive: false });
    cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); PL.drag = { x: e.clientX, y: e.clientY, v: PL.map ? Object.assign({}, PL.map.v) : null }; });
    cv.addEventListener('pointermove', e => {
      const rc = cv.getBoundingClientRect(); PL.hover = { x: e.clientX - rc.left, y: e.clientY - rc.top };
      if (PL.drag && PL.drag.v && PL.map && (e.buttons & 1)) { const dx = (e.clientX - PL.drag.x) / PL.map.pw * (PL.drag.v.x1 - PL.drag.v.x0), dy = (e.clientY - PL.drag.y) / PL.map.ph * (PL.drag.v.y1 - PL.drag.v.y0);
        S.plot.view = { x0: PL.drag.v.x0 - dx, x1: PL.drag.v.x1 - dx, y0: PL.drag.v.y0 + dy, y1: PL.drag.v.y1 + dy }; }
      dirty.plot = true;
    });
    cv.addEventListener('pointerup', () => { PL.drag = null; }); cv.addEventListener('pointerleave', () => { PL.hover = null; dirty.plot = true; });
    cv.addEventListener('dblclick', () => { S.plot.view = null; dirty.plot = true; });
    $('#legend').addEventListener('click', e => { const c = e.target.closest('[data-series]'); if (!c) return; const n = c.dataset.series; S.plot.hidden[n] = !S.plot.hidden[n]; PL.legendKey = ''; dirty.plot = true; });
    $('#legend').addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); e.target.click(); } });
    $('#plot-kind').addEventListener('change', e => { S.plot.kind = e.target.value; S.plot.view = null; PL.legendKey = ''; dirty.plot = true; });
    $('#plot-log').addEventListener('change', e => { S.plot.logy = e.target.checked; S.plot.view = null; dirty.plot = true; });
    $('#plot-reset').addEventListener('click', () => { S.plot.view = null; dirty.plot = true; });
    $('#plot-csv').addEventListener('click', exportPlotCSV);
    new ResizeObserver(() => { dirty.plot = true; }).observe($('#plot-wrap'));
  }
  function refreshPlotKinds() { const sel = $('#plot-kind'); sel.innerHTML = PLOT_KINDS.map(k => '<option value="' + k[0] + '">' + esc(k[1]) + '</option>').join(''); sel.value = S.plot.kind; }
  function setPlotKind(k) { S.plot.kind = k; S.plot.view = null; PL.legendKey = ''; $('#plot-kind').value = k; dirty.plot = true; }
  function plotCSVRows() {
    const spec = buildPlotSpec(); if (spec.empty) return null; const ss = spec.series.filter(s => s.mode !== 'points' || s.x.length > 5);
    if (spec.time || ss.every(s => s.x === ss[0].x || s.x.length === ss[0].x.length && s.x.every((x, i) => x === ss[0].x[i]))) {
      const rows = [[spec.xLabel].concat(ss.map(s => s.name))]; for (let i = 0; i < ss[0].x.length; i++) rows.push([ss[0].x[i]].concat(ss.map(s => s.y[i]))); return rows;
    }
    const rows = [[]]; ss.forEach(s => rows[0].push(s.name + ' — ' + spec.xLabel, s.name + ' — ' + spec.yLabel)); const L = Math.max(...ss.map(s => s.x.length));
    for (let i = 0; i < L; i++) rows.push(ss.flatMap(s => [s.x[i], s.y[i]])); return rows;
  }
  function exportPlotCSV() { const r = plotCSVRows(); if (!r) { toast('Nothing to export — this plot has no data yet.', true); return; } download('robolab-' + S.plot.kind + '.csv', toCSV(r), 'text/csv'); }

  /* ============================== playback =============================== */
  function setPlayData(d, label, andPlay) {
    S.play.data = d; S.play.label = label; S.play.t = 0; S.play.playing = false; updatePlayUI(); poseAtTime(0); if (andPlay) togglePlay();
  }
  function poseAtTime(t) {
    const d = S.play.data; if (!d) return; const T = d.t[d.t.length - 1]; t = RL.clamp(t, 0, T); let lo = 0, hi = d.t.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (d.t[m] <= t) lo = m; else hi = m; }
    const span = d.t[hi] - d.t[lo], f = span > 0 ? RL.clamp((t - d.t[lo]) / span, 0, 1) : 0;
    S.q = d.q[lo].map((v, i) => v + (d.q[hi][i] - v) * f); S.play.t = t; dirty.pose = dirty.live = true; dirty.plot = true; updatePlayUI(true);
  }
  function advancePlay(dt) {
    const d = S.play.data; if (!d) { S.play.playing = false; return; } const T = d.t[d.t.length - 1]; let t = S.play.t + dt * S.play.speed;
    if (t >= T) { if (S.play.loop) { t = t % T; clearTrail(); } else { t = T; S.play.playing = false; updatePlayUI(); } }
    poseAtTime(t);
  }
  function togglePlay() {
    if (!S.play.data) { toast('Nothing to play yet — generate a trajectory or run a simulation first.'); return; }
    const T = S.play.data.t[S.play.data.t.length - 1]; if (!S.play.playing && S.play.t >= T - 1e-9) { S.play.t = 0; clearTrail(); }
    S.play.playing = !S.play.playing; if (S.play.playing && S.play.t === 0) clearTrail(); updatePlayUI();
  }
  function updatePlayUI(timeOnly) {
    const d = S.play.data, T = d ? d.t[d.t.length - 1] : 0;
    $('#time-readout').textContent = 't = ' + S.play.t.toFixed(3) + ' / ' + T.toFixed(2) + ' s';
    if (!timeOnly) { $('#pl-play').textContent = S.play.playing ? '⏸' : '▶'; $('#pl-scrub').disabled = !d; $('#pl-play').title = (S.play.playing ? 'Pause' : 'Play') + (S.play.label ? ' — ' + S.play.label : '') + ' (Space)'; }
    if (d && document.activeElement !== $('#pl-scrub')) $('#pl-scrub').value = T > 0 ? Math.round(S.play.t / T * 1000) : 0;
  }
  function initPlayback() {
    $('#pl-play').addEventListener('click', togglePlay);
    $('#pl-reset').addEventListener('click', () => { if (!S.play.data) return; S.play.playing = false; clearTrail(); poseAtTime(0); updatePlayUI(); });
    $('#pl-scrub').addEventListener('input', e => { const d = S.play.data; if (!d) return; S.play.playing = false; poseAtTime(+e.target.value / 1000 * d.t[d.t.length - 1]); updatePlayUI(); });
    $('#pl-speed').addEventListener('change', e => { S.play.speed = +e.target.value; });
    $('#pl-loop').addEventListener('change', e => { S.play.loop = e.target.checked; });
  }

  /* ============================= left sidebar ============================ */
  function jointCard(i) {
    const j = S.robot.joints[i], [lo, hi] = RL.jointLimitsSI(j), sc = RL.jointDisplayScale(j), step = j.type === 'R' ? 0.1 : 0.001;
    return '<div class="joint' + (j.type === 'P' ? ' pris' : '') + (i === S.sel ? ' sel' : '') + '" data-i="' + i + '" tabindex="0" role="group" aria-label="Joint ' + (i + 1) + ' ' + esc(j.name) + '">' +
      '<div class="head"><b>q<sub>' + (i + 1) + '</sub> ' + esc(j.name) + '</b><span class="muted">' + (j.type === 'R' ? 'revolute' : 'prismatic') + '</span></div>' +
      '<div class="row"><input type="range" id="js-' + i + '" min="' + (lo * sc) + '" max="' + (hi * sc) + '" step="' + step + '" value="' + (S.q[i] * sc) + '" aria-label="Joint ' + (i + 1) + ' slider">' +
      '<input type="number" id="jv-' + i + '" step="' + step + '" value="' + fx(S.q[i] * sc, j.type === 'R' ? 2 : 4) + '" aria-label="Joint ' + (i + 1) + ' value in ' + (j.type === 'R' ? 'degrees' : 'metres') + '"><span class="muted">' + uPos(j).trim() + '</span></div>' +
      '<div class="lims"><span>' + fx(lo * sc, j.type === 'R' ? 1 : 3) + '</span><span>limits</span><span>' + fx(hi * sc, j.type === 'R' ? 1 : 3) + '</span></div></div>';
  }
  function renderLeft() {
    const r = S.robot, opts = RL.PRESET_IDS.map(p => '<option value="' + p[0] + '">' + esc(p[1]) + '</option>').join('');
    $('#left').innerHTML = '<div class="sect"><h3>Robot</h3><div class="row"><select id="sel-preset" aria-label="Load an example robot"><option value="">Load example robot…</option>' + opts + '<option value="blank1">New blank robot</option></select></div>' +
      '<div><b>' + esc(r.name) + '</b> <span class="muted">· ' + r.joints.length + ' DOF</span></div><div class="note">' + esc(r.scope) + '</div>' +
      '<div class="row"><button class="btn small" data-a="home" title="Move to the stored home configuration">Home pose</button><button class="btn small" data-a="zero" title="All joints to zero (within limits)">Zero pose</button></div></div>' +
      '<div class="sect"><h3>Joint values</h3><div id="joint-list">' + r.joints.map((_, i) => jointCard(i)).join('') + '</div><p class="muted" style="margin:6px 0 0">Values are in degrees (revolute) or metres (prismatic). Sliders respect joint limits.</p></div>';
  }
  function renderLeftValues() {
    S.robot.joints.forEach((j, i) => { const s = $('#js-' + i), n = $('#jv-' + i), sc = RL.jointDisplayScale(j); if (s && document.activeElement !== s) s.value = S.q[i] * sc; if (n && document.activeElement !== n) n.value = fx(S.q[i] * sc, j.type === 'R' ? 2 : 4); });
  }
  function selectJoint(i) {
    S.sel = i; $$('#joint-list .joint').forEach(c => c.classList.toggle('sel', +c.dataset.i === i)); highlightSelected();
    if (S.mode === 'builder') renderPanel(); else if (S.mode === 'dh') $$('#dh-table tr[data-i]').forEach(r => r.classList.toggle('sel', +r.dataset.i === i));
  }
  function initLeft() {
    const L = $('#left');
    L.addEventListener('input', e => { const t = e.target; if (t.type === 'range' && t.id.startsWith('js-')) { const i = +t.id.slice(3), j = S.robot.joints[i], q = S.q.slice(); q[i] = fromDisp(j, +t.value); S.play.playing = false; setQ(q); const n = $('#jv-' + i); if (n) n.value = fx(+t.value, j.type === 'R' ? 2 : 4); } });
    L.addEventListener('change', e => {
      const t = e.target;
      if (t.id === 'sel-preset') { if (t.value) loadPreset(t.value); return; }
      if (t.type === 'number' && t.id.startsWith('jv-')) {
        const i = +t.id.slice(3), j = S.robot.joints[i], raw = parseFloat(t.value); if (!Number.isFinite(raw)) { t.classList.add('invalid'); toast('Enter a valid number.', true); renderLeftValues(); return; } t.classList.remove('invalid');
        const [lo, hi] = RL.jointLimitsSI(j), v = fromDisp(j, raw), c = RL.clamp(v, lo, hi); if (c !== v) toast('Value limited to joint range ' + fx(toDisp(j, lo), 2) + ' … ' + fx(toDisp(j, hi), 2) + uPos(j).trim(), false);
        const q = S.q.slice(); q[i] = c; S.play.playing = false; setQ(q); renderLeftValues(); const s = $('#js-' + i); if (s) s.value = toDisp(j, c);
      }
    });
    L.addEventListener('click', e => {
      const b = e.target.closest('[data-a]'); if (b) { S.play.playing = false; if (b.dataset.a === 'home') setQ(S.robot.home.slice()); else setQ(RL.clampQ(S.robot, S.q.map(() => 0))); renderLeftValues(); return; }
      const c = e.target.closest('.joint'); if (c) selectJoint(+c.dataset.i);
    });
    L.addEventListener('keydown', e => { if ((e.key === 'Enter' || e.key === ' ') && e.target.classList.contains('joint')) { e.preventDefault(); selectJoint(+e.target.dataset.i); } });
  }

  /* ============================== status / loop ========================== */
  function updateStatus() {
    const m = MODES.find(x => x[0] === S.mode), ee = S.fkCache ? RL.Pof(S.fkCache.T) : [0, 0, 0];
    $('#vp-status').innerHTML = '<b>' + esc(S.robot.name) + '</b> · ' + S.robot.joints.length + ' DOF · mode: ' + esc(m ? m[1] : '') + ' · EE (' + fx(ee[0], 3) + ', ' + fx(ee[1], 3) + ', ' + fx(ee[2], 3) + ') m';
  }
  let lastT = performance.now(), lastPlotDraw = 0;
  function frame(ts) {
    const dt = Math.min(0.1, Math.max(0, (ts - lastT) / 1000)); lastT = ts;
    try {
      if (S.play.playing) advancePlay(dt);
      if (dirty.pose) { dirty.pose = false; updateRobotPose(); dirty.live = true; }
      if (dirty.live) { dirty.live = false; renderLeftValues(); updateStatus(); if (LIVE[S.mode]) LIVE[S.mode](); }
      if (dirty.plot && (ts - lastPlotDraw > 30 || !S.play.playing)) { dirty.plot = false; lastPlotDraw = ts; drawPlot(); }
      if (G.ok && dirty.render) { dirty.render = false; renderGL(); }
    } catch (err) { console.error(err); if (!frame.reported) { frame.reported = true; toast('Unexpected error: ' + err.message, true); } }
    requestAnimationFrame(frame);
  }
