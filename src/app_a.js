/* ============================================================================
   RoboLab Web — application (UI + visualisation). All mathematics is in RL (core).
   ========================================================================== */
(function () {
  'use strict';
  const $ = (s, r) => (r || document).querySelector(s);
  const $$ = (s, r) => Array.from((r || document).querySelectorAll(s));
  const esc = (s) => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const D2R = RL.D2R, R2D = RL.R2D, V = RL.V, Mx = RL.Mx;
  const fx = (x, p) => { p = p === undefined ? 4 : p; if (!Number.isFinite(x)) return '—'; if (Math.abs(x) < 5 * Math.pow(10, -p - 1)) x = 0; return x.toFixed(p); };
  const toDisp = (j, v) => v * RL.jointDisplayScale(j), fromDisp = (j, v) => v / RL.jointDisplayScale(j);
  const uPos = (j) => j.type === 'R' ? '°' : ' m', uVel = (j) => j.type === 'R' ? '°/s' : ' m/s', uAcc = (j) => j.type === 'R' ? '°/s²' : ' m/s²', uTau = (j) => j.type === 'R' ? 'N·m' : 'N';
  const MODES = [['builder', 'Robot Builder'], ['dh', 'DH Parameters'], ['fk', 'Forward Kinematics'], ['ik', 'Inverse Kinematics'], ['jac', 'Jacobian'], ['traj', 'Trajectory Planning'],
    ['dyn', 'Dynamics'], ['ws', 'Workspace Analysis'], ['res', 'Results & Export'], ['tut', 'Tutorials']];
  const STORE_KEY = 'robolab.web.v1', LIB_KEY = 'robolab.web.library.v1';

  /* ------------------------------- state --------------------------------- */
  const S = {
    robot: null, q: [], mode: 'builder', sel: 0, theme: 'dark',
    target: { pos: [1.2, 0.8, 0], rpy: [0, 0, 0], useOri: false },
    ikOpt: { lambda: 0.05, tol: 1e-4, maxIter: 300, starts: 12 },
    ik: { result: null, sols: [], analytic: null, message: '' },
    trajP: { type: 'quintic', space: 'joint', T: 3, dt: 0.02, q0: [], qf: [], v0: [], vf: [], a0: [], af: [], pEnd: [0.5, 0.5, 0] },
    traj: null, trajWarn: [], trajFail: [], trajNote: '',
    dynP: { qd: [], qdd: [], tau: [], simT: 2, simDt: 0.005 }, sim: null,
    ws: { pts: null, count: 0, failed: 0, min: null, max: null, running: false, progress: 0, mode: 'height' },
    wsP: { n: 20000, size: 0.02, opacity: 0.85, seed: 7, bbox: true },
    plot: { kind: 'q', hidden: {}, view: null, logy: false },
    play: { data: null, label: '', t: 0, playing: false, speed: 1, loop: false },
    view: { ortho: false, wire: false, grid: true, frames: true, axes: true, labels: false, jointAxes: true, trail: true, light: 1 },
    jacVel: [], jacTwist: [0, 0, 0, 0, 0, 0],
  };
  const dirty = { pose: true, live: true, render: true, plot: true, save: 0 };
  const G = { ok: false };
  const gravityOf = () => S.robot.gravity;

  function toast(msg, bad) {
    const t = $('#toast'); t.textContent = msg; t.className = 'show' + (bad ? ' bad' : '');
    clearTimeout(toast.h); toast.h = setTimeout(() => { t.className = ''; }, bad ? 5200 : 2600);
  }
  function download(name, text, mime) {
    const url = URL.createObjectURL(new Blob([text], { type: mime || 'text/plain' })), a = document.createElement('a');
    a.href = url; a.download = name; document.body.appendChild(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000);
  }
  const csvCell = (v) => { const s = typeof v === 'number' ? (Number.isFinite(v) ? String(v) : '') : String(v === undefined || v === null ? '' : v); return /[",\n]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s; };
  const toCSV = (rows) => rows.map(r => r.map(csvCell).join(',')).join('\n') + '\n';

  /* ----------------------- robot / state management ---------------------- */
  function resizeStateArrays() {
    const n = S.robot.joints.length, z = () => new Array(n).fill(0);
    S.q = S.q.slice(0, n); while (S.q.length < n) S.q.push(S.robot.home[S.q.length] || 0);
    S.sel = Math.min(S.sel, n - 1);
    ['q0', 'qf', 'v0', 'vf', 'a0', 'af'].forEach(k => { S.trajP[k] = (S.trajP[k] || []).slice(0, n); while (S.trajP[k].length < n) S.trajP[k].push(0); });
    ['qd', 'qdd', 'tau'].forEach(k => { S.dynP[k] = (S.dynP[k] || []).slice(0, n); while (S.dynP[k].length < n) S.dynP[k].push(0); });
    S.jacVel = (S.jacVel || []).slice(0, n); while (S.jacVel.length < n) S.jacVel.push(0);
  }
  function invalidateResults() {
    S.ik = { result: null, sols: [], analytic: null, message: '' }; S.traj = null; S.trajWarn = []; S.trajFail = []; S.sim = null;
    S.ws.pts = null; S.ws.count = 0; S.ws.failed = 0; S.ws.min = S.ws.max = null;
    S.play.data = null; S.play.playing = false; S.play.t = 0;
    if (G.ok) { clearWorkspaceGL(); updatePathLine(); clearTrail(); }
  }
  function loadRobot(robot, q, opts) {
    S.robot = robot; S.q = q ? q.slice() : robot.home.slice(); S.sel = 0; resizeStateArrays();
    if (!opts || !opts.keepResults) invalidateResults();
    S.trajP.q0 = S.q.slice(); S.trajP.qf = S.q.map((v, i) => RL.clampQ(robot, S.q.map((x, k) => k === i ? x + (robot.joints[i].type === 'R' ? 0.8 : 0.05) : x))[i]);
    const p = RL.Pof(RL.fk(robot, S.q).T); S.target.pos = [p[0], p[1], p[2]]; S.trajP.pEnd = [p[0], p[1], p[2]];
    structureChanged(); updateRobotPose(); dirty.pose = false; dirty.live = dirty.plot = true; scheduleSave();
  }
  function loadPreset(id) { loadRobot(RL.buildPreset(id)); toast('Loaded: ' + S.robot.name); if (G.ok) fitView(); }
  /** Called after any change in the number/type of joints or their geometry. */
  function structureChanged() {
    if (G.ok) buildRobotScene();
    renderLeft(); renderPanel(); updateStatus(); refreshPlotKinds(); dirty.pose = dirty.live = dirty.render = dirty.plot = true;
  }
  /** Called after a numeric parameter changed (no structural change). */
  function parametersChanged(keepPanel) {
    S.ik.result = null; S.ik.sols = []; S.traj = null; S.trajWarn = []; S.sim = null;
    dirty.pose = dirty.live = dirty.plot = true; if (G.ok) updatePathLine();
    renderLeftValues(); updateStatus(); scheduleSave();
    if (!keepPanel) renderPanel();
  }
  function setQ(q, source) {
    S.q = q; dirty.pose = dirty.live = true; if (source !== 'playback') scheduleSave();
  }
  function setMode(m, silent) {
    S.mode = m; $$('#nav button').forEach(b => b.setAttribute('aria-selected', b.dataset.mode === m ? 'true' : 'false'));
    renderPanel(); updateStatus(); if (G.ok) { updateTargetVisibility(); } dirty.render = true; scheduleSave();
    if (!silent && window.innerWidth <= 900) $('#right').scrollIntoView({ behavior: 'smooth', block: 'start' });
  }

  /* ------------------------------ persistence ---------------------------- */
  function projectObject() {
    return { format: 'robolab-web-project', version: 1, app: 'RoboLab Web', savedAt: new Date().toISOString(), robot: RL.cloneRobot(S.robot), q: S.q.slice(),
      settings: { mode: S.mode, theme: S.theme, view: Object.assign({}, S.view), target: { pos: S.target.pos.slice(), rpy: S.target.rpy.slice(), useOri: S.target.useOri } } };
  }
  function applySettings(st) {
    if (!st || typeof st !== 'object') return;
    if (st.theme === 'light' || st.theme === 'dark') S.theme = st.theme;
    if (st.view && typeof st.view === 'object') for (const k of Object.keys(S.view)) if (k in st.view && typeof st.view[k] === typeof S.view[k] && (typeof S.view[k] !== 'number' || Number.isFinite(st.view[k]))) S.view[k] = st.view[k];
    S.view.light = RL.clamp(S.view.light, 0.2, 2);
    if (st.target && Array.isArray(st.target.pos) && st.target.pos.length === 3 && st.target.pos.every(RL.isNum)) { S.target.pos = st.target.pos.slice(); }
    if (st.target && Array.isArray(st.target.rpy) && st.target.rpy.length === 3 && st.target.rpy.every(RL.isNum)) S.target.rpy = st.target.rpy.slice();
    if (st.target && typeof st.target.useOri === 'boolean') S.target.useOri = st.target.useOri;
    if (typeof st.mode === 'string' && MODES.some(m => m[0] === st.mode)) S.mode = st.mode;
  }
  function scheduleSave() { clearTimeout(dirty.save); dirty.save = setTimeout(() => { try { localStorage.setItem(STORE_KEY, JSON.stringify(projectObject())); } catch (e) { /* storage unavailable: ignore */ } }, 600); }
  function loadSaved() {
    try { const raw = localStorage.getItem(STORE_KEY); if (!raw) return null; const p = JSON.parse(raw), v = RL.validateProject(p); return v.ok ? { v, settings: p.settings } : null; } catch (e) { return null; }
  }
  function importProjectText(text) {
    let obj; try { obj = JSON.parse(text); } catch (e) { toast('File is not valid JSON: ' + e.message, true); return false; }
    const v = RL.validateProject(obj);
    if (!v.ok) { toast('Project rejected: ' + v.errors.slice(0, 3).join(' '), true); return false; }
    applySettings(obj.settings); applyTheme(); loadRobot(v.robot, v.q); syncViewButtons(); applyLight(); toast('Project loaded: ' + v.robot.name); return true;
  }
  function getLibrary() { try { const l = JSON.parse(localStorage.getItem(LIB_KEY) || '[]'); return Array.isArray(l) ? l.filter(x => RL.validateRobot(x).ok).slice(0, 20) : []; } catch (e) { return []; } }
  function setLibrary(l) { try { localStorage.setItem(LIB_KEY, JSON.stringify(l)); } catch (e) { toast('Could not write to browser storage.', true); } }

  /* ============================== 3D scene =============================== */
  const O = { target: null, r: 3, az: 0.75, el: 0.5 };
  function showGLError(msg) { const e = $('#gl-error'); e.style.display = 'flex'; e.textContent = msg; $('#vp-help').style.display = 'none'; }

  function initGL() {
    if (typeof THREE === 'undefined') { showGLError('The Three.js library could not be loaded (it is fetched from the jsDelivr CDN). The calculation panels and graphs still work; connect to the internet and reload to see the 3D view.'); return false; }
    try { G.renderer = new THREE.WebGLRenderer({ canvas: $('#gl'), antialias: true }); } catch (e) { showGLError('WebGL is not available in this browser, so the 3D view is disabled. The calculation panels and graphs still work.'); return false; }
    THREE.Object3D.DefaultUp.set(0, 0, 1);
    G.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    G.scene = new THREE.Scene();
    G.persp = new THREE.PerspectiveCamera(40, 1, 0.01, 300); G.ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -300, 300);
    G.persp.up.set(0, 0, 1); G.ortho.up.set(0, 0, 1); G.cam = G.persp;
    G.ambient = new THREE.AmbientLight(0xffffff, 0.55); G.key = new THREE.DirectionalLight(0xffffff, 0.85); G.key.position.set(3, -4, 6);
    G.fill = new THREE.DirectionalLight(0x9fc4ff, 0.35); G.fill.position.set(-4, 3, 2);
    G.scene.add(G.ambient, G.key, G.fill);
    G.worldAxes = new THREE.AxesHelper(1); G.scene.add(G.worldAxes);
    G.robotRoot = new THREE.Group(); G.scene.add(G.robotRoot);
    G.targetGroup = new THREE.Group(); G.targetGroup.matrixAutoUpdate = false; G.scene.add(G.targetGroup);
    O.target = new THREE.Vector3(0, 0, 0.3);
    initTrail(); initOrbit();
    new ResizeObserver(resizeGL).observe($('#viewport'));
    resizeGL(); applyLight(); G.ok = true; return true;
  }
  function disposeObject(o) {
    o.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) (Array.isArray(c.material) ? c.material : [c.material]).forEach(m => m.dispose()); });
  }
  function setM4(obj, T) {
    obj.matrix.set(T[0][0], T[0][1], T[0][2], T[0][3], T[1][0], T[1][1], T[1][2], T[1][3], T[2][0], T[2][1], T[2][2], T[2][3], 0, 0, 0, 1); obj.matrixWorldNeedsUpdate = true;
  }
  const niceStep = (x) => { const e = Math.pow(10, Math.floor(Math.log10(x))), m = x / e; return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * e; };

  function buildGrid(reach) {
    if (G.grid) { G.scene.remove(G.grid); disposeObject(G.grid); }
    const step = niceStep(reach / 5), div = 20, size = step * div;
    G.grid = new THREE.GridHelper(size, div, 0x4a5d70, 0x2b3947); G.grid.rotation.x = Math.PI / 2; G.grid.visible = S.view.grid; G.scene.add(G.grid);
    G.worldAxes.scale.setScalar(Math.max(0.2, reach * 0.35)); G.worldAxes.visible = S.view.axes;
  }
  function buildRobotScene() {
    if (!G.ok) return;
    while (G.robotRoot.children.length) { const c = G.robotRoot.children[0]; G.robotRoot.remove(c); disposeObject(c); }
    const robot = S.robot, n = robot.joints.length, reach = Math.max(0.3, RL.reachEstimate(robot)), r = RL.clamp(reach * 0.018, 0.008, 0.05);
    G.scale = r; G.reach = reach; G.mats = []; G.fg = []; G.dSeg = []; G.aSeg = []; G.jointMesh = []; G.jointMat = []; G.linkMat = []; G.axesHelpers = [];
    const mk = (color, extra) => { const m = new THREE.MeshStandardMaterial(Object.assign({ color, metalness: 0.25, roughness: 0.5 }, extra || {})); m.wireframe = S.view.wire; G.mats.push(m); return m; };
    const baseMat = mk(0x3d4a5a);
    const zSeg = new THREE.CylinderGeometry(1, 1, 1, 14); zSeg.rotateX(Math.PI / 2); zSeg.translate(0, 0, 0.5);
    const xSeg = new THREE.CylinderGeometry(1, 1, 1, 14); xSeg.rotateY(Math.PI / 2); xSeg.translate(-0.5, 0, 0);
    const revG = new THREE.CylinderGeometry(1.5 * r, 1.5 * r, 3.2 * r, 20); revG.rotateX(Math.PI / 2);
    const prisG = new THREE.BoxGeometry(2.6 * r, 2.6 * r, 4 * r);
    for (let i = 0; i <= n; i++) {
      const g = new THREE.Group(); g.matrixAutoUpdate = false; (i === 0 ? G.robotRoot : G.fg[i - 1]).add(g); G.fg.push(g);
      const ax = new THREE.AxesHelper(i === n ? reach * 0.09 : reach * 0.055); ax.visible = i === n || S.view.frames; g.add(ax); G.axesHelpers.push(ax);
    }
    const baseG = new THREE.CylinderGeometry(3 * r, 3.4 * r, 0.8 * r, 28); baseG.rotateX(Math.PI / 2); const base = new THREE.Mesh(baseG, baseMat); base.position.z = -0.4 * r; G.fg[0].add(base);
    for (let i = 0; i < n; i++) {
      const j = robot.joints[i], pris = j.type === 'P';
      const jm = mk(pris ? 0xd9a047 : 0x3fa5be), lm = mk(0x9fb0c2);
      G.jointMat.push(jm); G.linkMat.push(lm);
      const jmesh = new THREE.Mesh(pris ? prisG : revG, jm); G.fg[i].add(jmesh); G.jointMesh.push(jmesh);
      const lg = new THREE.LineBasicMaterial({ color: pris ? 0xd9a047 : 0x3fa5be }); G.mats.push(lg);
      const lgeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, -3.5 * r), new THREE.Vector3(0, 0, 3.5 * r)]);
      const axisLine = new THREE.Line(lgeo, lg); axisLine.userData.jointAxis = true; axisLine.visible = S.view.jointAxes; G.fg[i].add(axisLine);
      const dS = new THREE.Mesh(zSeg, lm); G.fg[i].add(dS); G.dSeg.push(dS);
      const aS = new THREE.Mesh(xSeg, lm); G.fg[i + 1].add(aS); G.aSeg.push(aS);
      const knuckle = new THREE.Mesh(new THREE.SphereGeometry(r * 1.05, 14, 10), lm); G.fg[i + 1].add(knuckle);
    }
    const tip = new THREE.Mesh(new THREE.ConeGeometry(1.3 * r, 3 * r, 16), mk(0xe7edf4)); tip.geometry.rotateX(Math.PI / 2); tip.geometry.translate(0, 0, 1.5 * r); G.fg[n].add(tip);
    G.labelEls = []; const lab = $('#labels'); lab.innerHTML = ''; for (let i = 0; i < n; i++) { const s = document.createElement('span'); lab.appendChild(s); G.labelEls.push(s); }
    buildGrid(reach); buildTarget(); highlightSelected(); setWire(S.view.wire);
    G.trailLen = 0; if (G.trailLine) G.trailLine.geometry.setDrawRange(0, 0);
    dirty.pose = dirty.render = true;
  }
  function buildTarget() {
    while (G.targetGroup.children.length) { const c = G.targetGroup.children[0]; G.targetGroup.remove(c); disposeObject(c); }
    const len = (G.reach || 1) * 0.12, ax = new THREE.AxesHelper(len), m = new THREE.Mesh(new THREE.OctahedronGeometry(len * 0.12), new THREE.MeshBasicMaterial({ color: 0xf2807b, wireframe: true }));
    G.targetGroup.add(ax, m); updateTargetPose();
  }
  function updateTargetPose() {
    if (!G.ok) return; const R = RL.rpyToR(S.target.rpy[0] * D2R, S.target.rpy[1] * D2R, S.target.rpy[2] * D2R), p = S.target.pos;
    setM4(G.targetGroup, [[R[0][0], R[0][1], R[0][2], p[0]], [R[1][0], R[1][1], R[1][2], p[1]], [R[2][0], R[2][1], R[2][2], p[2]], [0, 0, 0, 1]]); dirty.render = true;
  }
  function updateTargetVisibility() { if (G.ok) { G.targetGroup.visible = (S.mode === 'ik' || S.mode === 'traj'); dirty.render = true; } }
  function highlightSelected() {
    if (!G.ok || !G.jointMat) return;
    G.jointMat.forEach((m, i) => { m.emissive.setHex(i === S.sel ? 0x2d8fa5 : 0x000000); G.linkMat[i].emissive.setHex(i === S.sel ? 0x25343f : 0x000000); });
    dirty.render = true;
  }
  function setWire(on) { S.view.wire = on; if (G.mats) G.mats.forEach(m => { if (m.wireframe !== undefined) m.wireframe = on; }); dirty.render = true; }
  function applyLight() { if (!G.ok) return; G.ambient.intensity = 0.55 * S.view.light; G.key.intensity = 0.85 * S.view.light; G.fill.intensity = 0.35 * S.view.light; dirty.render = true; }

  /* trail + preview path */
  const TRAIL_MAX = 6000;
  function initTrail() {
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3)); geo.setDrawRange(0, 0);
    G.trailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffd479 })); G.trailLine.frustumCulled = false; G.scene.add(G.trailLine); G.trailLen = 0;
    G.pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x77d3a0, transparent: true, opacity: 0.8 })); G.pathLine.frustumCulled = false; G.pathLine.visible = false; G.scene.add(G.pathLine);
  }
  function addTrail(p) {
    const a = G.trailLine.geometry.attributes.position.array;
    if (G.trailLen > 0) { const k = (G.trailLen - 1) * 3; if (Math.hypot(a[k] - p[0], a[k + 1] - p[1], a[k + 2] - p[2]) < 1e-4) return; }
    if (G.trailLen >= TRAIL_MAX) { a.copyWithin(0, 3); G.trailLen = TRAIL_MAX - 1; }
    const k = G.trailLen * 3; a[k] = p[0]; a[k + 1] = p[1]; a[k + 2] = p[2]; G.trailLen++;
    G.trailLine.geometry.setDrawRange(0, G.trailLen); G.trailLine.geometry.attributes.position.needsUpdate = true;
  }
  function clearTrail() { if (!G.ok) return; G.trailLen = 0; G.trailLine.geometry.setDrawRange(0, 0); dirty.render = true; }
  function updatePathLine() {
    if (!G.ok) return; const d = S.traj && S.traj.ee;
    if (!d || d.length < 2) { G.pathLine.visible = false; dirty.render = true; return; }
    const a = new Float32Array(d.length * 3); d.forEach((p, i) => { a[3 * i] = p[0]; a[3 * i + 1] = p[1]; a[3 * i + 2] = p[2]; });
    G.pathLine.geometry.dispose(); G.pathLine.geometry = new THREE.BufferGeometry(); G.pathLine.geometry.setAttribute('position', new THREE.BufferAttribute(a, 3)); G.pathLine.visible = true; dirty.render = true;
  }

  /* pose update — called from the frame loop when q changed; no objects are re-created */
  function updateRobotPose() {
    const robot = S.robot, n = robot.joints.length, F = RL.fk(robot, S.q);
    S.fkCache = F;
    if (!G.ok) return F;
    for (let i = 1; i <= n; i++) setM4(G.fg[i], F.links[i - 1]);
    for (let i = 0; i < n; i++) {
      const j = robot.joints[i], d = j.d + (j.type === 'P' ? S.q[i] : 0), a = j.a, rr = G.scale * 0.55;
      const dv = Math.abs(d) > 1e-6, av = Math.abs(a) > 1e-6;
      G.dSeg[i].visible = dv; if (dv) G.dSeg[i].scale.set(rr, rr, d);
      G.aSeg[i].visible = av; if (av) G.aSeg[i].scale.set(a, rr, rr);
    }
    G.robotRoot.updateMatrixWorld(true);
    if (S.view.trail && S.play.playing) addTrail(RL.Pof(F.T));
    dirty.render = true; return F;
  }
  function updateLabels() {
    const lab = $('#labels'); lab.style.display = S.view.labels ? 'block' : 'none'; if (!S.view.labels || !S.fkCache) return;
    const w = $('#viewport').clientWidth, h = $('#viewport').clientHeight, v = new THREE.Vector3();
    S.robot.joints.forEach((j, i) => { const p = RL.Pof(S.fkCache.frames[i]); v.set(p[0], p[1], p[2]).project(G.cam);
      const el = G.labelEls[i]; if (!el) return; const vis = v.z < 1 && Math.abs(v.x) < 1.2 && Math.abs(v.y) < 1.2; el.style.display = vis ? 'block' : 'none';
      el.style.left = ((v.x * 0.5 + 0.5) * w) + 'px'; el.style.top = ((-v.y * 0.5 + 0.5) * h) + 'px'; el.textContent = 'q' + (i + 1) + ' = ' + fx(toDisp(j, S.q[i]), 1) + uPos(j).trim(); });
  }

  /* camera / orbit */
  function applyCam() {
    O.el = RL.clamp(O.el, -Math.PI / 2 + 0.02, Math.PI / 2 - 0.02); O.r = RL.clamp(O.r, 0.05, 100);
    const ce = Math.cos(O.el); G.cam.position.set(O.target.x + O.r * ce * Math.cos(O.az), O.target.y + O.r * ce * Math.sin(O.az), O.target.z + O.r * Math.sin(O.el));
    G.cam.up.set(0, 0, 1); G.cam.lookAt(O.target);
    if (G.cam === G.ortho) { const hh = O.r * Math.tan(20 * D2R), w = Math.max(1, $('#viewport').clientWidth), h = Math.max(1, $('#viewport').clientHeight); G.ortho.left = -hh * w / h; G.ortho.right = hh * w / h; G.ortho.top = hh; G.ortho.bottom = -hh; G.ortho.updateProjectionMatrix(); }
    dirty.render = true;
  }
  function resizeGL() {
    const el = $('#viewport'), w = Math.max(10, el.clientWidth), h = Math.max(10, el.clientHeight);
    G.renderer.setSize(w, h, false); G.persp.aspect = w / h; G.persp.updateProjectionMatrix(); applyCam();
  }
  function setProjection(ortho) { S.view.ortho = ortho; G.cam = ortho ? G.ortho : G.persp; applyCam(); syncViewButtons(); }
  function fitView() {
    if (!G.ok || !S.fkCache) return; const pts = S.fkCache.frames.map(RL.Pof); if (S.ws.min) pts.push(S.ws.min, S.ws.max);
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]; pts.forEach(p => { for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[k]); mx[k] = Math.max(mx[k], p[k]); } });
    const reach = Math.max(0.3, G.reach || 1); for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], k === 2 ? 0 : -reach * 0.2); mx[k] = Math.max(mx[k], reach * 0.2); }
    const c = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2], R = Math.max(0.2, 0.5 * Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]));
    O.target.set(c[0], c[1], c[2]); O.r = R * 3.0; applyCam();
  }
  function initOrbit() {
    const cv = $('#gl'), ptrs = new Map(); let moved = 0, down = null;
    cv.addEventListener('contextmenu', e => e.preventDefault());
    cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (ptrs.size === 1) { moved = 0; down = { x: e.clientX, y: e.clientY, shift: e.shiftKey, button: e.button }; } cv.focus(); });
    cv.addEventListener('pointermove', e => {
      const p = ptrs.get(e.pointerId); if (!p) return;
      if (ptrs.size === 1) {
        const dx = e.clientX - p.x, dy = e.clientY - p.y; moved += Math.abs(dx) + Math.abs(dy);
        if (e.buttons & 2 || e.buttons & 4 || e.shiftKey) pan(dx, dy); else { O.az -= dx * 0.008; O.el += dy * 0.008; applyCam(); }
      } else if (ptrs.size === 2) {
        const [a, b] = Array.from(ptrs.values()), d0 = Math.hypot(a.x - b.x, a.y - b.y), c0x = (a.x + b.x) / 2, c0y = (a.y + b.y) / 2;
        p.x = e.clientX; p.y = e.clientY;
        const d1 = Math.hypot(a.x - b.x, a.y - b.y), c1x = (a.x + b.x) / 2, c1y = (a.y + b.y) / 2;
        if (d1 > 0) O.r *= d0 / d1; pan(c1x - c0x, c1y - c0y); moved += 10; return;
      }
      p.x = e.clientX; p.y = e.clientY;
    });
    const up = (e) => {
      if (!ptrs.has(e.pointerId)) return; ptrs.delete(e.pointerId);
      if (ptrs.size === 0 && down && moved < 4 && down.button === 0 && down.shift) pickTarget(e);
      if (ptrs.size === 0) down = null;
    };
    cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
    cv.addEventListener('wheel', e => { e.preventDefault(); O.r *= Math.exp(RL.clamp(e.deltaY, -200, 200) * 0.0012); applyCam(); }, { passive: false });
    function pan(dx, dy) {
      G.cam.updateMatrixWorld(); const m = G.cam.matrixWorld.elements, s = (2 * O.r * Math.tan(20 * D2R)) / Math.max(1, cv.clientHeight);
      O.target.x += (-m[0] * dx + m[4] * dy) * s; O.target.y += (-m[1] * dx + m[5] * dy) * s; O.target.z += (-m[2] * dx + m[6] * dy) * s; applyCam();
    }
  }
  function pickTarget(e) {
    const cv = $('#gl'), rc = cv.getBoundingClientRect(), nd = new THREE.Vector2(((e.clientX - rc.left) / rc.width) * 2 - 1, -((e.clientY - rc.top) / rc.height) * 2 + 1);
    const ray = new THREE.Raycaster(); ray.setFromCamera(nd, G.cam); const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -S.target.pos[2]), hit = new THREE.Vector3();
    if (!ray.ray.intersectPlane(plane, hit)) { toast('Click was parallel to the target plane — orbit the view and try again.', true); return; }
    S.target.pos = [hit.x, hit.y, S.target.pos[2]]; updateTargetPose(); if (S.mode !== 'ik') setMode('ik'); else fillTargetInputs();
    toast('IK target set to (' + fx(hit.x, 3) + ', ' + fx(hit.y, 3) + ', ' + fx(S.target.pos[2], 3) + ') m'); scheduleSave();
  }

  /* workspace point cloud */
  function clearWorkspaceGL() {
    if (!G.ok) return;
    if (G.wsPoints) { G.scene.remove(G.wsPoints); disposeObject(G.wsPoints); G.wsPoints = null; }
    if (G.wsBox) { G.scene.remove(G.wsBox); disposeObject(G.wsBox); G.wsBox = null; }
    dirty.render = true;
  }
  function buildWorkspaceGL() {
    if (!G.ok || !S.ws.pts) return; clearWorkspaceGL();
    const n = S.ws.count, pos = S.ws.pts, col = new Float32Array(n * 3), mn = S.ws.min, mx = S.ws.max, c = new THREE.Color();
    for (let i = 0; i < n; i++) {
      let t; if (S.ws.mode === 'radius') t = Math.hypot(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]) / Math.max(1e-9, Math.hypot(Math.max(Math.abs(mn[0]), Math.abs(mx[0])), Math.max(Math.abs(mn[1]), Math.abs(mx[1])), Math.max(Math.abs(mn[2]), Math.abs(mx[2]))));
      else t = (pos[3 * i + 2] - mn[2]) / Math.max(1e-9, mx[2] - mn[2]);
      c.setHSL(0.58 - 0.58 * RL.clamp(t, 0, 1), 0.75, 0.58); col[3 * i] = c.r; col[3 * i + 1] = c.g; col[3 * i + 2] = c.b;
    }
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(pos.subarray(0, n * 3), 3)); geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    G.wsPoints = new THREE.Points(geo, new THREE.PointsMaterial({ size: S.wsP.size, vertexColors: true, transparent: true, opacity: S.wsP.opacity, sizeAttenuation: true, depthWrite: false })); G.wsPoints.frustumCulled = false; G.scene.add(G.wsPoints);
    const box = new THREE.Box3(new THREE.Vector3(mn[0], mn[1], mn[2]), new THREE.Vector3(mx[0], mx[1], mx[2])); G.wsBox = new THREE.Box3Helper(box, 0xeeb25a); G.wsBox.visible = S.wsP.bbox; G.scene.add(G.wsBox); dirty.render = true;
  }

  /* render loop */
  function renderGL() {
    applyCamIfNeeded(); G.renderer.render(G.scene, G.cam); updateLabels();
  }
  function applyCamIfNeeded() { /* camera is applied on interaction */ }
  function syncViewButtons() {
    $$('#vp-tools [data-view]').forEach(b => { const k = b.dataset.view; b.setAttribute('aria-pressed', String(!!S.view[k])); });
    const lc = $('#light-range'); if (lc) lc.value = S.view.light;
  }
  function buildViewTools() {
    const defs = [['fit', 'Fit', 'Fit robot to view (F)'], ['ortho', 'Ortho', 'Perspective / orthographic camera (C)'], ['wire', 'Wire', 'Wireframe / solid (W)'], ['grid', 'Grid', 'Show grid (G)'], ['axes', 'World axes', 'World XYZ axes'],
      ['frames', 'Frames', 'Show link coordinate frames (A)'], ['jointAxes', 'Joint axes', 'Show joint axes'], ['labels', 'Labels', 'Show joint values in the viewport'], ['trail', 'Trail', 'Record end-effector trail during playback'], ['clear', 'Clear trail', 'Clear trail (T)']];
    $('#vp-tools').innerHTML = defs.map(d => '<button class="btn small" data-view="' + d[0] + '" title="' + esc(d[2]) + '" aria-pressed="false">' + d[1] + '</button>').join('') +
      '<span class="light-ctl" title="Lighting intensity"><label for="light-range">Light</label><input id="light-range" type="range" min="0.2" max="2" step="0.05" value="1"></span>';
    $('#vp-tools').addEventListener('click', e => { const b = e.target.closest('[data-view]'); if (b) viewAction(b.dataset.view); });
    $('#light-range').addEventListener('input', e => { S.view.light = +e.target.value; applyLight(); scheduleSave(); });
  }
  function viewAction(k) {
    if (k === 'fit') { fitView(); return; } if (k === 'clear') { clearTrail(); return; }
    if (!G.ok) { toast('3D view is unavailable.', true); return; }
    if (k === 'ortho') { setProjection(!S.view.ortho); scheduleSave(); return; }
    S.view[k] = !S.view[k];
    if (k === 'wire') setWire(S.view.wire); if (k === 'grid') G.grid.visible = S.view.grid; if (k === 'axes') G.worldAxes.visible = S.view.axes;
    if (k === 'frames') G.axesHelpers.forEach((a, i) => { a.visible = i === G.axesHelpers.length - 1 || S.view.frames; });
    if (k === 'jointAxes') G.robotRoot.traverse(o => { if (o.userData.jointAxis) o.visible = S.view.jointAxes; });
    syncViewButtons(); dirty.render = true; scheduleSave();
  }
