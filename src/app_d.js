
  /* ============================== Tutorials ============================== */
  const planarRobot = (L1, L2, m1, m2) => { const r = RL.buildPreset('planar2'); r.joints[0].a = L1; r.joints[1].a = L2; r.name = '2-link planar arm (L₁=' + L1 + ', L₂=' + L2 + ')'; if (m1 !== undefined) { RL.autoInertial(r.joints[0], m1); RL.autoInertial(r.joints[1], m2); } else r.joints.forEach(j => RL.autoInertial(j)); return r; };
  const check = (label, expected, got, tol, unit) => '<tr><td>' + label + '</td><td>' + expected + '</td><td>' + fx(got, 5) + (unit || '') + '</td><td>' + (Math.abs(parseFloat(expected) - got) <= tol ? '<span class="ok">✔ matches</span>' : '<span class="bad">✘ differs</span>') + '</td></tr>';
  const chkTable = (rows) => '<div class="scroll-x"><table class="data"><tr><th>quantity</th><th>hand calculation</th><th>engine</th><th></th></tr>' + rows.join('') + '</table></div>';
  const EXAMPLES = [
    { title: 'Example 1 — Forward kinematics of a 2-link arm',
      problem: 'L₁ = L₂ = 1 m, q₁ = 30°, q₂ = 45°. Find the end-effector position.',
      steps: 'x = cos 30° + cos 75° = 0.866025 + 0.258819 = <b>1.12484</b> m<br>y = sin 30° + sin 75° = 0.500000 + 0.965926 = <b>1.46593</b> m',
      run() { const r = planarRobot(1, 1); loadRobot(r, [30 * D2R, 45 * D2R]); const p = RL.Pof(RL.fk(r, S.q).T); return chkTable([check('x', '1.12484', p[0], 1e-4, ' m'), check('y', '1.46593', p[1], 1e-4, ' m')]); } },
    { title: 'Example 2 — Inverse kinematics (two solutions)',
      problem: 'L₁ = L₂ = 1 m. Reach the point (1.5, 1.0).',
      steps: 'cos q₂ = (x² + y² − L₁² − L₂²)/(2L₁L₂) = (3.25 − 2)/2 = 0.625 ⇒ q₂ = ±51.318°<br>q₁ = atan2(y, x) − atan2(L₂ sin q₂, L₁ + L₂ cos q₂)<br>elbow-down (q₂ = +51.318°): q₁ = 33.690° − 25.659° = <b>8.031°</b><br>elbow-up (q₂ = −51.318°): q₁ = 33.690° + 25.659° = <b>59.349°</b>',
      run() { const r = planarRobot(1, 1); loadRobot(r, [0, 0]); S.target.pos = [1.5, 1, 0]; const a = RL.ik2R(r, 1.5, 1, 0); const d = a.solutions.find(s => s.q[1] > 0), u = a.solutions.find(s => s.q[1] < 0); 
        return chkTable([check('q₁ (elbow-down)', '8.031', d.q[0] * R2D, 1e-2, '°'), check('q₂ (elbow-down)', '51.318', d.q[1] * R2D, 1e-2, '°'), check('q₁ (elbow-up)', '59.349', u.q[0] * R2D, 1e-2, '°'), check('q₂ (elbow-up)', '-51.318', u.q[1] * R2D, 1e-2, '°')]) + '<div class="muted">Open “Analytic (2R)” to animate either solution.</div>'; } },
    { title: 'Example 3 — Jacobian and singularity',
      problem: 'L₁ = L₂ = 1 m at q = (0°, 90°). Find J, det J and the manipulability.',
      steps: 'J = [[−L₁ sin q₁ − L₂ sin(q₁+q₂), −L₂ sin(q₁+q₂)], [L₁ cos q₁ + L₂ cos(q₁+q₂), L₂ cos(q₁+q₂)]] = [[−1, −1], [1, 0]]<br>det J = L₁L₂ sin q₂ = <b>1</b>; the arm is singular when sin q₂ = 0 (fully stretched or folded).',
      run() { const r = planarRobot(1, 1); loadRobot(r, [0, 90 * D2R]); const J = RL.jacobian(r, S.q), a = RL.analyzeJacobian(J, 2); 
        return chkTable([check('J₁₁', '-1', J[0][0], 1e-9), check('J₁₂', '-1', J[0][1], 1e-9), check('J₂₁', '1', J[1][0], 1e-9), check('J₂₂', '0', J[1][1], 1e-9), check('manipulability', '1', a.manipulability, 1e-9)]); } },
    { title: 'Example 4 — Cubic trajectory',
      problem: 'One joint moves from 0° to 90° in T = 2 s with zero start/end velocity (cubic).',
      steps: 'q(t) = Δ(3u² − 2u³), u = t/T, Δ = π/2 rad<br>q(1 s) = Δ/2 = <b>45°</b><br>peak velocity at t = T/2: 1.5Δ/T = <b>1.1781 rad/s</b> (67.5°/s)<br>peak |acceleration| at the ends: 6Δ/T² = <b>2.3562 rad/s²</b>',
      run() { const r = planarRobot(1, 1); loadRobot(r, [0, 0]); const tr = RL.jointTrajectory({ type: 'cubic', q0: [0, 0], qf: [Math.PI / 2, 0], T: 2, dt: 0.01 }); S.trajP.type = 'cubic'; S.trajP.T = 2; S.trajP.dt = 0.01; S.trajP.q0 = [0, 0]; S.trajP.qf = [Math.PI / 2, 0]; 
        return chkTable([check('q(1 s)', '45', tr.q[100][0] * R2D, 1e-6, '°'), check('peak velocity', '1.1781', Math.max(...tr.qd.map(v => Math.abs(v[0]))), 1e-4, ' rad/s'), check('peak acceleration', '2.3562', Math.max(...tr.qdd.map(v => Math.abs(v[0]))), 1e-3, ' rad/s²')]) + '<div class="muted">Press “Generate & load trajectory” to animate it.</div>'; } },
    { title: 'Example 5 — Static gravity torque',
      problem: '2-link arm in the X–Y plane, gravity along −Y, L₁ = L₂ = 1 m, link masses 1 kg with the COM at mid-link, arm stretched along +X (q = 0).',
      steps: 'τ₁ = (m₁ L₁/2 + m₂ (L₁ + L₂/2)) g = (0.5 + 1.5)·9.81 = <b>19.62</b> N·m<br>τ₂ = m₂ (L₂/2) g = <b>4.905</b> N·m',
      run() { const r = planarRobot(1, 1, 1, 1); loadRobot(r, [0, 0]); const d = RL.inverseDynamics(r, [0, 0], [0, 0], [0, 0], r.gravity); 
        return chkTable([check('τ₁', '19.62', d.tau[0], 1e-6, ' N·m'), check('τ₂', '4.905', d.tau[1], 1e-6, ' N·m')]); } },
  ];
  const GLOSSARY = [['Degrees of freedom (DOF)', 'The number of independent joint variables; for a serial arm, the number of joints.'], ['Joint space / task space', 'Joint space is the set of joint values q; task space is the set of end-effector poses.'], ['Forward kinematics', 'Computing the end-effector pose from the joint values.'], ['Inverse kinematics', 'Finding joint values that give a desired end-effector pose; may have zero, one, several or infinitely many solutions.'], ['Denavit–Hartenberg (DH) parameters', 'Four numbers per link (a, α, d, θ) that define the transform between consecutive link frames.'], ['Homogeneous transform', 'A 4×4 matrix combining a 3×3 rotation and a translation.'], ['Rotation matrix', 'An orthonormal 3×3 matrix with determinant +1; its columns are the rotated axes.'], ['Jacobian', 'The matrix mapping joint velocities to end-effector linear and angular velocity.'], ['Singularity', 'A configuration where the Jacobian loses rank; the arm cannot move in some direction and IK becomes ill-conditioned.'], ['Manipulability', 'Yoshikawa’s measure √det(JJᵀ); zero at singularities.'], ['Damped least squares', 'A regularised pseudo-inverse that trades a small error for stability near singularities.'], ['Workspace', 'The set of end-effector positions the robot can reach within its joint limits.'], ['Trajectory', 'A time-parametrised path q(t) with velocity and acceleration.'], ['Inertia matrix M(q)', 'Configuration-dependent matrix relating joint accelerations to torques.'], ['Coriolis / centrifugal terms', 'Velocity-dependent torques arising from coupled link motion.'], ['Gravity vector g(q)', 'Joint torques needed to hold the arm static against gravity.']];
  const GUIDE = [['Load the 2-link planar arm and open Forward Kinematics.', 'planar2', 'fk'], ['Drag the joint sliders on the left. Compare x, y with L₁cos q₁ + L₂cos(q₁+q₂) using the reference box.', null, null], ['Open Inverse Kinematics, type a target (try 1.2, 0.8, 0) and press “Analytic (2R)”. Two solutions appear — animate each.', 'planar2', 'ik'], ['Open Jacobian and set q₂ = 0. The singular-value warning appears because the arm is fully stretched.', null, 'jac'], ['Open Trajectory Planning, press “Generate & load trajectory” and press play. Switch the graph to velocity and acceleration.', null, 'traj'], ['Load the 6-DOF arm, open Workspace Analysis and run the sampling to see the reachable point cloud.', 'ur5like', 'ws']];
  PANELS.tut = () => head('Tutorials', 'Learn the ideas, check worked examples against the engine, and practise.') +
    '<div class="sect tut"><details data-k="tg"' + (openKeys.tg === false ? '' : ' open') + '><summary><b>Guided tutorial</b></summary><ol class="steps">' + GUIDE.map((g, i) => '<li>' + g[0] + ' <button class="btn small" data-act="tut-go" data-i="' + i + '">Go</button></li>').join('') + '</ol></details></div>' +
    '<div class="sect tut"><details data-k="tw"' + dOpen('tw') + '><summary><b>Worked examples</b> (numbers you can verify independently)</summary>' + EXAMPLES.map((x, i) => '<div class="card"><h3>' + x.title + '</h3><p>' + x.problem + '</p><p>' + x.steps + '</p><button class="btn small primary" data-act="tut-ex" data-i="' + i + '">Load & verify with the engine</button><div id="ex-out-' + i + '"></div></div>').join('') + '</details></div>' +
    '<div class="sect tut"><details data-k="te"' + dOpen('te') + '><summary><b>Exercises</b></summary><div id="exercise"></div></details></div>' +
    '<div class="sect tut"><details data-k="tc"' + dOpen('tc') + '><summary><b>Concepts: frames, rotations, DH</b></summary>' +
    '<p>A <b>coordinate frame</b> is an origin plus three perpendicular axes. A rigid motion is a 4×4 matrix T = [[R, p],[0, 1]]: R rotates, p translates. Chaining frames means multiplying matrices in order, so the end-effector pose is the product of the link transforms.</p>' +
    '<p>Rotations about the axes: R<sub>z</sub>(θ) = [[c, −s, 0],[s, c, 0],[0, 0, 1]]. A valid rotation matrix satisfies RᵀR = I and det R = +1; the app checks this live in Forward Kinematics.</p>' +
    '<svg class="diagram" viewBox="0 0 440 230" role="img" aria-label="Classical DH parameters diagram"><line class="ax" x1="60" y1="200" x2="60" y2="30"/><text x="66" y="38">z<tspan dy="4" font-size="9">i−1</tspan></text><line class="ax" x1="300" y1="170" x2="360" y2="30"/><text x="362" y="38">z<tspan dy="4" font-size="9">i</tspan></text>' +
    '<line class="ln" x1="60" y1="150" x2="320" y2="150" stroke-dasharray="5 4"/><text x="170" y="144">a<tspan dy="4" font-size="9">i</tspan> (common normal)</text><line class="am" x1="40" y1="200" x2="40" y2="150"/><text x="8" y="180">d<tspan dy="4" font-size="9">i</tspan></text>' +
    '<path class="ln" d="M60 120 A24 24 0 0 1 84 144"/><text x="90" y="130">θ<tspan dy="4" font-size="9">i</tspan> (about z<tspan dy="4" font-size="9">i−1</tspan>)</text><path class="ln" d="M322 125 A30 30 0 0 1 340 140"/><text x="346" y="124">α<tspan dy="4" font-size="9">i</tspan></text></svg>' +
    '<p><b>Classical DH:</b> θ<sub>i</sub> = rotation about z<sub>i−1</sub> from x<sub>i−1</sub> to x<sub>i</sub>; d<sub>i</sub> = distance along z<sub>i−1</sub>; a<sub>i</sub> = distance along x<sub>i</sub>; α<sub>i</sub> = rotation about x<sub>i</sub> from z<sub>i−1</sub> to z<sub>i</sub>. Revolute joints vary θ; prismatic joints vary d. The sketch is schematic.</p></details></div>' +
    '<div class="sect tut"><details data-k="tgl"' + dOpen('tgl') + '><summary><b>Glossary</b></summary><dl style="margin:6px 0">' + GLOSSARY.map(g => '<dt><b>' + g[0] + '</b></dt><dd style="margin:0 0 6px 12px" class="muted">' + g[1] + '</dd>').join('') + '</dl></details></div>';
  POST.tut = () => renderExercise();
  ACT['tut-go'] = (b) => { const g = GUIDE[+b.dataset.i]; if (g[1]) loadPreset(g[1]); if (g[2]) setMode(g[2], true); };
  ACT['tut-ex'] = (b) => { const i = +b.dataset.i; const html = EXAMPLES[i].run(); S.play.playing = false; setTimeout(() => { const o = $('#ex-out-' + i); if (o) o.innerHTML = html; }, 0); };
  /* exercises */
  const EX = { kind: 'fk', data: null, shown: false, msg: '' };
  const rnd = (a) => a[Math.floor(Math.random() * a.length)];
  function exNew(kind) {
    EX.kind = kind || EX.kind; EX.shown = false; EX.msg = ''; const L1 = rnd([1, 1.2, 0.8]), L2 = rnd([1, 0.6, 0.9]), q1 = rnd([-45, -30, 15, 30, 45, 60]), q2 = rnd([-60, -30, 30, 45, 60, 90]);
    const x = L1 * Math.cos(q1 * D2R) + L2 * Math.cos((q1 + q2) * D2R), y = L1 * Math.sin(q1 * D2R) + L2 * Math.sin((q1 + q2) * D2R); EX.data = { L1, L2, q1, q2, x, y };
  }
  function renderExercise() {
    const el = $('#exercise'); if (!el) return; if (!EX.data) exNew('fk'); const d = EX.data;
    const tabs = '<div class="row"><button class="btn small' + (EX.kind === 'fk' ? ' primary' : '') + '" data-act="ex-kind" data-k="fk">Forward kinematics</button><button class="btn small' + (EX.kind === 'ik' ? ' primary' : '') + '" data-act="ex-kind" data-k="ik">Inverse kinematics</button><button class="btn small" data-act="ex-new">New exercise (reset)</button></div>';
    let body;
    if (EX.kind === 'fk') body = '<p>A 2-link planar arm has L₁ = ' + d.L1 + ' m, L₂ = ' + d.L2 + ' m, q₁ = ' + d.q1 + '°, q₂ = ' + d.q2 + '°. Find the end-effector position (give 3 decimals).</p><div class="grid2">' + fld('ex-a', 'x [m]', '') + fld('ex-b', 'y [m]', '') + '</div>';
    else body = '<p>A 2-link planar arm has L₁ = ' + d.L1 + ' m, L₂ = ' + d.L2 + ' m. Find joint angles (either elbow) that reach (' + fx(d.x, 3) + ', ' + fx(d.y, 3) + ') m.</p><div class="grid2">' + fld('ex-a', 'q₁ [°]', '') + fld('ex-b', 'q₂ [°]', '') + '</div>';
    el.innerHTML = tabs + body + '<div class="row"><button class="btn small primary" data-act="ex-check">Check answer</button><button class="btn small" data-act="ex-show">Show solution</button><button class="btn small" data-act="ex-load">Load robot in viewer</button></div><div id="ex-msg">' + EX.msg + '</div>';
  }
  ACT['ex-kind'] = (b) => { exNew(b.dataset.k); renderExercise(); }; ACT['ex-new'] = () => { exNew(); renderExercise(); };
  ACT['ex-load'] = () => { const d = EX.data, r = planarRobot(d.L1, d.L2); loadRobot(r, EX.kind === 'fk' ? [d.q1 * D2R, d.q2 * D2R] : [0, 0]); };
  ACT['ex-check'] = () => {
    const a = getN('ex-a'), b = getN('ex-b'), d = EX.data; if (Number.isNaN(a) || Number.isNaN(b)) { toast('Enter both numbers.', true); return; } let ok, msg;
    if (EX.kind === 'fk') { ok = Math.hypot(a - d.x, b - d.y) <= 0.01; msg = ok ? 'Correct — within 0.01 m.' : 'Not yet: your point is ' + fx(Math.hypot(a - d.x, b - d.y), 3) + ' m away from the true position.'; }
    else { const x = d.L1 * Math.cos(a * D2R) + d.L2 * Math.cos((a + b) * D2R), y = d.L1 * Math.sin(a * D2R) + d.L2 * Math.sin((a + b) * D2R), e = Math.hypot(x - d.x, y - d.y); ok = e <= 0.01; msg = ok ? 'Correct — your angles reach the target (error ' + fx(e, 4) + ' m).' : 'Your angles reach (' + fx(x, 3) + ', ' + fx(y, 3) + '), which misses by ' + fx(e, 3) + ' m.'; }
    EX.msg = '<div class="note ' + (ok ? 'ok' : 'warn') + '">' + msg + '</div>'; $('#ex-msg').innerHTML = EX.msg;
  };
  ACT['ex-show'] = () => {
    const d = EX.data; let m; if (EX.kind === 'fk') m = 'x = ' + fx(d.x, 4) + ' m, y = ' + fx(d.y, 4) + ' m'; else { const r = planarRobot(d.L1, d.L2), a = RL.ik2R(r, d.x, d.y, 0); m = a.solutions.map(s => 'q = (' + fx(s.q[0] * R2D, 3) + '°, ' + fx(s.q[1] * R2D, 3) + '°)').join(' or '); }
    EX.msg = '<div class="note">Solution: ' + m + '</div>'; $('#ex-msg').innerHTML = EX.msg;
  };

  /* ============================ project / report ========================= */
  function saveProject() { download('robolab-project.json', JSON.stringify(projectObject(), null, 2), 'application/json'); toast('Project saved.'); }
  function buildReport() {
    const F = RL.fk(S.robot, S.q), J = RL.jacobianFromFrames(S.robot, F.frames), R = RL.Rof(F.T), p = RL.Pof(F.T), e = RL.rToRPY(R), n = S.robot.joints.length;
    const tbl = (M, d) => '<table class="m">' + M.map(r => '<tr>' + r.map(v => '<td>' + fx(v, d || 4) + '</td>').join('') + '</tr>').join('') + '</table>';
    const ik = S.ik.result ? '<p>' + (S.ik.result.converged ? 'Converged' : '<b>NOT converged</b>') + ' after ' + S.ik.result.iterations + ' iterations; position residual ' + S.ik.result.posError.toExponential(2) + ' m.</p>' : '<p>No IK result.</p>';
    return '<!doctype html><html lang="en"><head><meta charset="utf-8"><title>RoboLab Web report — ' + esc(S.robot.name) + '</title><style>body{font:14px/1.5 system-ui,sans-serif;max-width:860px;margin:24px auto;padding:0 16px;color:#111}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:3px 8px;text-align:right}th{background:#eee}table.m{border:0;margin:6px 0}table.m td{border:0;font-family:monospace}h1{margin-bottom:0}.muted{color:#555}@media print{button{display:none}}</style></head><body>' +
      '<button onclick="window.print()">Print / save as PDF</button><h1>RoboLab Web calculation report</h1><p class="muted">Robot: ' + esc(S.robot.name) + ' · generated ' + esc(new Date().toLocaleString()) + ' · units: metres, degrees (radians internally)</p><p>' + esc(S.robot.scope) + '</p>' +
      '<h2>DH parameters (classical)</h2><table><tr><th>i</th><th>name</th><th>type</th><th>a [m]</th><th>α [°]</th><th>d [m]</th><th>θ offset [°]</th><th>min</th><th>max</th><th>mass [kg]</th></tr>' + S.robot.joints.map((j, i) => '<tr><td>' + (i + 1) + '</td><td style="text-align:left">' + esc(j.name) + '</td><td>' + j.type + '</td><td>' + fx(j.a) + '</td><td>' + fx(j.alpha, 3) + '</td><td>' + fx(j.d) + '</td><td>' + fx(j.theta, 3) + '</td><td>' + fx(j.qmin, 3) + '</td><td>' + fx(j.qmax, 3) + '</td><td>' + fx(j.mass, 3) + '</td></tr>').join('') + '</table>' +
      '<h2>Joint values</h2><p>' + esc(qStr(S.q)) + '</p><h2>End-effector pose</h2><p>Position = ' + vecStr(p) + ' m<br>Roll/pitch/yaw (ZYX) = ' + vecStr([e.roll * R2D, e.pitch * R2D, e.yaw * R2D], 3) + '°</p>' + tbl(F.T) +
      '<h2>Link transforms A<sub>i</sub></h2>' + F.links.map((A, i) => '<p>A' + (i + 1) + '</p>' + tbl(A)).join('') + '<h2>Jacobian (base frame, 6×' + n + ')</h2>' + tbl(J) +
      '<h2>Inverse kinematics</h2><p>Target ' + vecStr(S.target.pos) + ' m</p>' + ik + '<h2>Trajectory</h2><p>' + (S.traj ? S.traj.t.length + ' samples over ' + fx(S.traj.t[S.traj.t.length - 1], 3) + ' s (' + esc(S.traj.type) + ')' + (S.trajWarn.length ? '<br>Warnings: ' + S.trajWarn.map(esc).join(' ') : '') : 'None generated.') + '</p>' +
      '<p class="muted">Educational tool. Results are numerical and have not been validated against physical hardware.</p></body></html>';
  }
  ACT['x-report'] = () => { const html = buildReport(); const url = URL.createObjectURL(new Blob([html], { type: 'text/html' })); const w = window.open(url, '_blank'); if (!w) { download('robolab-report.html', html, 'text/html'); toast('Pop-up blocked — report downloaded instead.'); } setTimeout(() => URL.revokeObjectURL(url), 60000); };

  /* ============================ UI wiring / boot ========================= */
  function applyTheme() {
    document.documentElement.setAttribute('data-theme', S.theme);
    if (G.ok) { G.scene.background = new THREE.Color(S.theme === 'light' ? 0xdfe6ed : 0x10151c); dirty.render = true; } dirty.plot = true;
  }
  function openModal(id) { $('#' + id).classList.add('open'); const b = $('#' + id + ' button'); if (b) b.focus(); } function closeModal(id) { $('#' + id).classList.remove('open'); }
  function buildWelcome() {
    const desc = { planar2: 'Two revolute joints in a plane. Best for learning the maths.', arm3: 'Yaw, shoulder and elbow. Classic 3-D arm.', scara4: 'R-R-P-R with a prismatic Z slide.', ur5like: 'Six revolute joints, UR5-style DH table.' };
    $('#welcome-cards').innerHTML = RL.PRESET_IDS.map(p => '<button class="robot-card" data-preset="' + p[0] + '"><b>' + esc(p[1]) + '</b><span class="muted">' + esc(desc[p[0]]) + '</span></button>').join('');
    $('#welcome-cards').addEventListener('click', e => { const b = e.target.closest('[data-preset]'); if (b) { loadPreset(b.dataset.preset); closeModal('welcome'); } });
    $('#w-close').addEventListener('click', () => closeModal('welcome'));
    $('#w-tutorial').addEventListener('click', () => { closeModal('welcome'); setMode('tut'); });
    $('#h-close').addEventListener('click', () => closeModal('help'));
    const hb = document.createElement('button'); hb.className = 'btn'; hb.textContent = 'Show welcome screen'; hb.addEventListener('click', () => { closeModal('help'); openModal('welcome'); }); $('#help .row').prepend(hb);
  }
  function initShortcuts() {
    document.addEventListener('keydown', e => {
      if (e.key === 'Escape') { closeModal('welcome'); closeModal('help'); return; }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 's') { e.preventDefault(); saveProject(); return; }
      const tg = e.target.tagName; if (e.ctrlKey || e.metaKey || e.altKey || tg === 'INPUT' || tg === 'SELECT' || tg === 'TEXTAREA' || $('#welcome').classList.contains('open')) return;
      const k = e.key.toLowerCase(), map = { f: 'fit', w: 'wire', g: 'grid', a: 'frames', c: 'ortho', t: 'clear' };
      if (map[k]) { viewAction(map[k]); e.preventDefault(); } else if (k === ' ') { if (tg !== 'BUTTON') { e.preventDefault(); togglePlay(); } }
      else if (k === '[') document.body.classList.toggle('left-collapsed'); else if (k === ']') document.body.classList.toggle('right-collapsed');
      else if (k === '?') openModal('help'); else if (/^[0-9]$/.test(k)) { const m = MODES[(+k + 9) % 10]; if (m) setMode(m[0]); }
    });
  }
  function initChrome() {
    $('#btn-left').addEventListener('click', () => document.body.classList.toggle('left-collapsed')); $('#btn-right').addEventListener('click', () => document.body.classList.toggle('right-collapsed'));
    $('#btn-bottom').addEventListener('click', () => { const c = document.body.classList.toggle('bottom-collapsed'); $('#btn-bottom').textContent = c ? '▴' : '▾'; dirty.plot = true; });
    $('#btn-save').addEventListener('click', saveProject); $('#btn-open').addEventListener('click', () => $('#file-open').click());
    $('#btn-help').addEventListener('click', () => openModal('help'));
    $('#btn-theme').addEventListener('click', () => { S.theme = S.theme === 'dark' ? 'light' : 'dark'; applyTheme(); scheduleSave(); });
    $('#file-open').addEventListener('change', e => { const f = e.target.files[0]; e.target.value = ''; if (!f) return; if (f.size > 5e6) { toast('File too large for a project (limit 5 MB).', true); return; } const rd = new FileReader(); rd.onload = () => importProjectText(String(rd.result)); rd.onerror = () => toast('Could not read the file.', true); rd.readAsText(f); });
    const h = $('#resize-bottom'); let drag = null;
    h.addEventListener('pointerdown', e => { drag = { y: e.clientY, h: $('#bottom').getBoundingClientRect().height }; h.setPointerCapture(e.pointerId); });
    h.addEventListener('pointermove', e => { if (!drag) return; const nh = RL.clamp(drag.h + (drag.y - e.clientY), 120, window.innerHeight * 0.7); document.documentElement.style.setProperty('--bottom-h', nh + 'px'); });
    h.addEventListener('pointerup', () => { drag = null; });
    window.addEventListener('beforeunload', () => { try { localStorage.setItem(STORE_KEY, JSON.stringify(projectObject())); } catch (e) { /* ignore */ } });
  }
  function boot() {
    buildNav(); buildViewTools(); initPanel(); initLeft(); initPlayback(); initChrome(); initShortcuts(); buildWelcome(); initPlotInteraction(); refreshPlotKinds();
    initGL();
    const saved = loadSaved(); if (saved) applySettings(saved.settings);
    applyTheme(); const first = saved ? saved.v.robot : RL.buildPreset('planar2');
    loadRobot(first, saved ? saved.v.q : null); if (G.ok) setProjection(S.view.ortho); syncViewButtons(); applyLight();
    if (G.ok) { G.grid.visible = S.view.grid; G.worldAxes.visible = S.view.axes; G.axesHelpers.forEach((a, i) => { a.visible = i === G.axesHelpers.length - 1 || S.view.frames; }); G.robotRoot.traverse(o => { if (o.userData.jointAxis) o.visible = S.view.jointAxes; }); fitView(); }
    setMode(S.mode, true); updatePlayUI();
    if (!saved) openModal('welcome'); else toast('Restored your last session.');
    requestAnimationFrame(frame);
  }
  window.RoboLab = { S, RL, G, boot, ACT, setMode, loadPreset };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot); else boot();
})();
