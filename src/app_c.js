
  /* ============================ panel plumbing =========================== */
  const PANELS = {}, LIVE = {}, CHANGE = {}, ACT = {};
  const r6 = (v) => String(+Number(v).toFixed(6));
  const fld = (id, label, val, extra) => '<label class="field"><span>' + label + '</span><input type="number" id="' + id + '" value="' + val + '" step="any" ' + (extra || '') + '></label>';
  const getN = (id, lo, hi) => { const el = $('#' + id); if (!el) return NaN; const v = parseFloat(el.value); const ok = Number.isFinite(v) && (lo === undefined || v >= lo) && (hi === undefined || v <= hi); el.classList.toggle('invalid', !ok); return ok ? v : NaN; };
  const readList = (specs) => { const out = specs.map(s => getN(s[0], s[1], s[2])); if (out.some(v => Number.isNaN(v))) { toast('Please correct the highlighted fields (numbers within range).', true); return null; } return out; };
  const matHTML = (M, p) => '<div class="scroll-x"><table class="mat br">' + M.map(r => '<tr>' + r.map(v => '<td>' + fx(v, p === undefined ? 4 : p) + '</td>').join('') + '</tr>').join('') + '</table></div>';
  const head = (t, d) => '<div class="sect"><h3 style="font-size:15px">' + t + '</h3><div class="muted">' + d + '</div></div>';
  const vecStr = (v, p) => '(' + v.map(x => fx(x, p === undefined ? 4 : p)).join(', ') + ')';
  const qStr = (q, sep) => q.map((v, i) => fx(toDisp(S.robot.joints[i], v), 3) + uPos(S.robot.joints[i]).trim()).join(sep || ', ');
  const openKeys = {};
  const dOpen = (k) => openKeys[k] ? ' open' : '';

  function renderPanel() {
    const p = $('#panel'); p.innerHTML = (PANELS[S.mode] ? PANELS[S.mode]() : ''); if (POST[S.mode]) POST[S.mode](); dirty.live = true;
  }
  const POST = {};
  function initPanel() {
    const p = $('#panel');
    p.addEventListener('change', e => { if (CHANGE[S.mode]) CHANGE[S.mode](e); });
    p.addEventListener('click', e => { const b = e.target.closest('[data-act]'); if (b && ACT[b.dataset.act]) ACT[b.dataset.act](b, e); });
    p.addEventListener('toggle', e => { const d = e.target; if (d.tagName === 'DETAILS' && d.dataset.k) openKeys[d.dataset.k] = d.open; }, true);
  }
  function buildNav() {
    $('#nav').innerHTML = MODES.map((m, i) => '<button role="tab" data-mode="' + m[0] + '" aria-selected="false" title="' + m[1] + ' (' + ((i + 1) % 10) + ')">' + m[1] + '</button>').join('');
    $('#nav').addEventListener('click', e => { const b = e.target.closest('button[data-mode]'); if (b) setMode(b.dataset.mode); });
  }
  const NUMRANGE = { a: [-100, 100], alpha: [-3600, 3600], d: [-100, 100], theta: [-3600, 3600], qmin: [-1e4, 1e4], qmax: [-1e4, 1e4], mass: [0, 1e6] };
  function setJointField(i, key, raw) {
    const j = S.robot.joints[i]; const v = parseFloat(raw), rg = NUMRANGE[key];
    if (!Number.isFinite(v) || v < rg[0] || v > rg[1]) { toast('Value for ' + key + ' must be a number between ' + rg[0] + ' and ' + rg[1] + '.', true); return false; }
    const old = j[key]; j[key] = v;
    if ((key === 'qmin' || key === 'qmax') && !(j.qmin < j.qmax)) { j[key] = old; toast('Lower limit must be below upper limit.', true); return false; }
    S.q = RL.clampQ(S.robot, S.q); return true;
  }
  function geometryChanged() { S.q = RL.clampQ(S.robot, S.q); invalidateResults(); structureChanged(); scheduleSave(); }
  function topologyChanged() { invalidateResults(); resizeStateArrays(); S.trajP.q0 = S.q.slice(); S.trajP.qf = S.q.slice(); structureChanged(); scheduleSave(); }

  /* ============================== Builder ================================ */
  PANELS.builder = () => {
    const r = S.robot, j = r.joints[S.sel], lib = getLibrary(), t = (id, l, v) => '<label class="field"><span>' + l + '</span><input type="text" id="' + id + '" value="' + esc(v) + '" maxlength="60"></label>';
    const lp = j.type === 'R' ? '°' : ' m';
    return head('Robot Builder', 'Build a serial manipulator joint by joint. Every change updates the 3D model immediately.') +
      '<div class="sect"><h3>Robot</h3>' + t('b-rname', 'Robot name', r.name) +
      '<div class="row"><button class="btn small" data-act="b-add" title="Insert a revolute joint after the selected joint">Add joint</button><button class="btn small danger" data-act="b-del" title="Remove the selected joint">Remove joint</button><button class="btn small" data-act="b-dupj" title="Duplicate the selected joint">Duplicate joint</button></div>' +
      '<div class="row"><button class="btn small" data-act="b-sethome" title="Store the current joint values as the home configuration">Set home = current pose</button><button class="btn small" data-act="b-reset" title="Reload the original preset (or a blank robot)">Reset robot</button><button class="btn small" data-act="b-savelib" title="Keep a copy of this robot in your browser library">Save copy</button></div>' +
      (lib.length ? '<h4>Your saved copies</h4>' + lib.map((x, i) => '<div class="row"><span class="grow">' + esc(x.name) + ' <span class="muted">(' + x.joints.length + ' DOF)</span></span><button class="btn small" data-act="b-loadlib" data-i="' + i + '">Load</button><button class="btn small danger" data-act="b-dellib" data-i="' + i + '" aria-label="Delete copy ' + esc(x.name) + '">✕</button></div>').join('') : '') + '</div>' +
      '<div class="sect"><h3>Selected joint: q<sub>' + (S.sel + 1) + '</sub></h3><div class="grid2">' + t('b-name', 'Joint name', j.name) + t('b-link', 'Link name', j.linkName) + '</div>' +
      '<div class="grid2"><label class="field"><span>Joint type</span><select id="b-type"><option value="R"' + (j.type === 'R' ? ' selected' : '') + '>Revolute (rotates about z)</option><option value="P"' + (j.type === 'P' ? ' selected' : '') + '>Prismatic (slides along z)</option></select></label></div>' +
      '<h4>DH geometry (classical convention)</h4><div class="grid2">' + fld('b-a', 'Link length a [m]', r6(j.a)) + fld('b-alpha', 'Link twist α [°]', r6(j.alpha)) + fld('b-d', 'Link offset d [m]' + (j.type === 'P' ? ' (fixed part)' : ''), r6(j.d)) + fld('b-theta', 'Joint angle θ [°]' + (j.type === 'R' ? ' (offset)' : ' (fixed)'), r6(j.theta)) + '</div>' +
      '<h4>Joint limits [' + lp.trim() + ']</h4><div class="grid2">' + fld('b-qmin', 'Minimum', r6(j.qmin)) + fld('b-qmax', 'Maximum', r6(j.qmax)) + '</div>' +
      '<h4>Mass properties (for dynamics)</h4><div class="grid2">' + fld('b-mass', 'Mass [kg]', r6(j.mass), 'min="0"') + '</div>' +
      '<div class="muted">Centre of mass in link frame {' + (S.sel + 1) + '} [m]</div><div class="grid3">' + [0, 1, 2].map(k => fld('b-com' + k, 'xyz'[k], r6(j.com[k]))).join('') + '</div>' +
      '<div class="muted">Principal inertia about the COM, in link-frame axes [kg·m²]</div><div class="grid3">' + [0, 1, 2].map(k => fld('b-I' + k, ['Ixx', 'Iyy', 'Izz'][k], r6(j.inertia[k]), 'min="0"')).join('') + '</div>' +
      '<div class="row"><button class="btn small" data-act="b-auto" title="Place the COM at the middle of the link and use a slender-rod inertia estimate">Estimate COM & inertia from geometry</button></div></div>' +
      '<div class="sect"><h3>Gravity [m/s²] in base frame</h3><div class="grid3">' + [0, 1, 2].map(k => fld('b-g' + k, 'xyz'[k], r6(r.gravity[k]))).join('') + '</div>' +
      '<div class="note">Presets are simplified educational models. Procedural geometry only — no manufacturer CAD. Prismatic joints extend the link offset d along z; the 3D link length updates live.</div></div>';
  };
  CHANGE.builder = (e) => {
    const t = e.target, id = t.id, i = S.sel, j = S.robot.joints[i]; if (!id) return;
    if (id === 'b-rname') { S.robot.name = t.value.trim().slice(0, 60) || 'Robot'; renderLeft(); updateStatus(); scheduleSave(); return; }
    if (id === 'b-name') { j.name = t.value.trim().slice(0, 60) || 'Joint ' + (i + 1); renderLeft(); scheduleSave(); return; }
    if (id === 'b-link') { j.linkName = t.value.trim().slice(0, 60) || 'Link ' + (i + 1); scheduleSave(); return; }
    if (id === 'b-type') { j.type = t.value === 'P' ? 'P' : 'R'; if (j.type === 'P') { j.qmin = 0; j.qmax = 0.5; } else { j.qmin = -180; j.qmax = 180; } S.q[i] = RL.clamp(0, RL.jointLimitsSI(j)[0], RL.jointLimitsSI(j)[1]); S.robot.home[i] = S.q[i]; geometryChanged(); return; }
    const m = id.match(/^b-(a|alpha|d|theta|qmin|qmax|mass)$/); if (m) { if (setJointField(i, m[1], t.value)) geometryChanged(); else renderPanel(); return; }
    const c = id.match(/^b-(com|I)(\d)$/); if (c) { const v = parseFloat(t.value), k = +c[2]; const ok = Number.isFinite(v) && Math.abs(v) <= (c[1] === 'com' ? 100 : 1e6) && (c[1] === 'com' || v >= 0); if (!ok) { toast('Enter a valid ' + (c[1] === 'I' ? 'non-negative inertia.' : 'coordinate.'), true); renderPanel(); return; } (c[1] === 'com' ? j.com : j.inertia)[k] = v; parametersChanged(true); return; }
    const g = id.match(/^b-g(\d)$/); if (g) { const v = parseFloat(t.value); if (!Number.isFinite(v) || Math.abs(v) > 1e3) { toast('Gravity component must be a finite number.', true); renderPanel(); return; } S.robot.gravity[+g[1]] = v; parametersChanged(true); }
  };
  ACT['b-add'] = () => {
    if (S.robot.joints.length >= 12) { toast('Maximum is 12 joints.', true); return; }
    const k = S.sel + 1, nj = RL.makeJoint({ name: 'Joint ' + (S.robot.joints.length + 1), linkName: 'Link ' + (S.robot.joints.length + 1), a: 0.3, mass: 1 }); RL.autoInertial(nj);
    S.robot.joints.splice(k, 0, nj); S.robot.home.splice(k, 0, 0); S.q.splice(k, 0, 0); S.sel = k; topologyChanged();
  };
  ACT['b-del'] = () => {
    if (S.robot.joints.length <= 1) { toast('A robot needs at least one joint.', true); return; }
    S.robot.joints.splice(S.sel, 1); S.robot.home.splice(S.sel, 1); S.q.splice(S.sel, 1); S.sel = Math.max(0, S.sel - 1); topologyChanged();
  };
  ACT['b-dupj'] = () => {
    if (S.robot.joints.length >= 12) { toast('Maximum is 12 joints.', true); return; }
    const c = RL.cloneRobot({ joints: [S.robot.joints[S.sel]] }).joints[0]; c.name += ' copy'; S.robot.joints.splice(S.sel + 1, 0, c); S.robot.home.splice(S.sel + 1, 0, S.q[S.sel]); S.q.splice(S.sel + 1, 0, S.q[S.sel]); S.sel++; topologyChanged();
  };
  ACT['b-sethome'] = () => { S.robot.home = S.q.slice(); toast('Home configuration stored.'); scheduleSave(); };
  ACT['b-reset'] = () => { const id = S.robot.id; try { loadRobot(RL.buildPreset(RL.PRESET_IDS.some(p => p[0] === id) || id === 'blank1' ? id : 'blank1')); toast('Robot reset.'); } catch (e) { loadPreset('blank1'); } };
  ACT['b-auto'] = () => { RL.autoInertial(S.robot.joints[S.sel]); parametersChanged(false); toast('Mass properties estimated from link geometry (slender rod).'); };
  ACT['b-savelib'] = () => { const l = getLibrary(); l.unshift(RL.cloneRobot(S.robot)); setLibrary(l.slice(0, 20)); renderPanel(); toast('Copy saved in this browser.'); };
  ACT['b-loadlib'] = (b) => { const r = getLibrary()[+b.dataset.i]; if (r) { const v = RL.validateRobot(r); if (v.ok) loadRobot(v.robot); } };
  ACT['b-dellib'] = (b) => { const l = getLibrary(); l.splice(+b.dataset.i, 1); setLibrary(l); renderPanel(); };

  /* ================================ DH =================================== */
  PANELS.dh = () => {
    const rows = S.robot.joints.map((j, i) => '<tr data-i="' + i + '"' + (i === S.sel ? ' class="sel"' : '') + '><td>' + (i + 1) + '</td><td><select id="dh-' + i + '-type" aria-label="Joint ' + (i + 1) + ' type"><option value="R"' + (j.type === 'R' ? ' selected' : '') + '>R</option><option value="P"' + (j.type === 'P' ? ' selected' : '') + '>P</option></select></td>' +
      ['a', 'alpha', 'd', 'theta', 'qmin', 'qmax'].map(k => '<td><input type="number" step="any" id="dh-' + i + '-' + k + '" value="' + r6(j[k]) + '" aria-label="' + k + ' joint ' + (i + 1) + '"></td>').join('') + '</tr>').join('');
    return head('Denavit–Hartenberg parameters', 'Classical (distal) DH convention. Edit the table and the robot updates.') +
      '<div class="sect"><div class="formula">T<sub>i−1,i</sub> = R<sub>z</sub>(θ<sub>i</sub>) · T<sub>z</sub>(d<sub>i</sub>) · T<sub>x</sub>(a<sub>i</sub>) · R<sub>x</sub>(α<sub>i</sub>)</div>' +
      '<div class="scroll-x"><table class="data" id="dh-table"><tr><th>i</th><th>type</th><th>a [m]</th><th>α [°]</th><th>d [m]</th><th>θ off. [°]</th><th>min</th><th>max</th></tr>' + rows + '</table></div>' +
      '<div class="muted" style="margin-top:6px">Revolute: θ<sub>i</sub> = offset + q<sub>i</sub>. Prismatic: d<sub>i</sub> = offset + q<sub>i</sub>. Limits are in ° (revolute) or m (prismatic). Modified (proximal) DH is not implemented in this version.</div>' +
      '<div class="row"><button class="btn small" data-act="dh-csv">Export DH table (CSV)</button><button class="btn small" data-act="dh-mats">Export matrices (CSV)</button></div></div>' +
      '<div class="sect"><h3>Transformation matrices at the current pose</h3><div id="dh-live"></div></div>';
  };
  CHANGE.dh = (e) => {
    const m = e.target.id.match(/^dh-(\d+)-(\w+)$/); if (!m) return; const i = +m[1], k = m[2];
    if (k === 'type') { const j = S.robot.joints[i]; j.type = e.target.value; if (j.type === 'P') { j.qmin = 0; j.qmax = 0.5; } else { j.qmin = -180; j.qmax = 180; } S.q[i] = RL.clamp(0, ...RL.jointLimitsSI(j)); S.robot.home[i] = S.q[i]; geometryChanged(); return; }
    if (setJointField(i, k, e.target.value)) geometryChanged(); else renderPanel();
  };
  function frameDetails(F) {
    const n = S.robot.joints.length; let h = '';
    for (let i = 0; i < n; i++) {
      const j = S.robot.joints[i], th = (j.type === 'R' ? j.theta * D2R + S.q[i] : j.theta * D2R), d = j.d + (j.type === 'P' ? S.q[i] : 0);
      h += '<details data-k="A' + i + '"' + dOpen('A' + i) + '><summary><b>Link ' + (i + 1) + '</b> <span class="muted">a=' + fx(j.a, 3) + ' m, α=' + fx(j.alpha, 2) + '°, d=' + fx(d, 4) + ' m, θ=' + fx(th * R2D, 2) + '°</span></summary>' +
        '<div class="muted">A<sub>' + (i + 1) + '</sub> = <sup>' + i + '</sup>T<sub>' + (i + 1) + '</sub></div>' + matHTML(F.links[i]) + '<div class="muted">Cumulative <sup>0</sup>T<sub>' + (i + 1) + '</sub> = <sup>0</sup>T<sub>' + i + '</sub> · A<sub>' + (i + 1) + '</sub></div>' + matHTML(F.frames[i + 1]) + '</details>';
    }
    return h;
  }
  LIVE.dh = () => {
    const el = $('#dh-live'); if (!el || !S.fkCache) return; const F = S.fkCache, n = S.robot.joints.length;
    el.innerHTML = frameDetails(F) + '<h4>Final transformation base → end-effector</h4><div class="muted"><sup>0</sup>T<sub>' + n + '</sub> = A<sub>1</sub>A<sub>2</sub>…A<sub>' + n + '</sub></div>' + matHTML(F.T) + '<div class="kv"><span>Position</span><span>' + vecStr(RL.Pof(F.T)) + ' m</span></div>';
  };
  ACT['dh-csv'] = () => { const rows = [['i', 'name', 'type', 'a_m', 'alpha_deg', 'd_m', 'theta_offset_deg', 'qmin', 'qmax']]; S.robot.joints.forEach((j, i) => rows.push([i + 1, j.name, j.type, j.a, j.alpha, j.d, j.theta, j.qmin, j.qmax])); download('robolab-dh-table.csv', toCSV(rows), 'text/csv'); };
  ACT['dh-mats'] = () => { const F = RL.fk(S.robot, S.q), rows = [['Matrix', 'row', 'c1', 'c2', 'c3', 'c4']]; F.links.forEach((A, i) => A.forEach((r, k) => rows.push(['A' + (i + 1) + ' (' + i + 'T' + (i + 1) + ')', k + 1].concat(r)))); F.frames.slice(1).forEach((A, i) => A.forEach((r, k) => rows.push(['0T' + (i + 1), k + 1].concat(r)))); download('robolab-transforms.csv', toCSV(rows), 'text/csv'); };

  /* ======================== Forward kinematics =========================== */
  PANELS.fk = () => head('Forward kinematics', 'Move the joint sliders (left) and read the end-effector pose computed by chaining the DH matrices.') +
    '<div class="sect"><div class="formula"><sup>0</sup>T<sub>n</sub>(q) = A<sub>1</sub>(q<sub>1</sub>) · A<sub>2</sub>(q<sub>2</sub>) ⋯ A<sub>n</sub>(q<sub>n</sub>)</div><div id="fk-live"></div></div>' +
    '<div class="sect"><h3>Worked reference: 2-link planar arm</h3><div class="formula">x = L<sub>1</sub>cos q<sub>1</sub> + L<sub>2</sub>cos(q<sub>1</sub>+q<sub>2</sub>)<br>y = L<sub>1</sub>sin q<sub>1</sub> + L<sub>2</sub>sin(q<sub>1</sub>+q<sub>2</sub>)</div>' +
    '<div class="grid4">' + fld('fr-l1', 'L₁ [m]', 1) + fld('fr-l2', 'L₂ [m]', 1) + fld('fr-q1', 'q₁ [°]', 30) + fld('fr-q2', 'q₂ [°]', 45) + '</div>' +
    '<div class="row"><button class="btn small" data-act="fk-ref">Compare formula with the matrix engine</button></div><div id="fk-ref-out"></div></div>';
  LIVE.fk = () => {
    const el = $('#fk-live'); if (!el || !S.fkCache) return; const F = S.fkCache, R = RL.Rof(F.T), p = RL.Pof(F.T), e = RL.rToRPY(R), c = RL.rotationCheck(R), n = S.robot.joints.length;
    el.innerHTML = '<h4>End-effector position (base frame)</h4><div class="kv"><span>x</span><span>' + fx(p[0]) + ' m</span><span>y</span><span>' + fx(p[1]) + ' m</span><span>z</span><span>' + fx(p[2]) + ' m</span></div>' +
      '<h4>Orientation — rotation matrix R (columns are the tool axes in the base frame)</h4>' + matHTML(R) +
      '<h4>Orientation — Euler angles, ZYX convention (R = R<sub>z</sub>(yaw)·R<sub>y</sub>(pitch)·R<sub>x</sub>(roll))</h4><div class="kv"><span>roll</span><span>' + fx(e.roll * R2D, 3) + '°</span><span>pitch</span><span>' + fx(e.pitch * R2D, 3) + '°</span><span>yaw</span><span>' + fx(e.yaw * R2D, 3) + '°</span></div>' +
      (e.gimbal ? '<div class="note warn">Gimbal lock (pitch = ±90°): roll is set to 0 and yaw carries the combined rotation.</div>' : '') +
      '<h4>Homogeneous transform <sup>0</sup>T<sub>' + n + '</sub></h4>' + matHTML(F.T) + '<div class="muted">Check: max |RᵀR − I| = ' + c.orthError.toExponential(1) + ', det R = ' + fx(c.det, 12) + '</div>' +
      '<h4>Calculation sequence</h4><ol class="steps"><li>Build each A<sub>i</sub> from (a, α, d, θ) with θ = offset + q (revolute) or d = offset + q (prismatic).</li><li>Multiply left to right: <sup>0</sup>T<sub>i</sub> = <sup>0</sup>T<sub>i−1</sub>·A<sub>i</sub>.</li><li>Read position from the last column and orientation from the upper-left 3×3 block.</li></ol>' + frameDetails(F);
  };
  ACT['fk-ref'] = () => {
    const v = readList([['fr-l1', 0.001, 100], ['fr-l2', 0.001, 100], ['fr-q1', -3600, 3600], ['fr-q2', -3600, 3600]]); if (!v) return;
    const r = RL.buildPreset('planar2'); r.joints[0].a = v[0]; r.joints[1].a = v[1]; const q = [v[2] * D2R, v[3] * D2R], T = RL.fk(r, q).T;
    const x = v[0] * Math.cos(q[0]) + v[1] * Math.cos(q[0] + q[1]), y = v[0] * Math.sin(q[0]) + v[1] * Math.sin(q[0] + q[1]);
    $('#fk-ref-out').innerHTML = '<div class="kv"><span>Formula</span><span>x = ' + fx(x, 6) + ', y = ' + fx(y, 6) + '</span><span>Matrix engine</span><span>x = ' + fx(T[0][3], 6) + ', y = ' + fx(T[1][3], 6) + '</span><span>Difference</span><span>' + Math.hypot(x - T[0][3], y - T[1][3]).toExponential(2) + ' m</span></div><div class="note ok">The general DH engine reproduces the closed-form 2-link result.</div>';
  };

  /* ========================= Inverse kinematics ========================== */
  PANELS.ik = () => head('Inverse kinematics', 'Find joint values that place the end-effector at a target. Shift+click in the 3D view to set the target position.') +
    '<div class="sect"><h3>Target pose (base frame)</h3><div class="grid3">' + fld('ik-x', 'x [m]', r6(S.target.pos[0])) + fld('ik-y', 'y [m]', r6(S.target.pos[1])) + fld('ik-z', 'z [m]', r6(S.target.pos[2])) + '</div>' +
    '<label class="row"><input type="checkbox" id="ik-ori"' + (S.target.useOri ? ' checked' : '') + '> Also match orientation (6-DOF pose target)</label>' +
    '<div class="grid3">' + fld('ik-roll', 'roll [°]', r6(S.target.rpy[0])) + fld('ik-pitch', 'pitch [°]', r6(S.target.rpy[1])) + fld('ik-yaw', 'yaw [°]', r6(S.target.rpy[2])) + '</div>' +
    '<div class="row"><button class="btn small" data-act="ik-cur">Target = current end-effector pose</button></div>' +
    '<details data-k="ikopt"' + dOpen('ikopt') + '><summary>Solver settings</summary><div class="grid4">' + fld('ik-lam', 'damping λ', r6(S.ikOpt.lambda)) + fld('ik-tol', 'tolerance [m]', S.ikOpt.tol) + fld('ik-it', 'max iter', S.ikOpt.maxIter) + fld('ik-ms', 'restarts', S.ikOpt.starts) + '</div>' +
    '<div class="formula">Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e</div><div class="muted">Damped least squares with adaptive damping and a step limit; joint limits are enforced after every step. Orientation tolerance is 0.001 rad.</div></details></div>' +
    '<div class="sect"><div class="row"><button class="btn primary" data-act="ik-num">Solve (numerical DLS)</button><button class="btn" data-act="ik-multi" title="Run DLS from the current pose plus random restarts to find several solutions">Find multiple solutions</button><button class="btn" data-act="ik-ana" id="ik-ana-btn" title="Closed-form solution for 2-link planar RR robots">Analytic (2R)</button></div><div id="ik-live" class="muted"></div></div>' +
    '<div class="sect"><h3>Result</h3><div id="ik-out"></div></div>';
  POST.ik = () => { renderIKResults(); const b = $('#ik-ana-btn'); if (b) { b.disabled = !RL.isPlanar2R(S.robot); b.title = b.disabled ? 'Analytic solver applies only to a 2-link planar RR robot' : b.title; } };
  function fillTargetInputs() { const s = (id, v) => { const e = $('#' + id); if (e) e.value = r6(v); }; s('ik-x', S.target.pos[0]); s('ik-y', S.target.pos[1]); s('ik-z', S.target.pos[2]); s('ik-roll', S.target.rpy[0]); s('ik-pitch', S.target.rpy[1]); s('ik-yaw', S.target.rpy[2]); }
  function readTarget() {
    const v = readList([['ik-x', -100, 100], ['ik-y', -100, 100], ['ik-z', -100, 100], ['ik-roll', -3600, 3600], ['ik-pitch', -3600, 3600], ['ik-yaw', -3600, 3600], ['ik-lam', 1e-4, 5], ['ik-tol', 1e-9, 1], ['ik-it', 1, 5000], ['ik-ms', 1, 40]]); if (!v) return false;
    S.target.pos = v.slice(0, 3); S.target.rpy = v.slice(3, 6); S.target.useOri = $('#ik-ori').checked; S.ikOpt = { lambda: v[6], tol: v[7], maxIter: Math.round(v[8]), starts: Math.round(v[9]) }; updateTargetPose(); scheduleSave(); return true;
  }
  CHANGE.ik = (e) => { if (e.target.id && e.target.id.startsWith('ik-') && e.target.type !== 'button') { if (readTarget()) dirty.live = true; } };
  LIVE.ik = () => {
    const el = $('#ik-live'); if (!el || !S.fkCache) return; const p = RL.Pof(S.fkCache.T), d = V.norm(V.sub(S.target.pos, p)), reach = RL.reachEstimate(S.robot);
    el.innerHTML = 'Distance current → target: <b>' + fx(d, 4) + ' m</b> · target distance from base: ' + fx(V.norm(S.target.pos), 3) + ' m · upper bound of reach ≈ ' + fx(reach, 3) + ' m.';
  };
  ACT['ik-cur'] = () => { const T = S.fkCache.T, e = RL.rToRPY(RL.Rof(T)), p = RL.Pof(T); S.target.pos = p.slice(); S.target.rpy = [e.roll * R2D, e.pitch * R2D, e.yaw * R2D]; fillTargetInputs(); updateTargetPose(); scheduleSave(); };
  function ikTarget() { return { pos: S.target.pos.slice(), R: RL.rpyToR(S.target.rpy[0] * D2R, S.target.rpy[1] * D2R, S.target.rpy[2] * D2R) }; }
  function ikOptions() { return { lambda: S.ikOpt.lambda, tol: S.ikOpt.tol, maxIter: S.ikOpt.maxIter, useOri: S.target.useOri }; }
  const solRecord = (label, r) => ({ label, q: r.q, converged: r.converged, withinLimits: r.withinLimits, posError: r.posError, oriError: r.oriError, iterations: r.iterations, reason: r.reason });
  ACT['ik-num'] = () => {
    if (!readTarget()) return; const r = RL.ikDLS(S.robot, ikTarget(), S.q, ikOptions()); r.useOri = S.target.useOri; S.ik = { result: r, sols: [solRecord('Numerical DLS from current pose', r)], analytic: null, message: '' };
    setPlotKind('ikErr'); renderIKResults();
  };
  ACT['ik-multi'] = () => {
    if (!readTarget()) return; const t0 = performance.now(), m = RL.ikMulti(S.robot, ikTarget(), S.q, ikOptions(), S.ikOpt.starts, 99), sols = m.solutions.map((s, i) => solRecord('Solution ' + (i + 1) + (i === 0 ? ' (closest to current pose)' : ''), s));
    const best = m.solutions[0] || m.attempts.slice().sort((a, b) => a.posError - b.posError)[0]; best.useOri = S.target.useOri;
    S.ik = { result: best, sols: sols.length ? sols : [solRecord('Best failed attempt', best)], analytic: null, message: m.attempts.length + ' starts in ' + Math.round(performance.now() - t0) + ' ms; ' + m.solutions.length + ' distinct valid solution(s).' };
    setPlotKind('ikErr'); renderIKResults();
  };
  ACT['ik-ana'] = () => {
    if (!readTarget()) return; const a = RL.ik2R(S.robot, S.target.pos[0], S.target.pos[1], S.target.pos[2]);
    const sols = a.solutions.map(s => { const p = RL.Pof(RL.fk(S.robot, s.q).T), err = V.norm(V.sub(p, S.target.pos)); return { label: 'Analytic — ' + s.label, q: s.q, converged: err < 1e-6, withinLimits: s.withinLimits, posError: err, oriError: 0, iterations: 0, reason: 'closed form' }; });
    S.ik = { result: null, sols, analytic: a, message: a.message }; renderIKResults();
  };
  function renderIKResults() {
    const el = $('#ik-out'); if (!el) return; const r = S.ik.result, sols = S.ik.sols;
    if (!sols.length && !S.ik.analytic) { el.innerHTML = '<div class="muted">No solution computed yet. Set a target and press a solve button.</div>'; return; }
    let h = ''; if (S.ik.message) h += '<div class="note' + (S.ik.analytic && !S.ik.analytic.reachable ? ' bad' : '') + '">' + esc(S.ik.message) + '</div>';
    if (r) h += '<div class="row">' + (r.converged ? '<span class="badge ok">Converged</span>' : '<span class="badge bad">Not converged</span>') + '<span class="muted">' + r.iterations + ' iterations · ' + esc(r.reason) + '</span></div>' +
      (r.likelyUnreachable ? '<div class="note bad">Target is farther from the base than the robot can stretch — it is unreachable.</div>' : (!r.converged ? '<div class="note warn">The solver stopped with residual ' + fx(r.posError, 5) + ' m' + (S.target.useOri ? ' / ' + fx(r.oriError, 4) + ' rad' : '') + '. The target may be unreachable (joint limits, singularity or orientation constraint) or a local minimum — try “Find multiple solutions”.</div>' : ''));
    h += sols.map((s, i) => '<div class="card"><div class="row"><b>' + esc(s.label) + '</b><span class="grow"></span>' + (s.converged ? '<span class="badge ok">reached</span>' : '<span class="badge bad">not reached</span>') + (s.withinLimits ? '' : '<span class="badge warn">violates limits</span>') + '</div>' +
      '<div class="mono">q = [' + qStr(s.q) + ']</div><div class="muted">position residual ' + s.posError.toExponential(2) + ' m' + (S.target.useOri ? ', orientation residual ' + s.oriError.toExponential(2) + ' rad' : '') + ' (verified by forward kinematics)</div>' +
      '<div class="row"><button class="btn small" data-act="ik-apply" data-i="' + i + '">Apply pose</button><button class="btn small primary" data-act="ik-anim" data-i="' + i + '">Animate to this solution</button></div></div>').join('');
    el.innerHTML = h;
  }
  ACT['ik-apply'] = (b) => { const s = S.ik.sols[+b.dataset.i]; if (!s) return; S.play.playing = false; setQ(s.q.slice()); if (!s.converged) toast('Note: this configuration does not reach the target.', true); };
  ACT['ik-anim'] = (b) => {
    const s = S.ik.sols[+b.dataset.i]; if (!s) return; if (!s.converged) toast('Animating to a configuration that does NOT reach the target.', true);
    const tr = RL.jointTrajectory({ type: 'quintic', q0: S.q.slice(), qf: s.q.slice(), T: 1.5, dt: 0.02 }); setPlayData(tr, 'IK motion', true);
  };

  /* ============================== Jacobian =============================== */
  PANELS.jac = () => head('Jacobian analysis', 'Geometric Jacobian in the base frame: [v; ω] = J(q) q̇.') +
    '<div class="sect"><div id="jac-live"></div></div>' +
    '<div class="sect"><h3>Joint velocity → end-effector velocity</h3><div class="grid3">' + S.robot.joints.map((j, i) => fld('jq-' + i, 'q̇' + (i + 1) + ' [' + uVel(j).trim() + ']', r6(toDisp(j, S.jacVel[i] || 0)))).join('') + '</div><div id="jac-fwd"></div></div>' +
    '<div class="sect"><h3>Cartesian velocity → joint velocity</h3><div class="grid3">' + ['vx [m/s]', 'vy [m/s]', 'vz [m/s]', 'ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]'].map((l, k) => fld('jt-' + k, l, r6(S.jacTwist[k]))).join('') + '</div>' +
    '<label class="row"><input type="checkbox" id="jt-ang"' + (S.jacAng ? ' checked' : '') + '> Include angular velocity (6 equations)</label><div id="jac-inv"></div></div>';
  CHANGE.jac = (e) => {
    const id = e.target.id; if (id.startsWith('jq-')) { const i = +id.slice(3), v = parseFloat(e.target.value); if (Number.isFinite(v)) S.jacVel[i] = fromDisp(S.robot.joints[i], v); }
    else if (id.startsWith('jt-') && id !== 'jt-ang') { const v = parseFloat(e.target.value); if (Number.isFinite(v)) S.jacTwist[+id.slice(3)] = v; }
    else if (id === 'jt-ang') S.jacAng = e.target.checked; dirty.live = true;
  };
  LIVE.jac = () => {
    const el = $('#jac-live'); if (!el || !S.fkCache) return; const J = RL.jacobianFromFrames(S.robot, S.fkCache.frames), n = S.robot.joints.length, an3 = RL.analyzeJacobian(J, 3), an6 = n >= 6 ? RL.analyzeJacobian(J, 6) : null;
    let Jfd = null, dif = NaN; try { Jfd = RL.jacobianFD(S.robot, S.q); dif = RL.maxAbsDiff(J, Jfd); } catch (e) { /* ignore */ }
    const rl = ['vx', 'vy', 'vz', 'ωx', 'ωy', 'ωz'];
    const lab = '<div class="scroll-x"><table class="mat br">' + J.map((r, i) => '<tr><td class="muted" style="text-align:left">' + rl[i] + '</td>' + r.map(v => '<td>' + fx(v, 4) + '</td>').join('') + '</tr>').join('') + '</table></div>';
    const warn = (a, nm) => a.singular ? '<div class="note bad">Singular or near-singular configuration for ' + nm + ' (σ<sub>min</sub> = ' + a.smin.toExponential(2) + '). Some end-effector motions are impossible and joint velocities may blow up.</div>' : '<div class="note ok">Not singular for ' + nm + ' (σ<sub>min</sub> = ' + a.smin.toExponential(2) + ').</div>';
    el.innerHTML = '<h4>J(q), 6×' + n + ' (columns = joints; rows 1–3 linear, 4–6 angular, base frame)</h4>' + lab +
      '<div class="muted">Revolute column: [zᵢ₋₁ × (pₑ − pᵢ₋₁); zᵢ₋₁]. Prismatic column: [zᵢ₋₁; 0].</div>' +
      '<h4>Linear part J<sub>v</sub> (3×' + n + ')</h4><div class="kv"><span>Singular values</span><span>' + an3.sv.map(s => fx(s, 4)).join(', ') + '</span><span>Condition number</span><span>' + (Number.isFinite(an3.cond) ? fx(an3.cond, 2) : '∞') + '</span><span>Manipulability w</span><span>' + fx(an3.manipulability, 5) + '</span></div>' + warn(an3, 'position') +
      (an6 ? '<h4>Full Jacobian (6×6)</h4><div class="kv"><span>Singular values</span><span>' + an6.sv.map(s => fx(s, 4)).join(', ') + '</span><span>Manipulability w</span><span>' + fx(an6.manipulability, 5) + ' (= |det J|)</span></div>' + warn(an6, 'full pose') : '') +
      '<div class="muted">Yoshikawa manipulability w = √det(J Jᵀ) = product of singular values (units mix m and rad for the full Jacobian).</div>' +
      '<div class="kv"><span>Finite-difference check</span><span>max |J − J<sub>FD</sub>| = ' + (Number.isFinite(dif) ? dif.toExponential(2) : '—') + '</span></div>';
    const qd = S.jacVel.slice(), tw = Mx.mulVec(J, qd); $('#jac-fwd').innerHTML = '<div class="kv"><span>v (m/s)</span><span>' + vecStr(tw.slice(0, 3), 5) + '</span><span>ω (rad/s)</span><span>' + vecStr(tw.slice(3), 5) + '</span></div>';
    const rows = S.jacAng ? 6 : 3, res = RL.jointVelocityFromTwist(J, S.jacTwist, rows, 1e-3), inv = $('#jac-inv');
    if (!res) inv.innerHTML = '<div class="note bad">The damped system could not be solved.</div>';
    else inv.innerHTML = '<div class="mono">q̇ = [' + res.qd.map((v, i) => fx(toDisp(S.robot.joints[i], v), 4) + uVel(S.robot.joints[i]).trim()).join(', ') + ']</div>' + (res.residual > 1e-3 ? '<div class="note warn">Not exactly achievable here (residual ' + res.residual.toExponential(2) + '): the commanded velocity has a component the arm cannot produce at this pose; the damped solution is the closest approximation.</div>' : '<div class="note ok">Achieved: |J q̇ − v| = ' + res.residual.toExponential(2) + '.</div>');
  };

  /* ============================= Trajectory ============================== */
  PANELS.traj = () => {
    const P = S.trajP, J = S.robot.joints, row = (i) => { const j = J[i]; return '<tr><td>q' + (i + 1) + ' <span class="muted">' + uPos(j).trim() + '</span></td>' + ['q0', 'qf', 'v0', 'vf', 'a0', 'af'].map(k => { const sc = k[0] === 'q' ? 1 : 1; void sc; const val = toDisp(j, P[k][i] || 0); return '<td><input type="number" step="any" id="tp-' + k + '-' + i + '" value="' + r6(val) + '" aria-label="' + k + ' joint ' + (i + 1) + '"></td>'; }).join('') + '</tr>'; };
    return head('Trajectory planning', 'Move between two configurations with polynomial time-scaling, or along a Cartesian straight line using IK.') +
      '<div class="sect"><div class="grid2"><label class="field"><span>Space</span><select id="tp-space"><option value="joint"' + (P.space === 'joint' ? ' selected' : '') + '>Joint space</option><option value="cart"' + (P.space === 'cart' ? ' selected' : '') + '>Cartesian line (position, uses IK)</option></select></label>' +
      '<label class="field"><span>Profile</span><select id="tp-type"><option value="linear"' + (P.type === 'linear' ? ' selected' : '') + '>Linear (constant velocity)</option><option value="cubic"' + (P.type === 'cubic' ? ' selected' : '') + '>Cubic polynomial</option><option value="quintic"' + (P.type === 'quintic' ? ' selected' : '') + '>Quintic polynomial</option></select></label>' +
      fld('tp-T', 'Duration T [s]', r6(P.T), 'min="0.01"') + fld('tp-dt', 'Sampling interval [s]', r6(P.dt), 'min="0.0005"') + '</div>' +
      '<div class="formula">cubic, zero end velocities: q(t) = q₀ + (q<sub>f</sub> − q₀)(3u² − 2u³), u = t/T</div></div>' +
      '<div class="sect"><div class="scroll-x"><table class="data"><tr><th>joint</th><th>start q₀</th><th>end q<sub>f</sub></th><th>v₀ [/s]</th><th>v<sub>f</sub></th><th>a₀ [/s²]</th><th>a<sub>f</sub></th></tr>' + J.map((_, i) => row(i)).join('') + '</table></div>' +
      '<div class="muted">Angles in °, prismatic in m. Velocity and acceleration boundary values are used by cubic (v) and quintic (v, a); linear ignores them. Cartesian mode uses only q₀ and the end position below, with a quintic time-scaling.</div>' +
      '<div class="row"><button class="btn small" data-act="tp-start">q₀ = current</button><button class="btn small" data-act="tp-end">q<sub>f</sub> = current</button><button class="btn small" data-act="tp-swap">Swap</button><button class="btn small" data-act="tp-gostart">Show robot at q₀</button></div>' +
      '<div id="tp-cart"' + (P.space === 'cart' ? '' : ' hidden') + '><h4>Cartesian end position [m]</h4><div class="grid3">' + [0, 1, 2].map(k => fld('tp-p' + k, 'xyz'[k], r6(P.pEnd[k]))).join('') + '</div><div class="row"><button class="btn small" data-act="tp-ptarget">Use IK target</button></div></div></div>' +
      '<div class="sect"><div class="row"><button class="btn primary" data-act="tp-gen">Generate & load trajectory</button><button class="btn" data-act="tp-play">Play / pause</button></div><div id="traj-out"></div></div>';
  };
  POST.traj = () => renderTrajOut();
  CHANGE.traj = (e) => { if (e.target.id === 'tp-space') { S.trajP.space = e.target.value; $('#tp-cart').hidden = S.trajP.space !== 'cart'; } };
  function readTrajParams() {
    const J = S.robot.joints, n = J.length, ids = [['tp-T', 0.01, 600], ['tp-dt', 0.0005, 10]]; const base = readList(ids); if (!base) return null;
    const P = { space: $('#tp-space').value, type: $('#tp-type').value, T: base[0], dt: base[1] }; let bad = false;
    for (const k of ['q0', 'qf', 'v0', 'vf', 'a0', 'af']) { P[k] = []; for (let i = 0; i < n; i++) { const v = getN('tp-' + k + '-' + i, -1e5, 1e5); if (Number.isNaN(v)) bad = true; P[k].push(fromDisp(J[i], v)); } }
    if (P.space === 'cart') { const pe = readList([['tp-p0', -100, 100], ['tp-p1', -100, 100], ['tp-p2', -100, 100]]); if (!pe) return null; P.pEnd = pe; }
    if (bad) { toast('Please correct the highlighted fields.', true); return null; } return P;
  }
  const setTrajInputs = (k, arr) => arr.forEach((v, i) => { const e = $('#tp-' + k + '-' + i); if (e) e.value = r6(toDisp(S.robot.joints[i], v)); });
  ACT['tp-start'] = () => { S.trajP.q0 = S.q.slice(); setTrajInputs('q0', S.q); };
  ACT['tp-end'] = () => { S.trajP.qf = S.q.slice(); setTrajInputs('qf', S.q); };
  ACT['tp-swap'] = () => { const P = readTrajParams(); if (!P) return; S.trajP.q0 = P.qf; S.trajP.qf = P.q0; setTrajInputs('q0', P.qf); setTrajInputs('qf', P.q0); };
  ACT['tp-gostart'] = () => { const P = readTrajParams(); if (P) { S.play.playing = false; setQ(P.q0); } };
  ACT['tp-ptarget'] = () => { [0, 1, 2].forEach(k => { $('#tp-p' + k).value = r6(S.target.pos[k]); }); };
  ACT['tp-play'] = () => togglePlay();
  ACT['tp-gen'] = () => {
    const P = readTrajParams(); if (!P) return; Object.assign(S.trajP, P); S.trajFail = []; S.trajNote = '';
    let tr;
    try {
      if (P.space === 'joint') tr = RL.jointTrajectory({ type: P.type, q0: P.q0, qf: P.qf, v0: P.v0, vf: P.vf, a0: P.a0, af: P.af, T: P.T, dt: P.dt });
      else { const c = RL.cartesianLineTrajectory(S.robot, P.q0, P.pEnd, P.T, P.dt, { lambda: S.ikOpt.lambda, tol: S.ikOpt.tol }); tr = c.traj; S.trajFail = c.failed; S.trajNote = tr.velocityNote || ''; }
    } catch (err) { toast(err.message, true); return; }
    tr.ee = tr.q.map(q => RL.fkPos(S.robot, q)); S.traj = tr; S.active = 'traj'; S.trajWarn = RL.trajectoryChecks(S.robot, tr); S.sim = null; updatePathLine(); S.plot.view = null;
    if (S.trajFail.length) { S.play.data = null; S.play.playing = false; updatePlayUI(); toast('Cartesian trajectory has ' + S.trajFail.length + ' failed IK sample(s) — playback disabled.', true); }
    else setPlayData(tr, 'trajectory', false);
    setPlotKind('q'); renderTrajOut();
  };
  function renderTrajOut() {
    const el = $('#traj-out'); if (!el) return; const tr = S.traj; if (!tr) { el.innerHTML = '<div class="muted">No trajectory yet.</div>'; return; }
    const J = S.robot.joints, n = J.length; let path = 0; for (let k = 1; k < tr.ee.length; k++) path += V.norm(V.sub(tr.ee[k], tr.ee[k - 1]));
    const pk = (A, j) => Math.max(...A.map(r => Math.abs(r[j])));
    let h = '<div class="kv"><span>Samples</span><span>' + tr.t.length + ' over ' + fx(tr.t[tr.t.length - 1], 3) + ' s</span><span>End-effector path length</span><span>' + fx(path, 4) + ' m</span></div>' +
      '<div class="scroll-x"><table class="data"><tr><th>joint</th><th>min</th><th>max</th><th>peak |q̇|</th><th>peak |q̈|</th></tr>' + J.map((j, i) => '<tr><td>q' + (i + 1) + '</td><td>' + fx(toDisp(j, Math.min(...tr.q.map(r => r[i]))), 3) + '</td><td>' + fx(toDisp(j, Math.max(...tr.q.map(r => r[i]))), 3) + '</td><td>' + fx(toDisp(j, pk(tr.qd, i)), 3) + uVel(j) + '</td><td>' + fx(toDisp(j, pk(tr.qdd, i)), 3) + uAcc(j) + '</td></tr>').join('') + '</table></div>';
    if (S.trajNote) h += '<div class="note">' + esc(S.trajNote) + '</div>';
    if (S.trajWarn.length) h += S.trajWarn.map(w => '<div class="note warn">⚠ ' + esc(w) + '</div>').join(''); else h += '<div class="note ok">All samples are within the joint limits and finite.</div>';
    if (S.trajFail.length) h += '<div class="note bad"><b>' + S.trajFail.length + ' of ' + tr.t.length + ' Cartesian samples failed.</b> The IK could not reach these points (they are outside the workspace, past joint limits, or at a singularity), so the motion is not valid and playback is disabled. First failures: ' + S.trajFail.slice(0, 5).map(f => 't=' + fx(f.t, 2) + ' s (' + f.posError.toExponential(1) + ' m)').join('; ') + '.</div>';
    el.innerHTML = h;
  }

  /* ============================== Dynamics =============================== */
  PANELS.dyn = () => {
    const J = S.robot.joints;
    return head('Robot dynamics', 'Rigid-body dynamics of the serial chain: τ = M(q) q̈ + C(q, q̇) q̇ + g(q).') +
      '<div class="sect"><div class="note">Scope: full rigid-body Lagrangian model — point mass plus diagonal inertia about each link COM (set in Robot Builder), gravity from the Builder. No joint friction, actuator dynamics, contact or payload. C(q,q̇) comes from Christoffel symbols with ∂M/∂q by central differences (≈1e-9 relative accuracy). Validated against the textbook 2-link equations (see Results → Self-tests).</div>' +
      '<div class="formula">τ = M(q) q̈ + C(q,q̇) q̇ + g(q)</div><ul style="margin:4px 0 0 16px;padding:0"><li><b>M(q)</b>: configuration-dependent inertia (symmetric, positive-definite).</li><li><b>C(q,q̇)q̇</b>: Coriolis and centrifugal forces from joint motion.</li><li><b>g(q)</b>: torque needed to hold the links against gravity.</li></ul></div>' +
      '<div class="sect"><h3>Inverse dynamics at the current pose</h3><div class="grid3">' + J.map((j, i) => fld('dq-' + i, 'q̇' + (i + 1) + ' [' + uVel(j).trim() + ']', r6(toDisp(j, S.dynP.qd[i])))).join('') + J.map((j, i) => fld('dqq-' + i, 'q̈' + (i + 1) + ' [' + uAcc(j).trim() + ']', r6(toDisp(j, S.dynP.qdd[i])))).join('') + '</div><div id="dyn-live"></div></div>' +
      '<div class="sect"><h3>Forward dynamics simulation</h3><div class="muted">Integrates q̈ = M⁻¹(τ − Cq̇ − g) with RK4 from the current pose and the q̇ above, holding constant applied torques/forces.</div><div class="grid3">' + J.map((j, i) => fld('ft-' + i, 'τ' + (i + 1) + ' [' + uTau(j) + ']', r6(S.dynP.tau[i]))).join('') + fld('fs-T', 'Duration [s]', r6(S.dynP.simT)) + fld('fs-dt', 'Step [s]', r6(S.dynP.simDt)) + '</div>' +
      '<div class="row"><button class="btn primary" data-act="dyn-sim">Simulate</button><button class="btn" data-act="dyn-inv" title="Compute τ(t) along the current trajectory and show it in the graph">Torques along trajectory</button></div><div id="dyn-sim-out"></div></div>';
  };
  CHANGE.dyn = (e) => {
    const m = e.target.id.match(/^(dq|dqq|ft)-(\d+)$/); if (!m) return; const v = parseFloat(e.target.value); if (!Number.isFinite(v)) { e.target.classList.add('invalid'); return; } e.target.classList.remove('invalid'); const i = +m[2], j = S.robot.joints[i];
    if (m[1] === 'dq') S.dynP.qd[i] = fromDisp(j, v); else if (m[1] === 'dqq') S.dynP.qdd[i] = fromDisp(j, v); else S.dynP.tau[i] = v; dirty.live = true;
  };
  LIVE.dyn = () => {
    const el = $('#dyn-live'); if (!el) return;
    try {
      const r = RL.inverseDynamics(S.robot, S.q, S.dynP.qd, S.dynP.qdd, gravityOf()), J = S.robot.joints;
      el.innerHTML = '<div class="scroll-x"><table class="data"><tr><th>joint</th><th>M q̈</th><th>C q̇</th><th>g</th><th>τ total</th></tr>' + J.map((j, i) => '<tr><td>q' + (i + 1) + ' [' + uTau(j) + ']</td><td>' + fx(r.Mqdd[i], 4) + '</td><td>' + fx(r.Cqd[i], 4) + '</td><td>' + fx(r.G[i], 4) + '</td><td><b>' + fx(r.tau[i], 4) + '</b></td></tr>').join('') + '</table></div>' +
        '<details data-k="dynM"' + dOpen('dynM') + '><summary>Mass matrix M(q)</summary>' + matHTML(r.M, 4) + '<div class="muted">Units mix kg·m² (revolute) and kg (prismatic).</div></details><details data-k="dynC"' + dOpen('dynC') + '><summary>Coriolis matrix C(q, q̇)</summary>' + matHTML(r.C, 4) + '</details>' +
        '<div class="muted">Static holding torque/force g(q) = [' + r.G.map(x => fx(x, 3)).join(', ') + ']</div>';
    } catch (err) { el.innerHTML = '<div class="note bad">Dynamics could not be evaluated: ' + esc(err.message) + '</div>'; }
  };
  ACT['dyn-sim'] = () => {
    const J = S.robot.joints, n = J.length, v = readList([['fs-T', 0.01, 20], ['fs-dt', 0.0005, 0.1]].concat(J.map((_, i) => ['ft-' + i, -1e6, 1e6]))); if (!v) return;
    S.dynP.simT = v[0]; S.dynP.simDt = v[1]; S.dynP.tau = v.slice(2, 2 + n); let sim;
    try { sim = RL.simulateForward(S.robot, S.q.slice(), S.dynP.qd.slice(), S.dynP.tau.slice(), v[0], v[1], gravityOf()); } catch (e) { toast(e.message, true); return; }
    sim.ee = sim.q.map(q => RL.fkPos(S.robot, q)); S.sim = sim; S.active = 'sim'; S.plot.view = null; setPlayData(sim, 'forward dynamics', false); setPlotKind('q');
    const free = S.dynP.tau.every(t => t === 0), E0 = sim.energy[0], drift = Math.max(...sim.energy.map(e => Math.abs(e - E0))), warns = RL.trajectoryChecks(S.robot, sim);
    $('#dyn-sim-out').innerHTML = '<div class="kv"><span>Steps</span><span>' + (sim.t.length - 1) + '</span>' + (free ? '<span>Energy drift</span><span>' + drift.toExponential(2) + ' J (torque-free ⇒ total energy should be constant)</span>' : '') + '</div>' + warns.map(w => '<div class="note warn">⚠ ' + esc(w) + ' Joint limits are not enforced by the simulation.</div>').join('');
  };
  ACT['dyn-inv'] = () => { if (!S.traj) { toast('Generate a trajectory first (Trajectory Planning).', true); return; } S.active = 'traj'; delete S.traj.tau; setPlotKind('tau'); };

  /* ============================== Workspace ============================== */
  PANELS.ws = () => head('Workspace analysis', 'Random sampling of joint space within the joint limits; each sample is mapped through forward kinematics.') +
    '<div class="sect"><div class="grid2">' + fld('ws-n', 'Samples (100 – 200 000)', S.wsP.n, 'min="100" max="200000"') + fld('ws-seed', 'Random seed', S.wsP.seed) + fld('ws-size', 'Point size [m]', S.wsP.size, 'step="0.005"') + fld('ws-op', 'Opacity (0.05 – 1)', S.wsP.opacity, 'step="0.05"') + '</div>' +
    '<div class="grid2"><label class="field"><span>Colour by</span><select id="ws-col"><option value="height"' + (S.ws.mode === 'height' ? ' selected' : '') + '>Height (z)</option><option value="radius"' + (S.ws.mode === 'radius' ? ' selected' : '') + '>Distance from base</option></select></label><label class="row" style="align-self:end"><input type="checkbox" id="ws-box"' + (S.wsP.bbox ? ' checked' : '') + '> Show bounding box</label></div>' +
    '<div class="row"><button class="btn primary" data-act="ws-run">Run sampling</button><button class="btn" data-act="ws-cancel">Cancel</button><button class="btn" data-act="ws-clear">Clear</button></div><progress id="ws-prog" max="1" value="' + S.ws.progress + '"></progress><div id="ws-out"></div></div>' +
    '<div class="sect"><div class="note">Random sampling estimates the workspace; it does not guarantee every boundary point, and sparse regions near the boundary are under-represented. Sampling runs in small time slices so the page stays responsive. Orientation reachability and self-collision are not evaluated. See also the graph “Workspace samples” projections.</div></div>';
  POST.ws = () => renderWsOut();
  function renderWsOut() {
    const el = $('#ws-out'); if (!el) return; const w = S.ws; if (!w.count && !w.running) { el.innerHTML = '<div class="muted">No samples yet.</div>'; return; }
    el.innerHTML = '<div class="kv"><span>Valid samples</span><span>' + w.count + '</span><span>Failed / invalid</span><span>' + w.failed + '</span>' + (w.min ? '<span>Bounds x</span><span>[' + fx(w.min[0], 3) + ', ' + fx(w.max[0], 3) + '] m</span><span>Bounds y</span><span>[' + fx(w.min[1], 3) + ', ' + fx(w.max[1], 3) + '] m</span><span>Bounds z</span><span>[' + fx(w.min[2], 3) + ', ' + fx(w.max[2], 3) + '] m</span>' : '') + '</div>';
  }
  CHANGE.ws = (e) => {
    const id = e.target.id; if (id === 'ws-col') { S.ws.mode = e.target.value; buildWorkspaceGL(); return; }
    if (id === 'ws-box') { S.wsP.bbox = e.target.checked; if (G.wsBox) { G.wsBox.visible = S.wsP.bbox; dirty.render = true; } return; }
    if (id === 'ws-size' || id === 'ws-op') { const s = getN('ws-size', 0.001, 1), o = getN('ws-op', 0.05, 1); if (!Number.isNaN(s)) S.wsP.size = s; if (!Number.isNaN(o)) S.wsP.opacity = o; if (G.wsPoints) { G.wsPoints.material.size = S.wsP.size; G.wsPoints.material.opacity = S.wsP.opacity; G.wsPoints.material.needsUpdate = true; dirty.render = true; } }
  };
  ACT['ws-run'] = () => {
    if (S.ws.running) return; const v = readList([['ws-n', 100, 200000], ['ws-seed', 0, 4294967295], ['ws-size', 0.001, 1], ['ws-op', 0.05, 1]]); if (!v) return;
    S.wsP.n = Math.round(v[0]); S.wsP.seed = Math.round(v[1]); S.wsP.size = v[2]; S.wsP.opacity = v[3]; const n = S.wsP.n, robot = S.robot, rng = RL.makeRng(S.wsP.seed), pts = new Float32Array(n * 3), q = new Array(robot.joints.length), out = [0, 0, 0], mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    let i = 0, cnt = 0, failed = 0; const token = {}; S.ws.token = token; S.ws.running = true; S.ws.count = 0; S.ws.failed = 0; clearWorkspaceGL();
    const step = () => {
      if (S.ws.token !== token) return; const t0 = performance.now();
      while (i < n && performance.now() - t0 < 12) { for (let k = 0; k < 400 && i < n; k++, i++) { RL.sampleConfig(robot, rng, q); RL.fkPos(robot, q, out); if (Number.isFinite(out[0] + out[1] + out[2])) { pts[3 * cnt] = out[0]; pts[3 * cnt + 1] = out[1]; pts[3 * cnt + 2] = out[2]; cnt++; for (let a = 0; a < 3; a++) { if (out[a] < mn[a]) mn[a] = out[a]; if (out[a] > mx[a]) mx[a] = out[a]; } } else failed++; } }
      S.ws.progress = i / n; const pg = $('#ws-prog'); if (pg) pg.value = S.ws.progress;
      if (i < n) { setTimeout(step, 0); return; }
      S.ws.running = false; S.ws.pts = pts; S.ws.count = cnt; S.ws.failed = failed; S.ws.min = cnt ? mn : null; S.ws.max = cnt ? mx : null; buildWorkspaceGL(); renderWsOut(); setPlotKind('wsXY'); fitView(); toast('Workspace sampling finished: ' + cnt + ' points.');
    };
    step();
  };
  ACT['ws-cancel'] = () => { S.ws.running = false; S.ws.token = null; toast('Sampling cancelled.'); renderWsOut(); };
  ACT['ws-clear'] = () => { S.ws.running = false; S.ws.token = null; S.ws.pts = null; S.ws.count = 0; S.ws.failed = 0; S.ws.min = S.ws.max = null; S.ws.progress = 0; clearWorkspaceGL(); renderPanel(); dirty.plot = true; };

  /* ============================ Results & export ========================= */
  PANELS.res = () => head('Results & export', 'Download numerical results, save the whole project, or print a calculation report.') +
    '<div class="sect"><h3>Current state</h3><div id="res-live"></div></div>' +
    '<div class="sect"><h3>Export</h3><div class="row"><button class="btn" data-act="x-plot">Current graph (CSV)</button><button class="btn" data-act="x-traj">Trajectory data (CSV)</button><button class="btn" data-act="dh-csv">DH table (CSV)</button><button class="btn" data-act="dh-mats">Transform matrices (CSV)</button></div>' +
    '<div class="row"><button class="btn" data-act="x-ws">Workspace points (CSV)</button><button class="btn" data-act="x-project">Project (JSON)</button><button class="btn" data-act="x-open">Open project…</button><button class="btn primary" data-act="x-report">Printable report (HTML)</button></div></div>' +
    '<div class="sect"><h3>Validation</h3><div class="muted">Runs the numerical self-tests inside your browser (FK vs analytic, IK round-trips, Jacobian vs finite differences, trajectories, dynamics vs textbook formulas, input validation).</div><div class="row"><button class="btn" data-act="x-test">Run self-tests</button></div><div id="test-out"></div></div>' +
    '<div class="sect"><h3>Project</h3><div class="row"><button class="btn danger" data-act="x-reset">Reset project (clear saved session)</button></div></div>';
  LIVE.res = () => {
    const el = $('#res-live'); if (!el || !S.fkCache) return; const T = S.fkCache.T, e = RL.rToRPY(RL.Rof(T)), p = RL.Pof(T);
    el.innerHTML = '<div class="kv"><span>Robot</span><span>' + esc(S.robot.name) + ' (' + S.robot.joints.length + ' DOF)</span><span>Joint values</span><span>[' + qStr(S.q) + ']</span><span>EE position</span><span>' + vecStr(p) + ' m</span><span>EE roll/pitch/yaw</span><span>' + vecStr([e.roll * R2D, e.pitch * R2D, e.yaw * R2D], 2) + '°</span>' +
      '<span>IK</span><span>' + (S.ik.result ? (S.ik.result.converged ? 'converged' : 'NOT converged') + ', residual ' + S.ik.result.posError.toExponential(2) + ' m' : 'not run') + '</span><span>Trajectory</span><span>' + (S.traj ? S.traj.t.length + ' samples' + (S.trajFail.length ? ', ' + S.trajFail.length + ' failed' : '') : 'none') + '</span><span>Workspace</span><span>' + (S.ws.count ? S.ws.count + ' samples' : 'none') + '</span></div>';
  };
  ACT['x-plot'] = exportPlotCSV;
  ACT['x-traj'] = () => {
    const d = activeSeries(); if (!d) { toast('No trajectory or simulation to export.', true); return; } const J = S.robot.joints, tau = ensureTau(d), ee = ensureEE(d);
    const hdr = ['t_s'].concat(J.map((j, i) => 'q' + (i + 1) + '_' + (j.type === 'R' ? 'deg' : 'm')), J.map((j, i) => 'qd' + (i + 1) + '_' + (j.type === 'R' ? 'deg_s' : 'm_s')), J.map((j, i) => 'qdd' + (i + 1) + '_' + (j.type === 'R' ? 'deg_s2' : 'm_s2')), tau ? J.map((j, i) => 'tau' + (i + 1) + '_' + (j.type === 'R' ? 'Nm' : 'N')) : [], ['ee_x_m', 'ee_y_m', 'ee_z_m']), rows = [hdr];
    d.t.forEach((t, k) => rows.push([t].concat(d.q[k].map((v, i) => toDisp(J[i], v)), d.qd[k].map((v, i) => toDisp(J[i], v)), d.qdd[k].map((v, i) => toDisp(J[i], v)), tau ? tau[k] : [], ee[k]))); download('robolab-trajectory.csv', toCSV(rows), 'text/csv');
  };
  ACT['x-ws'] = () => { if (!S.ws.count) { toast('Run the workspace sampling first.', true); return; } const rows = [['x_m', 'y_m', 'z_m']]; for (let i = 0; i < S.ws.count; i++) rows.push([S.ws.pts[3 * i], S.ws.pts[3 * i + 1], S.ws.pts[3 * i + 2]]); download('robolab-workspace.csv', toCSV(rows), 'text/csv'); };
  ACT['x-project'] = saveProject; ACT['x-open'] = () => $('#file-open').click();
  ACT['x-reset'] = () => { try { localStorage.removeItem(STORE_KEY); } catch (e) { /* ignore */ } loadPreset('planar2'); S.view = Object.assign(S.view, { ortho: false, wire: false, grid: true, frames: true, axes: true, labels: false, jointAxes: true, trail: true, light: 1 }); syncViewButtons(); setProjection(false); setMode('builder'); toast('Project reset.'); };
  ACT['x-test'] = () => {
    const out = $('#test-out'); out.innerHTML = '<div class="muted">Running…</div>';
    setTimeout(() => { let res; try { res = RL.selfTests(); } catch (e) { out.innerHTML = '<div class="note bad">Self-test crashed: ' + esc(e.message) + '</div>'; return; }
      const pass = res.filter(r => r.pass).length; out.innerHTML = '<div class="note ' + (pass === res.length ? 'ok' : 'bad') + '"><b>' + pass + ' / ' + res.length + ' tests passed</b></div>' + res.map(r => '<details data-k="t-' + esc(r.name) + '"><summary><span class="' + (r.pass ? 'ok' : 'bad') + '">' + (r.pass ? '✔' : '✘') + '</span> ' + esc(r.name) + '</summary><div class="muted mono">' + esc(r.detail) + '</div></details>').join(''); }, 30);
  };
