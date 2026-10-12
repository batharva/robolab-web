// Emits engine outputs at fixed configurations as JSON for tests/reference_check.py
const RL = require('../src/core.js');
const out = { cases: [] };
const q6 = [0.3, -1.1, 1.4, -0.7, 0.9, 0.2], q2 = [0.4, 1.1], qd2 = [0.7, -0.5];
for (const [id, q] of [['ur5like', q6], ['arm3', [0.2, 0.6, -0.9]], ['planar2', q2], ['scara4', [0.5, -0.4, 0.12, 0.3]]]) {
  const r = RL.buildPreset(id), F = RL.fk(r, q), J = RL.jacobian(r, q);
  out.cases.push({ id, joints: r.joints.map(j => ({ type: j.type, a: j.a, alpha: j.alpha, d: j.d, theta: j.theta })), q, T: F.T, J });
}
const p = RL.buildPreset('planar2'), D = RL.inverseDynamics(p, q2, qd2, [0.3, -0.2], [0, -9.81, 0]);
out.dyn = { m: p.joints.map(j => j.mass), l: p.joints.map(j => j.a), lc: p.joints.map(j => -j.com[0]), I: p.joints.map(j => j.inertia[2]), q: q2, qd: qd2, qdd: [0.3, -0.2], tau: D.tau, M: D.M, G: D.G };
console.log(JSON.stringify(out));
