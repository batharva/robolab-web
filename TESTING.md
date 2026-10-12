# Testing

## How to run
* **In the browser:** *Results & Export → Run self-tests* (28 checks, same code as below).
* **Node:** `node tests/run_selftests.js`
* **Independent cross-check (NumPy):** `python3 tests/reference_check.py` — recomputes FK, Jacobians (finite differences) and 2-link inverse dynamics with NumPy and compares with the JS engine.

## Automated checks and tolerances
| # | Check | Tolerance / criterion | Result |
|---|---|---|---|
| 1 | DH identity (a=α=d=θ=0 → I) | exact | pass |
| 2 | Rz(90°)·Tx(1) gives p = (0,1,0) | 1e-12 | pass |
| 3 | Rotation matrices orthonormal, det = 1 (200 random 6-DOF poses) | 1e-12 | max 7.8e-16 |
| 4 | FK = analytic 2-link formulas, position and yaw (300 configs) | 1e-12 | max 5e-16 |
| 5 | Fast position-only FK = full matrix FK | 1e-12 | 0 |
| 6 | Analytic 2R IK → FK round trip (400 solutions); unreachable reported | 1e-9 | max 1.6e-15 |
| 7 | Numerical DLS IK, 6-DOF position: ≥18/20 converge and verify via FK | 1e-4 m | 20/20 |
| 8 | DLS IK with orientation (pose): ≥10/12 | 1e-4 m, 1e-3 rad | 12/12 |
| 9 | Unreachable target (5 m) reported as *not converged* + "unreachable" | must not claim success | pass |
| 10 | IK clamps to joint limits | within limits | pass |
| 11 | Geometric Jacobian vs central finite differences (6-DOF; SCARA with prismatic joint) | 1e-6 | ~2e-10 |
| 12 | Planar Jacobian at q=(0,90°) = [[−0.8,−0.8],[1,0]]; singular at q₂=0; |det J| = L₁L₂\|sin q₂\| | 1e-9 | pass |
| 13 | Cubic: endpoints, zero end velocity, q(T/2) formula | 1e-12 | pass |
| 14 | Quintic: all six boundary conditions (q, q̇, q̈ at both ends) | 1e-10 | 1.8e-15 |
| 15 | q̇, q̈ equal numerical derivatives of q | 1e-5 / 1e-4 | 2.5e-6 / 1.5e-5 |
| 16 | Trajectory limit violation is detected | warning present | pass |
| 17 | Cartesian trajectory to unreachable point reports failed samples | failures listed | pass |
| 18 | M, C q̇, g match textbook 2-link equations (40 random states) | 1e-9 / 1e-6 / 1e-9 | 1e-15 / 5e-10 / 4e-15 |
| 19 | M symmetric positive-definite (6-DOF) | 1e-12 | pass |
| 20 | Forward ∘ inverse dynamics round trip (SCARA, prismatic) | 1e-9 | 3e-15 |
| 21 | Energy conserved in torque-free RK4 simulation (1 s, dt 2 ms) | 1e-6 relative | 2.3e-8 J |
| 22 | Invalid robot/project JSON rejected; valid project round-trips; T=0 trajectory raises an error | — | pass |

NumPy cross-check: FK ≤ 2.8e-16, Jacobian ≤ 1e-10, 2-link torque ≤ 2.9e-11.

## Worked examples (independently checkable)
Tutorials → *Worked examples* compare hand-calculated numbers with the engine:
1. 2-link FK, L=1,1, q=(30°,45°): x=1.12484, y=1.46593.
2. 2-link IK to (1.5,1.0): q₂=±51.318°, q₁=8.031° / 59.349°.
3. Jacobian at q=(0,90°): [[−1,−1],[1,0]], det = 1.
4. Cubic 0→90° in 2 s: q(1 s)=45°, peak q̇=1.1781 rad/s, peak |q̈|=2.3562 rad/s².
5. Static gravity torque, L=1,1, m=1,1, q=0: τ₁=19.62 N·m, τ₂=4.905 N·m.

## UI-level smoke test (headless)
The full page was loaded in jsdom with the real Three.js and a stubbed WebGL renderer: all 10 modes rendered for all 4 robots with no runtime errors; IK (analytic, numeric, multi-start, unreachable), trajectories (joint and failing Cartesian), forward-dynamics simulation, workspace sampling, builder edits (add/duplicate/remove, type change, invalid input, markup injection in names), all five worked examples and the in-page self-tests ran cleanly; the scene-graph end-effector pose equalled FK to machine precision for random configurations.

## Known limitations of the testing
* Real WebGL rendering, touch gestures, and layout on phones/tablets were **not** exercised automatically — check visually in Chrome/Firefox/Safari and on a phone.
* Numerical IK success rates depend on the start pose and the robot; failures are reported honestly, not hidden.
* Accuracy: FK/Jacobian are exact to rounding; C(q,q̇) uses central differences (≈1e-9 relative); the DLS solver stops at 1e-4 m by default.
* Mass properties of presets are estimates; dynamics results are only as good as the supplied masses and inertias.
* Accessibility: keyboard focus, labels and reduced-motion are supported, but no screen-reader audit has been done and the WebGL canvas itself is not described beyond its label.
