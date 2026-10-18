
<p align="center">
  <h1 align="center">🤖 RoboLab Web</h1>
  <p align="center"><strong>Interactive Robotics Analysis Studio</strong></p>
  <p align="center">
    Explore robot kinematics, dynamics, trajectory planning, and 3D workspaces directly in your browser.
  </p>
  <p align="center">
    <a href="https://batharva.github.io/robolab-web/">
      <strong>🚀 Launch Live Demo</strong>
    </a>
    ·
    <a href="https://github.com/batharva/robolab-web">
      <strong>View Source</strong>
    </a>
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/Platform-Web%20Browser-2ea44f?style=flat-square" alt="Web Browser">
    <img src="https://img.shields.io/badge/JavaScript-Client--Side-f1e05a?style=flat-square" alt="JavaScript">
    <img src="https://img.shields.io/badge/3D-Three.js-111111?style=flat-square" alt="Three.js">
    <img src="https://img.shields.io/badge/Hosting-GitHub%20Pages-222222?style=flat-square" alt="GitHub Pages">
  </p>
</p>

---

## Overview

**RoboLab Web** is a browser-based robotics learning and analysis environment that connects robotics mathematics with interactive calculations, graphs, and 3D visualization.

Build robot manipulators, edit Denavit–Hartenberg (DH) parameters, solve forward and inverse kinematics, analyze Jacobians, simulate dynamics, plan trajectories, and explore robot workspaces.

- **No installation required** for the live demo.
- **Client-side application:** no backend, account, or API key is required for normal use.
- **Static deployment:** can be hosted using GitHub Pages.
- **Educational focus:** designed for experimentation and learning, not safety-critical robot control.

## ✨ Features

### 🦾 1. Robot Builder

- Create manipulators with 1–12 joints.
- Configure revolute and prismatic joints.
- Edit DH parameters, joint limits, mass, center of mass, and inertia.
- Add, remove, duplicate, rename, and modify joints.
- Explore four example robot configurations.
- Save and restore configurations using browser storage.

### 📐 2. Denavit–Hartenberg Parameters

- Interactive classical DH parameter table.
- Calculate individual link transformation matrices.
- Compute cumulative homogeneous transformations.
- Visualize robot coordinate frames.
- Export DH parameters to CSV.

Classical DH transformation:

\[
{}^{i-1}T_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)
\]

### 🎯 3. Forward Kinematics

- Calculate end-effector position and orientation.
- Display the complete 4 × 4 homogeneous transformation matrix.
- Inspect the 3 × 3 rotation matrix.
- Calculate ZYX Euler angles.
- Check rotation-matrix orthonormality.
- Review calculation steps and compare results with analytical reference solutions.

### 🔄 4. Inverse Kinematics

- Calculate joint configurations for a desired end-effector target.
- Solve position-only and full-pose inverse kinematics.
- Damped least-squares numerical solver with adaptive damping.
- Joint-limit handling and multi-start solving.
- Analytical two-link planar IK with elbow-up and elbow-down solutions.
- Animate the robot toward a solution.
- Set IK targets interactively in the 3D viewport.
- Report convergence failures and unreachable targets.

### 🔢 5. Jacobian Analysis

- Calculate the geometric Jacobian in the base frame.
- Analyze singular values and condition number.
- Evaluate manipulability and detect singular configurations.
- Validate Jacobian calculations using finite differences.
- Map joint velocities to end-effector velocities.
- Solve inverse-velocity kinematics using damping.

### 📈 6. Trajectory Planning

- Cubic and quintic joint-space trajectories.
- Configurable position, velocity, and acceleration boundary conditions.
- Cartesian straight-line trajectories using inverse kinematics.
- Joint-limit warnings and failed-sample reporting.
- Playback controls, scrubbing, looping, and adjustable speed.
- End-effector path visualization and motion trails.
- Position, velocity, and acceleration plots.

### ⚙️ 7. Robot Dynamics

Analyze the manipulator equation of motion:

\[
\tau=M(q)\ddot q+C(q,\dot q)\dot q+g(q)
\]

Capabilities include:

- Joint-space mass matrix \(M(q)\).
- Coriolis and centrifugal terms.
- Gravity torque calculations.
- Individual torque-term breakdown.
- Forward-dynamics simulation using fourth-order Runge–Kutta (RK4).
- Torque plots along trajectories.
- Configurable link masses, centers of mass, and inertia properties.

### 🌐 8. Workspace Analysis

- Random sampling of robot configurations.
- Interactive 3D workspace point cloud.
- Workspace bounds and bounding boxes.
- XY, XZ, and YZ projections.
- Configurable visualization color modes.
- Cancellable workspace sampling.

### 📊 9. Interactive Graphs

Visualize and analyze:

- Joint position, velocity, acceleration, and torque.
- End-effector X, Y, and Z coordinates.
- End-effector path projections.
- Inverse-kinematics error history.
- Workspace projections.

Graph controls include zoom, pan, hover inspection, legend toggling, and CSV export.

### 📚 10. Learning Tools

- Guided application tour.
- Five worked robotics examples.
- Forward- and inverse-kinematics exercises.
- Robotics concepts and DH diagrams.
- Interactive glossary.
- In-browser mathematical self-tests.

### 💾 11. Projects and Export

- Save and open projects as JSON.
- Validate imported project data.
- Automatically restore the previous session.
- Export graph and calculation data as CSV.
- Generate printable HTML reports.
- Store configurations in browser storage.

---

## 🛠️ Technology Stack

| Technology | Purpose |
|---|---|
| HTML5 | Application structure |
| CSS3 | Interface design and styling |
| JavaScript | Robotics calculations and application logic |
| Three.js | Interactive 3D visualization |
| Python | Development-time source generation and testing |
| Node.js | Automated JavaScript tests |
| NumPy | Independent numerical verification |
| GitHub Pages | Static website hosting |

**Architecture:** Client-side web application. No backend, account, API key, or build step is required for normal deployment.

Three.js is loaded from a CDN, so an internet connection is required to load the 3D visualization library.

---

## 🧮 Robotics Mathematics

| Concept | Application |
|---|---|
| Homogeneous transformations | Link poses and end-effector position |
| Rotation matrices | Three-dimensional orientation |
| Forward kinematics | Joint coordinates to end-effector pose |
| Inverse kinematics | Desired pose to joint coordinates |
| Geometric Jacobian | Joint and end-effector velocity relationships |
| Singular-value analysis | Singularity and manipulability assessment |
| Cubic and quintic polynomials | Smooth trajectory generation |
| Mass matrix and gravity vector | Robot dynamics |
| Christoffel symbols | Coriolis and centrifugal terms |
| RK4 integration | Forward-dynamics simulation |

### Coordinate and Unit Conventions

- Internal length units: metres.
- Internal angular units: radians.
- DH table and UI angles: degrees.
- World coordinate system: Z-up.
- DH convention: classical DH; modified DH is not implemented.
- Euler angle convention: ZYX, with

\[
R=R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})
\]

---

## 🧪 Testing and Validation

The project includes automated mathematical tests, worked examples, and independent numerical verification.

Validation covers:

- DH transformation identities.
- Rotation-matrix orthonormality and determinant.
- Forward kinematics against analytical solutions.
- Analytical and numerical inverse kinematics.
- Jacobian calculations against finite differences.
- Cubic and quintic trajectory boundary conditions.
- Mass matrix, Coriolis terms, and gravity torques.
- Forward- and inverse-dynamics consistency.
- Energy conservation in torque-free simulation.
- Invalid project-data rejection and project round trips.
- Application modes and robot-scene transformations.

### Run Tests

**JavaScript self-tests**

```bash
node tests/run_selftests.js
```

**Independent NumPy verification**

```bash
python3 tests/reference_check.py
```

**Browser tests**

Open RoboLab Web and select:

`Results & Export → Run self-tests`

See [`TESTING.md`](TESTING.md) for detailed results and known testing limitations.

---

## 🚀 Getting Started

### Option 1: Use the Live Application

Open RoboLab Web directly in your browser:

**[🚀 Launch RoboLab Web](https://batharva.github.io/robolab-web/)**

No installation is required.

### Option 2: Run Locally

Clone the repository:

```bash
git clone https://github.com/batharva/robolab-web.git
cd robolab-web
```

Open `index.html` in a modern web browser.

An internet connection is required to load Three.js from the CDN.

### Option 3: Deploy on GitHub Pages

1. Push `index.html`, `README.md`, and all required project files to GitHub.
2. Navigate to **Settings → Pages**.
3. Under **Build and deployment**, choose **Deploy from a branch**.
4. Select the `main` branch and `/ (root)` folder.
5. Click **Save**.
6. Wait for deployment and open your GitHub Pages URL.

---

## 📁 Project Structure

```text
robolab-web/
├── index.html
├── README.md
├── TESTING.md
├── build.py
├── style.css
├── body.html
├── src/
│   ├── core.js
│   └── app_*.js
└── tests/
    ├── run_selftests.js
    └── reference_check.py
```

This is the documented project layout. Confirm the filenames against your current repository before publishing.

---

## 👨‍💻 Authors

### Atharva Bhosale
Mechanical Engineering Student | Robotics | Simulation | Software Development

- **GitHub:** [@batharva](https://github.com/batharva)

### Kanishk Patel

- **GitHub:** [@KanishkPatel0206](https://github.com/KanishkPatel0206)
---

## 📄 License

Add a `LICENSE` file to the repository to specify the terms under which others can use, modify, and distribute this project.

---
