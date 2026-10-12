
# 🤖 RoboLab Web — Interactive Robotics Analysis Studio

### A browser-based platform for robot modelling, kinematics, dynamics, trajectory planning, and simulation.

<p align="center">
  <a href="https://batharva.github.io/robolab-web/">
    <img src="https://img.shields.io/badge/🚀_Live_Demo-Launch_RoboLab-blue?style=for-the-badge" alt="Live Demo">
  </a>
  <img src="https://img.shields.io/badge/Platform-Web%20Browser-green?style=for-the-badge" alt="Platform">
  <img src="https://img.shields.io/badge/JavaScript-Client--Side-yellow?style=for-the-badge" alt="JavaScript">
  <img src="https://img.shields.io/badge/3D-Three.js-black?style=for-the-badge" alt="Three.js">
</p>

<p align="center">
  <b><a href="https://batharva.github.io/robolab-web/">🌐 Open Live Application</a></b>
</p>

RoboLab Web is an interactive robotics analysis and educational simulation tool that brings core robotics mathematics to your browser.

Build robot manipulators, explore Denavit–Hartenberg (DH) parameters, calculate forward and inverse kinematics, analyze Jacobians, simulate robot dynamics, plan trajectories, and visualize robot workspaces using interactive 3D graphics and graphs.

The application runs entirely on the client side and can be deployed as a static website using GitHub Pages.

---

## ✨ Features

### 🦾 1. Robot Builder
- Create robot manipulators with 1–12 joints.
- Configure revolute and prismatic joints.
- Edit DH parameters, joint limits, mass, center of mass, and inertia.
- Add, remove, duplicate, rename, and modify joints.
- Explore four example robot configurations.
- Save and restore robot configurations using browser storage.

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

### 🎯 3. Forward Kinematics (FK)
- Calculate end-effector position and orientation.
- Display the complete 4 × 4 homogeneous transformation matrix.
- Inspect the 3 × 3 rotation matrix.
- Calculate ZYX Euler angles.
- Check rotation-matrix orthonormality.
- View calculation steps and compare results with analytical reference solutions.

### 🔄 4. Inverse Kinematics (IK)
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

RoboLab Web implements core mathematical techniques used in robot manipulation and analysis.

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
- Euler angle convention: ZYX, with \(R=R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})\).

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

The accompanying testing documentation reports successful results for its listed automated checks and independent NumPy comparisons.

### Run Tests

**In the browser**

Open RoboLab Web and select `Results & Export → Run self-tests`.

**Using Node.js**

```bash
node tests/run_selftests.js
```

**Independent NumPy verification**

```bash
python3 tests/reference_check.py
```

See [`TESTING.md`](TESTING.md) for detailed results and known testing limitations.

---

## 🚀 Getting Started

### Option 1: Use the Live Application

Open RoboLab Web directly in your browser:

**[Launch RoboLab Web](https://batharva.github.io/robolab-web/)**

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

1. Push `index.html`, `README.md`, and the required project files to GitHub.
2. Navigate to **Settings → Pages**.
3. Under **Build and deployment**, choose **Deploy from a branch**.
4. Select the `main` branch and `/ (root)` folder.
5. Click **Save**.
6. Wait for deployment and open your GitHub Pages URL.

---

## 📁 Project Structure

```text
robolab-web/
├── index.html              # Deployable browser application
├── README.md               # Project documentation
├── TESTING.md              # Testing and validation details
├── build.py                # Development-time source builder
├── style.css               # Application styling
├── body.html               # Application interface markup
├── src/
│   ├── core.js             # Robotics mathematics engine
│   └── app_*.js            # Application modules
└── tests/
    ├── run_selftests.js    # Automated JavaScript tests
    └── reference_check.py  # Independent NumPy checks
```

---

## ⚠️ Limitations

- Only classical DH parameters are implemented; modified DH is not supported.
- Example robot models are simplified educational representations.
- Default mass and inertia values are estimates.
- Dynamics do not include friction, actuator dynamics, contact forces, or joint-limit enforcement during simulation.
- Numerical inverse kinematics may fail near singularities, joint limits, or local minima.
- Cartesian trajectory velocity and acceleration are estimated using finite differences.
- Workspace sampling estimates reachability; it does not prove the exact boundary.
- Workspace analysis does not evaluate self-collision or orientation reachability.
- Automated tests do not replace visual testing of WebGL rendering, touch interactions, mobile layouts, or accessibility.

RoboLab Web is designed for education, numerical experimentation, and robotics analysis rather than safety-critical control of physical robots.

---

## 🎯 Project Goals

- Make robotics mathematics easier to understand through interactive visualization.
- Provide browser-based tools for studying robot kinematics and dynamics.
- Connect mathematical equations with numerical results and 3D motion.
- Help students experiment with robot configurations without installing specialized desktop software.
- Provide transparent calculation results and meaningful failure reporting.

---

## 🔮 Future Improvements

Potential future enhancements include:

- Modified DH convention support.
- Additional industrial robot models.
- URDF import and export.
- Collision detection and avoidance.
- Advanced trajectory optimization.
- Friction and actuator dynamics.
- Additional inverse-kinematics algorithms.
- Improved mobile visualization and accessibility.

These are potential enhancements, not claims about currently implemented features.

---

## 👨‍💻 Author

**Atharva Bhosale**

Mechanical Engineering Student | Robotics | Simulation | Software Development

- **GitHub:** [@batharva](https://github.com/batharva)
- **Project Repository:** [robolab-web](https://github.com/batharva/robolab-web)
- **Live Demo:** [RoboLab Web](https://batharva.github.io/robolab-web/)

---

## 📄 License

Add a `LICENSE` file to the repository to specify the terms under which others can use, modify, and distribute this project.

---

<p align="center">
  <b>Explore robotics. Understand the mathematics. Visualize the motion.</b>
  <br><br>
  <a href="https://batharva.github.io/robolab-web/">🚀 Launch RoboLab Web</a>
</p>
