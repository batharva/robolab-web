```{=html}
<p align="center">
```
```{=html}
<h1 align="center">
```
🤖 RoboLab Web
```{=html}
</h1>
```
```{=html}
<p align="center">
```
`<strong>`{=html}Interactive Robotics Analysis Studio`</strong>`{=html}
```{=html}
</p>
```
```{=html}
<p align="center">
```
Explore robot kinematics, dynamics, trajectory planning, and 3D
workspaces directly in your browser.
```{=html}
</p>
```
```{=html}
<p align="center">
```
`<a href="https://batharva.github.io/robolab-web/">`{=html}`<strong>`{=html}🚀
Launch Live Demo`</strong>`{=html}`</a>`{=html} ·
`<a href="https://github.com/batharva/robolab-web">`{=html}`<strong>`{=html}View
Source`</strong>`{=html}`</a>`{=html}
```{=html}
</p>
```
```{=html}
<p align="center">
```
`<img src="https://img.shields.io/badge/Platform-Web%20Browser-2ea44f?style=flat-square" alt="Platform: Web Browser">`{=html}
`<img src="https://img.shields.io/badge/JavaScript-Client--Side-f1e05a?style=flat-square" alt="JavaScript">`{=html}
`<img src="https://img.shields.io/badge/3D-Three.js-111111?style=flat-square" alt="Three.js">`{=html}
`<img src="https://img.shields.io/badge/Hosting-GitHub%20Pages-222222?style=flat-square" alt="GitHub Pages">`{=html}
```{=html}
</p>
```
```{=html}
</p>
```

------------------------------------------------------------------------

## Overview

**RoboLab Web** is a browser-based robotics learning and analysis
environment that connects robotics mathematics with interactive
calculations, graphs, and 3D visualization.

Use it to build robot manipulators, edit Denavit--Hartenberg (DH)
parameters, solve forward and inverse kinematics, inspect Jacobians,
simulate dynamics, plan trajectories, and explore robot workspaces.

-   **No installation required** for the live demo.
-   **Client-side application:** no backend, account, or API key is
    required for normal use.
-   **Static deployment:** can be hosted using GitHub Pages.
-   **Educational focus:** designed for experimentation and learning,
    not safety-critical robot control.

## Features

### 🦾 Robot Builder

-   Create manipulators with 1--12 joints.
-   Configure revolute and prismatic joints.
-   Edit DH parameters, joint limits, mass, center of mass, and inertia.
-   Add, remove, duplicate, rename, and modify joints.
-   Explore four example robot configurations.
-   Save and restore configurations using browser storage.

### 📐 Denavit--Hartenberg (DH) Parameters

-   Edit a classical DH parameter table.
-   Calculate individual link transformation matrices and cumulative
    homogeneous transformations.
-   Visualize robot coordinate frames.
-   Export DH parameters to CSV.

The classical DH transformation used is:

\[ {}\^{i-1}T_i =
R_z(`\theta`{=tex}\_i)T_z(d_i)T_x(a_i)R_x(`\alpha`{=tex}\_i) \]

### 🎯 Forward Kinematics

-   Calculate end-effector position and orientation.
-   Inspect the 4 × 4 homogeneous transformation matrix and 3 × 3
    rotation matrix.
-   Calculate ZYX Euler angles.
-   Check rotation-matrix orthonormality.
-   Review calculation steps and compare results with analytical
    reference solutions.

### 🔄 Inverse Kinematics

-   Solve for joint configurations that reach a desired end-effector
    target.
-   Solve position-only and full-pose inverse-kinematics problems.
-   Use a damped least-squares numerical solver with adaptive damping.
-   Handle joint limits and use multi-start solving.
-   Solve two-link planar IK analytically with elbow-up and elbow-down
    solutions.
-   Animate solutions and set targets in the 3D viewport.
-   Report convergence failures and unreachable targets.

### 🔢 Jacobian Analysis

-   Calculate the geometric Jacobian in the base frame.
-   Inspect singular values and condition number.
-   Evaluate manipulability and identify singular configurations.
-   Validate Jacobian calculations using finite differences.
-   Map joint velocities to end-effector velocities.
-   Solve inverse-velocity kinematics using damping.

### 📈 Trajectory Planning

-   Generate cubic and quintic joint-space trajectories.
-   Configure position, velocity, and acceleration boundary conditions.
-   Generate Cartesian straight-line trajectories using inverse
    kinematics.
-   View joint-limit warnings and failed-sample reports.
-   Play, scrub, loop, and adjust playback speed.
-   Visualize end-effector paths and motion trails.
-   Plot position, velocity, and acceleration.

### ⚙️ Robot Dynamics

Analyze the manipulator equation of motion:

\[ `\tau `{=tex}= M(q)`\ddot `{=tex}q +
C(q,`\dot `{=tex}q)`\dot `{=tex}q + g(q) \]

Capabilities include: - Joint-space mass matrix (M(q)). - Coriolis and
centrifugal terms. - Gravity torque calculations and torque-term
breakdowns. - Forward-dynamics simulation using fourth-order
Runge--Kutta (RK4). - Torque plots along trajectories. - Configurable
link masses, centers of mass, and inertia properties.

### 🌐 Workspace Analysis

-   Sample random robot configurations.
-   Visualize the workspace as an interactive 3D point cloud.
-   Inspect workspace bounds and bounding boxes.
-   View XY, XZ, and YZ projections.
-   Choose visualization color modes.
-   Cancel workspace sampling.

### 📊 Interactive Graphs

Visualize joint position, velocity, acceleration, and torque;
end-effector coordinates; path projections; inverse-kinematics error
history; and workspace projections.

Graph controls include zoom, pan, hover inspection, legend toggling, and
CSV export.

### 📚 Learning Tools

-   Guided application tour.
-   Five worked robotics examples.
-   Forward- and inverse-kinematics exercises.
-   Robotics concepts and DH diagrams.
-   Interactive glossary.
-   In-browser mathematical self-tests.

### 💾 Projects and Export

-   Save and open projects as JSON.
-   Validate imported project data.
-   Restore the previous session automatically.
-   Export graph and calculation data as CSV.
-   Generate printable HTML reports.
-   Store configurations in browser storage.

## Getting Started

### Option 1: Use the live application

Open **[RoboLab Web](https://batharva.github.io/robolab-web/)** in a
modern browser. No installation is required.

An internet connection is needed to load the Three.js library from its
CDN.

### Option 2: Run locally

Clone the repository:

``` bash
git clone https://github.com/batharva/robolab-web.git
cd robolab-web
```

Open `index.html` in a modern web browser.

### Option 3: Publish with GitHub Pages

1.  Push `index.html` and all required project files to the repository.
2.  Open the repository's **Settings → Pages**.
3.  Under **Build and deployment**, select **Deploy from a branch**.
4.  Select the `main` branch and the `/ (root)` folder.
5.  Click **Save** and wait for deployment.
6.  Open the URL shown in the Pages settings.

If the site is already deployed, updating the files on the configured
branch will trigger a new deployment.

## Testing and Validation

The project includes automated mathematical tests, worked examples, and
independent numerical verification.

Validation covers: - DH transformation identities. - Rotation-matrix
orthonormality and determinant. - Forward kinematics against analytical
solutions. - Analytical and numerical inverse kinematics. - Jacobian
calculations against finite differences. - Cubic and quintic trajectory
boundary conditions. - Mass matrix, Coriolis terms, and gravity
torques. - Forward- and inverse-dynamics consistency. - Energy
conservation in torque-free simulation. - Invalid project-data rejection
and project round trips. - Application modes and robot-scene
transformations.

Run the JavaScript self-tests with Node.js:

``` bash
node tests/run_selftests.js
```

Run the independent NumPy checks:

``` bash
python3 tests/reference_check.py
```

You can also open the application and select **Results & Export → Run
self-tests**.

See [`TESTING.md`](TESTING.md) for detailed results and known testing
limitations. Test commands assume the corresponding files and
dependencies are present in your checkout.

## Technology Stack

  Technology     Purpose
  -------------- ------------------------------------------------
  HTML5          Application structure
  CSS3           Interface design and styling
  JavaScript     Robotics calculations and application logic
  Three.js       Interactive 3D visualization
  Python         Development-time source generation and testing
  Node.js        Automated JavaScript tests
  NumPy          Independent numerical verification
  GitHub Pages   Static website hosting

## Robotics Mathematics

  Concept                          Application
  -------------------------------- -----------------------------------------------
  Homogeneous transformations      Link poses and end-effector position
  Rotation matrices                Three-dimensional orientation
  Forward kinematics               Joint coordinates to end-effector pose
  Inverse kinematics               Desired pose to joint coordinates
  Geometric Jacobian               Joint and end-effector velocity relationships
  Singular-value analysis          Singularity and manipulability assessment
  Cubic and quintic polynomials    Smooth trajectory generation
  Mass matrix and gravity vector   Robot dynamics
  Christoffel symbols              Coriolis and centrifugal terms
  RK4 integration                  Forward-dynamics simulation

### Coordinate and unit conventions

-   Internal length units: metres.
-   Internal angular units: radians.
-   DH table and UI angles: degrees.
-   World coordinate system: Z-up.
-   DH convention: classical DH; modified DH is not implemented.
-   Euler angle convention: ZYX, with (R =
    R_z(`\text{yaw}`{=tex})R_y(`\text{pitch}`{=tex})R_x(`\text{roll}`{=tex})).

## Project Structure

``` text
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

This is the documented project layout; filenames can vary if the
repository has changed.

## Limitations

-   Only classical DH parameters are implemented; modified DH is not
    supported.
-   Example robot models are simplified educational representations.
-   Default mass and inertia values are estimates.
-   Dynamics do not include friction, actuator dynamics, contact forces,
    or joint-limit enforcement during simulation.
-   Numerical inverse kinematics may fail near singularities, joint
    limits, or local minima.
-   Cartesian trajectory velocity and acceleration are estimated using
    finite differences.
-   Workspace sampling estimates reachability; it does not prove the
    exact boundary.
-   Workspace analysis does not evaluate self-collision or orientation
    reachability.
-   Automated tests do not replace visual testing of WebGL rendering,
    touch interactions, mobile layouts, or accessibility.

RoboLab Web is intended for education, numerical experimentation, and
robotics analysis---not safety-critical control of physical robots.

## Future Improvements

Potential enhancements include: - Modified DH convention support. -
Additional industrial robot models. - URDF import and export. -
Collision detection and avoidance. - Advanced trajectory optimization. -
Friction and actuator dynamics. - Additional inverse-kinematics
algorithms. - Improved mobile visualization and accessibility.

These are potential future enhancements, not claims about currently
implemented features.

## Author

**Atharva Bhosale**\
Mechanical Engineering Student \| Robotics \| Simulation \| Software
Development

-   GitHub: [@batharva](https://github.com/batharva)
-   Repository:
    [batharva/robolab-web](https://github.com/batharva/robolab-web)
-   Live demo: [RoboLab Web](https://batharva.github.io/robolab-web/)

## License

No specific license is asserted here. If you want others to reuse,
modify, and distribute the project, add a `LICENSE` file that states the
terms you choose.

------------------------------------------------------------------------

```{=html}
<p align="center">
```
`<strong>`{=html}Explore robotics. Understand the mathematics. Visualize
the motion.`</strong>`{=html}`<br>`{=html}`<br>`{=html}
`<a href="https://batharva.github.io/robolab-web/">`{=html}🚀 Launch
RoboLab Web`</a>`{=html}
```{=html}
</p>
```
