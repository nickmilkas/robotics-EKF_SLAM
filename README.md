# EKF‑SLAM (Unknown Correspondence) for a Differential‑Drive Robot (Python + Pygame)

This project implements a **2D Extended Kalman Filter SLAM (EKF‑SLAM)** pipeline for a **differential‑drive robot** equipped with a simple **range‑bearing “radar” sensor**. The simulation is interactive (teleoperation) and visualizes **ground truth**, **odometry drift**, and the **EKF‑SLAM estimate** while incrementally building a landmark map under **unknown data association**.

---

## Contents

- [What this repo does](#what-this-repo-does)
- [Repository layout](#repository-layout)
- [Requirements](#requirements)
- [Install](#install)
- [Run](#run)
- [Controls](#controls)
- [Outputs and visualization](#outputs-and-visualization)
- [EKF‑SLAM: mathematical model](#ekf‑slam-mathematical-model)
  - [State vector and covariance](#state-vector-and-covariance)
  - [Differential‑drive motion model](#differential-drive-motion-model)
  - [RK4 integration (discretization)](#rk4-integration-discretization)
  - [Prediction step](#prediction-step)
  - [Range‑bearing measurement model](#range-bearing-measurement-model)
  - [Observation Jacobians](#observation-jacobians)
  - [Data association (unknown correspondence)](#data-association-unknown-correspondence)
  - [Landmark initialization (state augmentation)](#landmark-initialization-state-augmentation)
  - [EKF correction / update](#ekf-correction--update)
- [Tuning guide](#tuning-guide)
- [Known limitations](#known-limitations)
- [Roadmap / extensions](#roadmap--extensions)
- [References](#references)

---

## What this repo does

At a high level, the system runs a standard EKF‑SLAM loop:

1. **Teleop** sets left/right wheel velocities.
2. A motion model propagates a noisy odometry pose.
3. A simulated radar returns **range** and **bearing** to visible landmarks.
4. EKF‑SLAM performs:
   - **Prediction** using the control input and motion noise.
   - **Association** using **Mahalanobis distance gating** (nearest neighbor).
   - **Landmark initialization** for unseen measurements.
   - **Update** for associated landmarks.
5. Pygame renders the robot, sensor cone, landmarks, and trajectories.

---

## Repository layout

The repository is intentionally small. The core logic is split across a few files:

- **`main.py`**  
  Entry point: creates the simulation and EKF‑SLAM objects, runs the main loop, and triggers plotting.

- **`live_simulation.py`**  
  Pygame visualization + interactive loop (teleoperation, rendering, world/landmark setup).

- **`motion_and_radar.py`**  
  Differential‑drive motion model, RK4 integration utilities, and radar detection / field‑of‑view logic.

- **`ekf_slam.py`**  
  EKF‑SLAM implementation: prediction, association, state augmentation, and update steps.

- **`general_functions.py`**  
  Math helpers (angle wrapping, Jacobian helpers, numerical utilities).

> Tip: If you are publishing this repo, consider adding a `.gitignore` for editor folders and `__pycache__/`.


> **Images in this README:** The screenshots above are referenced as `pictures/...`.  
> When you add this README to GitHub, commit the two image files under `pictures/` so the images render correctly.


---

## Requirements

- Python 3.9+ (older versions may work, but 3.9+ is recommended)
- `numpy`
- `pygame`
- `matplotlib` (for trajectory/landmark plots and/or debugging)

---

## Install

Create and activate a virtual environment (recommended), then install dependencies:

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
source .venv/bin/activate

pip install numpy pygame matplotlib
```

---

## Run

From the repository root:

```bash
python main.py
```

If your system uses `python3`:

```bash
python3 main.py
```

---

## Controls

The simulation is designed for simple teleoperation:

- **Left arrow**: turn left (differential wheel speeds)
- **Right arrow**: turn right
- Releasing keys returns to a nominal straight‑line motion
- Close the window to exit (standard pygame quit event)

If you extend key bindings in `live_simulation.py`, update this section accordingly.

---

## Outputs and visualization

The visualization typically includes:

- **Robot** (black)
- **Ground truth / true trajectory** (often red)
- **Noisy odometry path** (often blue)
- **EKF‑SLAM estimated trajectory** (often purple)
- **Radar / sensor field‑of‑view** (green sector)
- **Landmarks**: true positions and estimated positions (as they are initialized/updated)

Plotting/summary figures are generated at the end of a run (or during debugging) using `matplotlib`.

## Screenshots

### EKF-SLAM trajectories and landmark estimates (real coordinates)

![Robot paths and observed landmarks](pictures/paths_landmarks.png)

This figure compares the **real path**, **odometry path**, and **EKF-SLAM path**, along with the **observed landmarks** in metric coordinates.

### Webots environment snapshot (extension work)

![Webots scene](pictures/webots_scene.png)

A representative Webots scene used to validate the same SLAM pipeline concepts in a physics-based simulator (differential drive + landmarks/obstacles).


---

# EKF‑SLAM: mathematical model

This section documents the model implemented in `ekf_slam.py` and the motion/measurement functions used by the simulation.

## State vector and covariance

EKF‑SLAM estimates the joint posterior over robot pose and all landmark positions.

Robot pose:
\[
\mathbf{x}_t = \begin{bmatrix} x_t \\ y_t \\ \theta_t \end{bmatrix}
\]

Landmarks (2D points):
\[
\mathbf{m}_i = \begin{bmatrix} m_{x,i} \\ m_{y,i} \end{bmatrix}
\]

Joint state:
\[
\boldsymbol{\mu}_t =
\begin{bmatrix}
\mathbf{x}_t \\
\mathbf{m}_1 \\
\vdots \\
\mathbf{m}_N
\end{bmatrix}
\in \mathbb{R}^{3 + 2N}
\]

Covariance:
\[
\boldsymbol{\Sigma}_t \in \mathbb{R}^{(3+2N)\times(3+2N)}
\]

The covariance contains:
- robot pose uncertainty
- landmark uncertainty
- cross‑correlations between robot and landmarks

---

## Differential‑drive motion model

With wheel linear velocities \(v_l, v_r\) and wheelbase \(b\):

\[
v = \frac{v_r + v_l}{2}, \qquad \omega = \frac{v_r - v_l}{b}
\]

Continuous‑time kinematics:

\[
\dot{x} = v\cos\theta,\qquad
\dot{y} = v\sin\theta,\qquad
\dot{\theta} = \omega
\]

---

## RK4 integration (discretization)

The simulator integrates the kinematics using **Runge‑Kutta 4 (RK4)**.  
For state \(\mathbf{x}\) and dynamics \(\dot{\mathbf{x}} = f(\mathbf{x}, u)\) with timestep \(\Delta t\):

\[
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_t, u)\,\Delta t \\
\mathbf{k}_2 &= f(\mathbf{x}_t + \tfrac{1}{2}\mathbf{k}_1, u)\,\Delta t \\
\mathbf{k}_3 &= f(\mathbf{x}_t + \tfrac{1}{2}\mathbf{k}_2, u)\,\Delta t \\
\mathbf{k}_4 &= f(\mathbf{x}_t + \mathbf{k}_3, u)\,\Delta t \\
\mathbf{x}_{t+1} &= \mathbf{x}_t + \tfrac{1}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)
\end{aligned}
\]

Angle normalization is applied after integration:
\[
\theta \leftarrow \mathrm{wrap}(\theta)\in(-\pi,\pi]
\]

---

## Prediction step

The EKF‑SLAM prediction applies the motion model to the **robot pose** portion of \(\boldsymbol{\mu}\), leaving landmarks unchanged:

\[
\boldsymbol{\mu}_{t|t-1} = g(\boldsymbol{\mu}_{t-1}, u_t)
\]

The covariance is propagated using the motion Jacobian \(G_t\) and motion noise \(R_t\):

\[
\boldsymbol{\Sigma}_{t|t-1} = G_t \boldsymbol{\Sigma}_{t-1} G_t^\top + R_t
\]

### Motion Jacobian (robot block)

A common small‑step approximation for the robot Jacobian (with \(v\) and \(\Delta t\)) is:

\[
G_x =
\begin{bmatrix}
1 & 0 & -v\Delta t\sin\theta \\
0 & 1 & \ \ v\Delta t\cos\theta \\
0 & 0 & 1
\end{bmatrix}
\]

In EKF‑SLAM, this is embedded in the full Jacobian \(G_t\) by placing \(G_x\) in the top‑left \(3\times3\) block and using identity elsewhere (landmarks unchanged):

\[
G_t =
\begin{bmatrix}
G_x & 0 \\
0 & I
\end{bmatrix}
\]

The motion noise \(R_t\) is similarly injected primarily into the robot pose block (and zeros elsewhere).

---

## Range‑bearing measurement model

The sensor returns range and bearing to a landmark \(i\):

Let:
\[
\Delta x = m_{x,i} - x,\quad \Delta y = m_{y,i} - y,\quad q = \Delta x^2 + \Delta y^2
\]

Expected measurement:
\[
\hat{z}_i(\boldsymbol{\mu}) =
\begin{bmatrix}
\hat{r}_i \\
\hat{\phi}_i
\end{bmatrix}
=
\begin{bmatrix}
\sqrt{q} \\
\mathrm{wrap}(\mathrm{atan2}(\Delta y,\Delta x) - \theta)
\end{bmatrix}
\]

Measured value:
\[
z =
\begin{bmatrix}
r \\
\phi
\end{bmatrix}
\]

Measurement noise:
\[
Q =
\begin{bmatrix}
\sigma_r^2 & 0 \\
0 & \sigma_\phi^2
\end{bmatrix}
\]

---

## Observation Jacobians

For a landmark \(i\), the Jacobian with respect to the robot pose and that landmark is:

\[
H_i =
\begin{bmatrix}
\frac{\partial \hat{r}}{\partial x} &
\frac{\partial \hat{r}}{\partial y} &
\frac{\partial \hat{r}}{\partial \theta} &
\frac{\partial \hat{r}}{\partial m_x} &
\frac{\partial \hat{r}}{\partial m_y}
\\[6pt]
\frac{\partial \hat{\phi}}{\partial x} &
\frac{\partial \hat{\phi}}{\partial y} &
\frac{\partial \hat{\phi}}{\partial \theta} &
\frac{\partial \hat{\phi}}{\partial m_x} &
\frac{\partial \hat{\phi}}{\partial m_y}
\end{bmatrix}
\]

Using \(r=\sqrt{q}\):

\[
\begin{aligned}
H_{x} &=
\begin{bmatrix}
-\Delta x/r & -\Delta y/r & 0\\
\Delta y/q & -\Delta x/q & -1
\end{bmatrix}
\\[6pt]
H_{m} &=
\begin{bmatrix}
\Delta x/r & \Delta y/r\\
-\Delta y/q & \Delta x/q
\end{bmatrix}
\end{aligned}
\]

The full EKF‑SLAM Jacobian for landmark \(i\) is placed into the global \(H\) matrix at the appropriate landmark indices.

---

## Data association (unknown correspondence)

Because correspondence is unknown, the implementation uses **nearest‑neighbor association** with **Mahalanobis distance gating**.

For a measurement \(z_k\) and a candidate landmark \(i\):

Innovation:
\[
\nu_i = z_k - \hat{z}_i
\]
(with bearing wrapped to \((-\pi,\pi]\))

Innovation covariance:
\[
S_i = H_i \boldsymbol{\Sigma} H_i^\top + Q
\]

Mahalanobis distance:
\[
d_i^2 = \nu_i^\top S_i^{-1}\nu_i
\]

Association rule:
- Choose \(i^* = \arg\min_i d_i^2\)
- If \(d_{i^*}^2 < \tau\) (threshold), associate to landmark \(i^*\)
- Otherwise, treat as a **new landmark**

### Choosing the threshold \(\tau\)

A principled choice is based on a \(\chi^2\) gate for 2 degrees of freedom:
- 95% gate: \(\tau \approx 5.99\)
- 99% gate: \(\tau \approx 9.21\)

In practice, this parameter may be tuned along with \(Q\).

---

## Landmark initialization (state augmentation)

When a measurement is not associated, a new landmark is created from the current robot pose and the measurement \((r,\phi)\):

\[
\begin{aligned}
m_x &= x + r\cos(\theta + \phi) \\
m_y &= y + r\sin(\theta + \phi)
\end{aligned}
\]

The mean vector is augmented:
\[
\boldsymbol{\mu} \leftarrow
\begin{bmatrix}
\boldsymbol{\mu} \\
m_x \\
m_y
\end{bmatrix}
\]

Covariance augmentation is done by expanding \(\boldsymbol{\Sigma}\). A standard approach uses Jacobians with respect to the robot pose and measurement:

Let \(\alpha = \theta + \phi\). Then:

\[
J_x =
\begin{bmatrix}
1 & 0 & -r\sin\alpha \\
0 & 1 & \ \ r\cos\alpha
\end{bmatrix},
\quad
J_z =
\begin{bmatrix}
\cos\alpha & -r\sin\alpha \\
\sin\alpha & \ \ r\cos\alpha
\end{bmatrix}
\]

The new landmark covariance block can be approximated by:
\[
\Sigma_{mm} = J_x \Sigma_{xx} J_x^\top + J_z Q J_z^\top
\]

Cross‑covariances between existing state and the new landmark are expanded consistently using \(J_x\).

> Implementation note: Exact augmentation details can vary; this repo uses the standard EKF‑SLAM pattern of expanding \(\mu\) and \(\Sigma\) when a landmark is first observed.

---

## EKF correction / update

For an associated landmark, we perform the standard EKF update.

Kalman gain:
\[
K = \boldsymbol{\Sigma}H^\top\left(H\boldsymbol{\Sigma}H^\top + Q\right)^{-1}
\]

State update:
\[
\boldsymbol{\mu} \leftarrow \boldsymbol{\mu} + K\,\nu
\]

Covariance update (Joseph form not required but more numerically stable):
\[
\boldsymbol{\Sigma} \leftarrow (I - KH)\boldsymbol{\Sigma}
\]

Angle wrap must be applied to the robot heading and bearing innovations.

---

# Tuning guide

The following parameters determine most of the system behavior:

## 1) Motion noise \(R\)
- Increasing \(R\) makes the filter trust odometry **less** and rely more on landmark updates.
- Too small: EKF can become inconsistent (overconfident), especially with unmodeled slip.
- Too large: estimate becomes noisy and may drift between updates.

## 2) Measurement noise \(Q\)
- Increasing \(Q\) makes the filter trust measurements **less**.
- Too small: aggressive corrections; association errors become catastrophic.
- Too large: landmarks converge slowly; the map remains uncertain.

## 3) Association threshold \(\tau\)
- Too low: the system creates duplicate landmarks (many “new” points).
- Too high: wrong associations distort both pose and map.

## 4) Sensor range/FOV
- Narrow FOV + sparse landmarks increases drift between updates.
- Wide FOV increases measurements but can increase association ambiguity.

---

# Known limitations

- **Nearest‑neighbor association** is simple and fast but can fail in dense environments.
- **EKF linearization** can be brittle for large nonlinearities or poor initializations.
- Landmark initialization accuracy depends heavily on the current pose estimate and measurement noise.
- Numerical stability can degrade if covariance is not kept symmetric / positive semidefinite (PSDs).

---

# Roadmap / extensions

If you plan to extend this work, common next steps are:

1. **Robust data association**
   - JCBB (Joint Compatibility Branch and Bound)
   - Multi‑hypothesis tracking

2. **Better motion model**
   - Explicit wheel encoder noise model
   - Slip and bias modeling

3. **Consistency improvements**
   - Joseph‑form covariance update
   - Symmetrization of \(\Sigma\)

4. **Webots integration**
   - Replace simulated radar with Webots sensor observations
   - Use wheel encoders for odometry and validate against ground truth
   - Compare EKF‑SLAM trajectories and maps across scenarios

---

# References

- S. Thrun, W. Burgard, D. Fox — *Probabilistic Robotics*
- Cyrill Stachniss — EKF‑SLAM lecture notes
- Standard EKF‑SLAM derivations (range‑bearing landmark SLAM)
