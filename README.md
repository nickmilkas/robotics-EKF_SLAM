# EKF-SLAM with Unknown Correspondence (Differential Drive + Range–Bearing Sensor)

This repository contains my implementation of **2D EKF-SLAM** for a **differential-drive robot** equipped with a forward **range–bearing (“radar”) sensor**. The system estimates the robot pose and a landmark map **simultaneously**, under **unknown data association**, and visualizes the resulting trajectories and landmark estimates.

---

## Visual results

### EKF-SLAM trajectories and landmark estimates (real coordinates)

![Robot paths and observed landmarks](pictures/paths_landmarks.png)

The plot compares the **real path**, **odometry-only path**, and the **EKF-SLAM path**, together with the set of **observed landmarks** in metric coordinates.

### Webots environment snapshot (extension work)

![Webots scene](pictures/webots_scene.png)

A representative Webots setup used to validate the same pipeline concepts in a physics-based simulator (differential drive + obstacles/landmarks).

---

## What I implemented

### 1) Interactive simulation and sensing
- A 2D simulation with **teleoperation** control of left/right wheel velocities.
- A forward-facing **sensor field-of-view** and range–bearing detections to landmarks.
- Continuous motion integrated with **Runge–Kutta 4 (RK4)** to improve numerical stability versus Euler integration.

### 2) Full EKF-SLAM state and covariance
- Joint state $\boldsymbol{\mu}$ contains the **robot pose** and all **landmark positions**.
- Joint covariance $\boldsymbol{\Sigma}$ maintains robot uncertainty, landmark uncertainty, and their cross-correlations.

### 3) Unknown correspondence via Mahalanobis gating
- Each incoming observation is compared against all existing landmarks using **Mahalanobis distance**.
- If no landmark falls within a gating threshold, a **new landmark** is initialized (state augmentation).

---

## Code organization (high-level)

- `main.py`  
  Orchestrates the simulation + EKF-SLAM loop and produces plots.

- `live_simulation.py`  
  Interactive visualization and environment/landmark setup.

- `motion_and_radar.py`  
  Differential-drive kinematics, RK4 integration utilities, and radar/visibility logic.

- `ekf_slam.py`  
  EKF-SLAM prediction, association, landmark initialization (augmentation), and update.

- `general_functions.py`  
  Helper functions (angle wrapping, small math utilities, etc.).

---

# EKF-SLAM formulation used in this project

## Notation and joint state

Robot pose:
$$
\mathbf{x}_t =
\begin{bmatrix}
x_t\\
y_t\\
\theta_t
\end{bmatrix}.
$$

Landmark $i$:
$$
\mathbf{m}_i =
\begin{bmatrix}
m_{x,i}\\
m_{y,i}
\end{bmatrix}.
$$

Joint EKF-SLAM state (robot + $N$ landmarks):
$$
\boldsymbol{\mu}_t =
\begin{bmatrix}
\mathbf{x}_t\\
\mathbf{m}_1\\
\vdots\\
\mathbf{m}_N
\end{bmatrix}
\in \mathbb{R}^{3+2N},
\qquad
\boldsymbol{\Sigma}_t \in \mathbb{R}^{(3+2N)\times(3+2N)}.
$$

Angle wrapping is applied throughout:
$$
\operatorname{wrap}(\alpha) \in (-\pi,\pi].
$$

---

## Differential-drive motion model

Given left/right wheel linear velocities $(v_l, v_r)$ and wheelbase $b$:
$$
v = \frac{v_r + v_l}{2},
\qquad
\omega = \frac{v_r - v_l}{b}.
$$

Continuous-time kinematics:
$$
\dot{x} = v\cos\theta,
\qquad
\dot{y} = v\sin\theta,
\qquad
\dot{\theta} = \omega.
$$

### RK4 discretization

For $\dot{\mathbf{x}} = f(\mathbf{x},u)$ and timestep $\Delta t$:
$$
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_t,u)\Delta t\\
\mathbf{k}_2 &= f\!\left(\mathbf{x}_t+\tfrac{1}{2}\mathbf{k}_1,u\right)\Delta t\\
\mathbf{k}_3 &= f\!\left(\mathbf{x}_t+\tfrac{1}{2}\mathbf{k}_2,u\right)\Delta t\\
\mathbf{k}_4 &= f(\mathbf{x}_t+\mathbf{k}_3,u)\Delta t\\
\mathbf{x}_{t+1} &= \mathbf{x}_t+\tfrac{1}{6}\left(\mathbf{k}_1+2\mathbf{k}_2+2\mathbf{k}_3+\mathbf{k}_4\right).
\end{aligned}
$$

---

## EKF prediction

The prediction step propagates the **robot pose** portion of $\boldsymbol{\mu}$ using the motion model and leaves landmarks unchanged:
$$
\boldsymbol{\mu}_{t|t-1} = g(\boldsymbol{\mu}_{t-1},u_t).
$$

Covariance propagation:
$$
\boldsymbol{\Sigma}_{t|t-1} = G_t\,\boldsymbol{\Sigma}_{t-1}\,G_t^\top + R_t.
$$

A common small-step approximation for the robot Jacobian block (embedded into the full $G_t$) is:
$$
G_x =
\begin{bmatrix}
1 & 0 & -v\Delta t\sin\theta\\
0 & 1 & \ \ v\Delta t\cos\theta\\
0 & 0 & 1
\end{bmatrix},
\qquad
G_t=
\begin{bmatrix}
G_x & 0\\
0 & I_{2N}
\end{bmatrix}.
$$

$R_t$ injects motion uncertainty primarily into the robot pose block (with zeros elsewhere).

---

## Range–bearing measurement model

For landmark $i$, define:
$$
\Delta x = m_{x,i}-x,\quad \Delta y = m_{y,i}-y,\quad q=\Delta x^2+\Delta y^2,\quad r=\sqrt{q}.
$$

Expected measurement:
$$
\hat{\mathbf{z}}_i(\boldsymbol{\mu})=
\begin{bmatrix}
\hat{r}_i\\
\hat{\phi}_i
\end{bmatrix}
=
\begin{bmatrix}
r\\
\operatorname{wrap}\!\left(\operatorname{atan2}(\Delta y,\Delta x)-\theta\right)
\end{bmatrix}.
$$

Observed measurement:
$$
\mathbf{z}=
\begin{bmatrix}
r\\
\phi
\end{bmatrix}.
$$

Measurement noise:
$$
Q=
\begin{bmatrix}
\sigma_r^2 & 0\\
0 & \sigma_\phi^2
\end{bmatrix}.
$$

---

## Observation Jacobians

The Jacobian for landmark $i$ with respect to robot pose and that landmark has the form:
$$
H_i=
\begin{bmatrix}
H_x & 0 & \cdots & H_m & \cdots & 0
\end{bmatrix},
$$
where the non-zero blocks are:

Robot block:
$$
H_x=
\begin{bmatrix}
-\Delta x/r & -\Delta y/r & 0\\
\Delta y/q & -\Delta x/q & -1
\end{bmatrix}.
$$

Landmark block:
$$
H_m=
\begin{bmatrix}
\Delta x/r & \Delta y/r\\
-\Delta y/q & \Delta x/q
\end{bmatrix}.
$$

---

## Unknown correspondence (data association)

For an incoming observation $\mathbf{z}_k$ and candidate landmark $i$:

Innovation (with angle wrapping on the bearing residual):
$$
\boldsymbol{\nu}_i = \mathbf{z}_k - \hat{\mathbf{z}}_i,
\qquad
\nu_{\phi,i}\leftarrow \operatorname{wrap}(\nu_{\phi,i}).
$$

Innovation covariance:
$$
S_i = H_i\,\boldsymbol{\Sigma}\,H_i^\top + Q.
$$

Mahalanobis distance:
$$
d_i^2 = \boldsymbol{\nu}_i^\top S_i^{-1}\boldsymbol{\nu}_i.
$$

Association rule (nearest neighbor + gating):
- Choose $i^\*=\arg\min_i d_i^2$
- If $d_{i^\*}^2 < \tau$, associate to landmark $i^\*$
- Otherwise initialize a new landmark

---

## Landmark initialization (state augmentation)

When a measurement is not associated, a new landmark estimate is initialized from the current pose:
$$
\begin{aligned}
m_x &= x + r\cos(\theta+\phi),\\
m_y &= y + r\sin(\theta+\phi).
\end{aligned}
$$

The state is augmented:
$$
\boldsymbol{\mu}\leftarrow
\begin{bmatrix}
\boldsymbol{\mu}\\
m_x\\
m_y
\end{bmatrix}.
$$

A standard augmentation approach uses Jacobians w.r.t. pose and measurement.
Let $\alpha=\theta+\phi$:
$$
J_x=
\begin{bmatrix}
1 & 0 & -r\sin\alpha\\
0 & 1 & \ \ r\cos\alpha
\end{bmatrix},
\qquad
J_z=
\begin{bmatrix}
\cos\alpha & -r\sin\alpha\\
\sin\alpha & \ \ r\cos\alpha
\end{bmatrix}.
$$

One common landmark covariance seed is:
$$
\Sigma_{mm} = J_x\,\Sigma_{xx}\,J_x^\top + J_z\,Q\,J_z^\top,
$$
and cross-covariances are expanded consistently using $J_x$.

---

## EKF update

For an associated landmark:
$$
K = \boldsymbol{\Sigma}H^\top\left(H\boldsymbol{\Sigma}H^\top + Q\right)^{-1}.
$$

State update:
$$
\boldsymbol{\mu}\leftarrow \boldsymbol{\mu} + K\boldsymbol{\nu}.
$$

Covariance update:
$$
\boldsymbol{\Sigma}\leftarrow (I-KH)\boldsymbol{\Sigma}.
$$

---

## References (theoretical background)

- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*
- Standard EKF-SLAM derivations for range–bearing landmark SLAM
