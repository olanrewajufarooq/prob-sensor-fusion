# Probabilistic Guarantees for Sensor Fusion under Noise Mis-Specification

Kalman, EKF, and NIS-Driven Robust Variants on Unicycle Dynamics

## Table of Contents

- [1. Project Overview](#1-project-overview)
- [2. Dynamics](#2-dynamics)
- [3. Trajectories](#3-trajectories)
- [4. Filters](#4-filters)
- [5. Robustness Rationale (Markov and Chebyshev)](#5-robustness-rationale-markov-and-chebyshev)
- [6. Theoretical Metrics (NEES vs NIS)](#6-theoretical-metrics-nees-vs-nis)
- [7. Experimental Setup](#7-experimental-setup)
- [8. Project Structure](#8-project-structure)
- [9. How to Run](#9-how-to-run)
- [10. Video Guide](#10-video-guide)

---

## 1. Project Overview

We study localization under noise mis-specification on a unicycle robot with PID control. Four filters are compared:

| Filter      | Dynamics model        | Noise model            | Notes                                        |
|-------------|-----------------------|------------------------|----------------------------------------------|
| KF          | Linear CV             | Gaussian               | Overconfident under nonlinearity/mis-noise   |
| EKF         | Nonlinear unicycle    | Gaussian               | Baseline with correct kinematics             |
| Robust KF   | Linear CV             | Adaptive via NIS       | Covariance inflation + outlier rejection     |
| Robust EKF  | Nonlinear unicycle    | Adaptive via NIS       | Proposed: correct model + adaptive covariance|

The core contribution is demonstrating that distribution-agnostic robustness mechanisms (Markov-based outlier rejection and Chebyshev-based covariance inflation) can mitigate performance degradation under heavy-tailed and correlated noise, while maintaining theoretical guarantees without knowledge of the true noise distribution.

[Return to Table of Contents](#table-of-contents)

---

## 2. Dynamics

### State Vector

$$
x = [p_x,\; p_y,\; \theta,\; v]^\top
$$

where $p_x$ and $p_y$ are position, $\theta$ is heading, and $v$ is forward speed.

### Control Input

$$
u = [v_c,\; \omega_c]^\top
$$

where $v_c$ is commanded velocity and $\omega_c$ is commanded angular rate.

### Process Noise

$$
w \sim \mathcal{N}(0,Q)
$$

### Unicycle Motion Dynamics

The nonlinear unicycle kinematic model updates the state as follows:

$$
p_x^+ = p_x + v \cos\theta\,\Delta t + w_1
$$

$$
p_y^+ = p_y + v \sin\theta\,\Delta t + w_2
$$

$$
\theta^+ = \theta + \omega_c\,\Delta t + w_3
$$

$$
v^+ = v + v_c\,\Delta t + w_4
$$

### Linear Constant-Velocity Model (for KF)

The standard Kalman Filter uses a simplified linear model:

$$
x^+ = F_{\text{linear}} x + G u + w
$$

where the state transition matrix is:

$$
F_{\text{linear}} = \begin{bmatrix} 
1 & 0 & \Delta t & 0 \\ 
0 & 1 & 0 & \Delta t \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix}
$$

This matrix couples position ($p_x, p_y$) to velocity ($\dot{p}_x, \dot{p}_y$) through the time step $\Delta t$. The process noise covariance is:

$$
Q_{\text{linear}} = Q
$$

This linear model ignores the nonlinear coupling of position with heading and velocity, and is used only by the standard KF. In contrast, the Extended Kalman Filter (EKF) uses the nonlinear unicycle dynamics that properly capture how velocity and heading determine position changes.

[Return to Table of Contents](#table-of-contents)

---

## 3. Trajectories

### Circular Trajectory

$$
p_x = R\cos(\omega t),\quad p_y = R\sin(\omega t)
$$

$$
\theta = \text{atan2}(\dot p_y,\dot p_x),\quad v = \sqrt{\dot p_x^2+\dot p_y^2}
$$

### Figure-8 Trajectory

$$
p_x = R\sin(\omega t),\quad p_y = R\sin(\omega t)\cos(\omega t)
$$

The heading $\theta$ and speed $v$ are derived from the time derivatives of $p_x$ and $p_y$.

### Spiral Trajectory

$$
R(t)=R\left(0.1+0.9\,\frac{t}{T_{\text{full}}}\right),\quad \omega_t=2\omega
$$

$$
p_x=R(t)\cos(\omega_t t),\quad p_y=R(t)\sin(\omega_t t)
$$

The heading $\theta$ and speed $v$ are derived from the time derivatives of $p_x$ and $p_y$. The radius expands linearly over the simulation, creating an outward spiral.

### High-Curvature Trajectory

$$
p_x = R\sin(2\omega t)\cos(\omega t),\quad p_y = R\sin(\omega t)\cos(2\omega t)
$$

This trajectory combines two oscillation frequencies to create sharp, unpredictable turns and rapid heading changes. It is designed to expose the limitations of the standard EKF by creating scenarios where:

- **Linearization errors accumulate** due to extreme curvature
- **Filter divergence is likely** without adaptive mechanisms
- **RobustEKF's covariance adaptation** should significantly improve performance

This trajectory is ideal for demonstrating when robust filtering becomes necessary.

[Return to Table of Contents](#table-of-contents)

## 4. Filters

### 4.1 Kalman Filter (KF)

#### Prediction Step

$$
x^- = F x^+ + G u
$$

$$
P^- = F P F^\top + Q
$$

#### Update (Correction) Step

$$
K = P^- H^\top (H P^- H^\top + R)^{-1}
$$

$$
x^+ = x^- + K(z - H x^-)
$$

$$
P^+ = (I-KH)P^-
$$

The standard Kalman Filter assumes linear dynamics and Gaussian, uncorrelated measurement noise. It is used with the linear constant-velocity model and provides a baseline that underperforms when the nonlinear unicycle dynamics are significant or when noise is non-Gaussian.

---

### 4.2 Extended Kalman Filter (EKF)

#### Nonlinear Prediction

The EKF prediction step uses the nonlinear unicycle kinematics:

$$
x^- = f(x^+, u)
$$

$$
P^- = F_k P F_k^\top + Q
$$

where $F_k$ is the Jacobian matrix of $f$ evaluated at the current state (see derivation below).

#### Measurement Update

The update step matches the KF form:

$$
K = P^- H^\top (H P^- H^\top + R)^{-1}
$$

$$
x^+ = x^- + K(z - H x^-)
$$

$$
P^+ = (I-KH)P^-
$$

#### Jacobian Matrix Derivation for Unicycle Dynamics

We define the nonlinear state transition function as:

$$
f_1(x, u) = p_x + v \cos\theta\,\Delta t
$$

$$
f_2(x, u) = p_y + v \sin\theta\,\Delta t
$$

$$
f_3(x, u) = \theta + \omega_c\,\Delta t
$$

$$
f_4(x, u) = v + v_c\,\Delta t
$$

The Jacobian matrix $F_k = \frac{\partial f}{\partial x}$ is computed by taking partial derivatives of each component with respect to each state variable.

**First row** ($\frac{\partial f_1}{\partial x_i}$):

$$
\frac{\partial f_1}{\partial p_x} = 1,\quad \frac{\partial f_1}{\partial p_y} = 0,\quad \frac{\partial f_1}{\partial \theta} = -v \sin\theta\,\Delta t,\quad \frac{\partial f_1}{\partial v} = \cos\theta\,\Delta t
$$

**Second row** ($\frac{\partial f_2}{\partial x_i}$):

$$
\frac{\partial f_2}{\partial p_x} = 0,\quad \frac{\partial f_2}{\partial p_y} = 1,\quad \frac{\partial f_2}{\partial \theta} = v \cos\theta\,\Delta t,\quad \frac{\partial f_2}{\partial v} = \sin\theta\,\Delta t
$$

**Third row** ($\frac{\partial f_3}{\partial x_i}$):

$$
\frac{\partial f_3}{\partial p_x} = 0,\quad \frac{\partial f_3}{\partial p_y} = 0,\quad \frac{\partial f_3}{\partial \theta} = 1,\quad \frac{\partial f_3}{\partial v} = 0
$$

**Fourth row** ($\frac{\partial f_4}{\partial x_i}$):

$$
\frac{\partial f_4}{\partial p_x} = 0,\quad \frac{\partial f_4}{\partial p_y} = 0,\quad \frac{\partial f_4}{\partial \theta} = 0,\quad \frac{\partial f_4}{\partial v} = 1
$$

**Assembled Jacobian Matrix:**

$$
F_k = \begin{bmatrix}
1 & 0 & -v \sin\theta\,\Delta t & \cos\theta\,\Delta t\\
0 & 1 & v \cos\theta\,\Delta t & \sin\theta\,\Delta t\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

**Physical Interpretation:** The off-diagonal terms in the first two rows show the coupling between heading $\theta$, velocity $v$, and position changes. The diagonal-dominated structure in the heading and velocity rows reflects that these variables evolve mostly independently of position.

---

### 4.3 Robust KF and Robust EKF

Both robust variants employ the same adaptive mechanisms applied on top of either linear (Robust KF) or nonlinear (Robust EKF) prediction.

#### Innovation and Innovation Covariance

$$
r = z - H x^-
$$

$$
S = H P^- H^\top + R
$$

#### Normalized Innovation Squared (NIS)

$$
\nu = r^\top S^{-1} r
$$

This is a key metric used to detect estimation inconsistency.

#### Defense Mechanism 1: Acute Outlier Rejection (Markov-based)

When a single measurement is extreme (large residual), we detect and downweight it using a Markov inequality threshold.

**Threshold Computation:**

$$
\mathbb{E}[\|r\|^2] \approx \text{tr}(S)
$$

$$
T = \frac{\text{tr}(S)}{\delta}
$$

with $\delta = 10^{-3}$.

**Outlier Detection and Handling:**

If $\|r\|^2 > T$, inflate the measurement noise for that step:

$$
R_{\text{used}} = \gamma_R\, R,\quad \gamma_R \gg 1
$$

Recompute the Kalman gain and update using the inflated $S$:

$$
S = H P^- H^\top + R_{\text{used}}
$$

$$
K = P^- H^\top S^{-1}
$$

Otherwise, use nominal $R$.

**Effect:** Outliers contribute less to the state correction, preventing filter divergence from a single bad measurement.

#### Defense Mechanism 2: Chronic Inconsistency Detection (Chebyshev-based)

When the NIS is consistently elevated (chronic underestimation of covariance), inflate the state covariance.

**Windowed NIS Mean:**

Maintain a sliding window of size $W = 20$ and compute the mean NIS:

$$
\bar\nu = \frac{1}{W}\sum_{i=1}^{W} \nu_i
$$

**Trigger Condition:**

$$
\bar\nu > nz\left(1+2\sqrt{\frac{1}{W}}\right)
$$

where $nz$ is the measurement dimension (number of sensor readings per update).

**Covariance Inflation:**

When triggered, inflate the state covariance:

$$
P \leftarrow \alpha\, P,\quad \alpha \uparrow \text{ (capped at 3.0)}
$$

**Covariance Decay:**

When consistent (NIS mean within bounds), allow the inflation factor to decay:

$$
\alpha \leftarrow \max(1.0, \beta \alpha),\quad \beta < 1
$$

**Effect:** Persistent underestimation of uncertainty is countered by gradually increasing the filter's state uncertainty, which widens the innovation threshold and accommodates larger residuals.

**Rationale:** Markov handles rare, extreme residuals by inflating R; Chebyshev handles persistent underestimation by inflating P over time. Both mechanisms are distribution-agnostic (rely only on moments) and enable the filter to adapt without knowledge of the true noise distribution.

[Return to Table of Contents](#table-of-contents)

---

## 5. Robustness Rationale (Markov and Chebyshev)

### Markov Inequality Derivation

**Statement:** For a nonnegative random variable $X$,

$$
P(X > T) \le \frac{\mathbb{E}[X]}{T}
$$

**Proof Sketch:** The Markov inequality follows from the observation that for any $T > 0$,

$$
\mathbb{E}[X] = \int_0^{\infty} P(X > t)\, dt \ge \int_0^{T} P(X > t)\, dt + \int_{T}^{\infty} P(X > t)\, dt
$$

On the interval $[T, \infty)$, we have $P(X > t) \le 1$, so:

$$
\mathbb{E}[X] \ge \int_T^{\infty} P(X > t)\, dt \ge T \cdot P(X > T)
$$

Rearranging yields:

$$
P(X > T) \le \frac{\mathbb{E}[X]}{T}
$$

**Application to Outlier Detection:**

In our filter, $X = \|r\|^2$ is the squared residual norm. If we choose

$$
T = \frac{\mathbb{E}[\|r\|^2]}{\delta}
$$

with $\delta = 10^{-3}$, then:

$$
P(\|r\|^2 > T) \le \delta = 10^{-3}
$$

This ensures that an outlier (measurement exceeding the threshold) occurs with probability at most 0.1%. When $\|r\|^2 > T$, we inflate the measurement noise covariance $R$ to downweight that measurement, preventing a single outlier from corrupting the estimate.

**Practical Estimation:** We approximate $\mathbb{E}[\|r\|^2]$ by the trace of the innovation covariance:

$$
\mathbb{E}[\|r\|^2] \approx \text{tr}(S) = \text{tr}(H P^- H^\top + R)
$$

This approximation holds when the residuals are approximately centered at zero (filter is consistent).

---

### Chebyshev Inequality Derivation

**Statement:** For a random variable $Y$ with finite mean $\mu$ and variance $\sigma^2$,

$$
P(|Y - \mu| > k) \le \frac{\sigma^2}{k^2}
$$

**Proof Sketch:** By Markov's inequality applied to $(Y - \mu)^2$:

$$
P(|Y - \mu| > k) = P((Y - \mu)^2 > k^2) \le \frac{\mathbb{E}[(Y - \mu)^2]}{k^2} = \frac{\sigma^2}{k^2}
$$

**Application to NIS Mean:**

Let $\bar\nu = \frac{1}{W}\sum_{i=1}^{W} \nu_i$ be the sample mean of NIS values over a window of size $W$.

Under nominal conditions (filter is consistent), $\mathbb{E}[\nu] = nz$ (the measurement dimension), where $nz$ is the number of sensors.

The variance of the window mean is:

$$
\bar{\nu} = \frac{\nu}{W}
$$

Under Gaussian assumptions, $\nu \approx 2 nz^2$, so:

$$
\bar{\nu} \approx \frac{2 nz^2}{W}
$$

Therefore:

$$
\sigma_{\bar\nu} = \sqrt{\bar{\nu}} \approx nz\sqrt{\frac{2}{W}}
$$

**Trigger Threshold:**

Applying Chebyshev with confidence level, we choose:

$$
k = 2\sigma_{\bar\nu} \approx nz \cdot 2\sqrt{\frac{2}{W}}
$$

The trigger condition is:

$$
\bar\nu > \mu_{\bar\nu} + k = nz + nz \cdot 2\sqrt{\frac{2}{W}} = nz\left(1 + 2\sqrt{\frac{2}{W}}\right)
$$

When this threshold is exceeded, the window mean is significantly above the nominal expected value, indicating that the filter is underestimating uncertainty (chronic inconsistency). We then inflate the state covariance $P$ to account for the underestimation.

**Response:** Upon triggering, we inflate $P$ by a factor $\alpha$:

$$
P \leftarrow \alpha P,\quad \alpha \text{ increases (capped at 3.0)}
$$

As consistency improves, $\alpha$ decays:

$$
\alpha \leftarrow \max(1.0, \beta \alpha),\quad \beta = 0.995 \text{ (or similar)}
$$

---

### Summary of Robustness Mechanisms

| Mechanism | Inequality | Detection | Response |
|-----------|-----------|-----------|----------|
| **Acute Outliers** | Markov | $\|r\|^2 > \frac{\text{tr}(S)}{\delta}$ | Inflate $R$ for that step |
| **Chronic Underestimation** | Chebyshev | $\bar\nu > nz(1+2\sqrt{\frac{2}{W}})$ | Gradually inflate $P$ over time |

Both mechanisms are **distribution-agnostic**: they depend only on the mean (and variance for Chebyshev), not on the specific shape of the noise distribution. This allows them to provide robustness against heavy-tailed, correlated, and other non-Gaussian noise models without explicit knowledge of the true distribution.

[Return to Table of Contents](#table-of-contents)

---

## 6. Theoretical Metrics (NEES vs NIS)

### Normalized Estimation Error Squared (NEES)

$$
\epsilon_t = e_t^\top P_t^{-1} e_t
$$

where $e_t = x_t - x_t^{\text{true}}$ is the estimation error (true state minus estimated state).

**Properties:**
- $\epsilon_t$ follows a chi-squared distribution with $n_x$ degrees of freedom (where $n_x = 4$ for our system).
- Expected value: $\mathbb{E}[\epsilon_t] = n_x = 4$.
- Requires ground truth $x_t^{\text{true}}$ to compute.

**When Available:** NEES can only be computed in simulation when the true state is known. It is used **post-simulation** to assess filter consistency and estimate quality.

**Interpretation:** 
- If $\bar\epsilon = \frac{1}{T}\sum_{t=1}^{T} \epsilon_t \approx n_x$, the filter's covariance estimates are well-calibrated.
- If $\bar\epsilon > n_x$, the filter is underconfident (covariance too small); if $\bar\epsilon < n_x$, overconfident (covariance too large).

---

### Normalized Innovation Squared (NIS)

$$
\nu_t = r_t^\top S_t^{-1} r_t
$$

where $r_t = z_t - H x_t^-$ is the innovation (measurement residual) and $S_t = H P_t^- H^\top + R$ is the innovation covariance.

**Properties:**
- $\nu_t$ follows a chi-squared distribution with $n_z$ degrees of freedom (where $n_z$ is the number of measurements).
- Expected value: $\mathbb{E}[\nu_t] = n_z$.
- Depends only on estimated state $x_t^-$, innovation $r_t$, and covariance matrices (no ground truth required).

**When Available:** NIS can be computed **during simulation** using only the filter's own predictions and measurements. It requires no ground truth.

**Interpretation:**
- If $\bar\nu = \frac{1}{T}\sum_{t=1}^{T} \nu_t \approx n_z$, the filter is consistent.
- If $\bar\nu > n_z$, the filter is underconfident (covariance too small, innovations larger than expected).
- If $\bar\nu < n_z$, the filter is overconfident (covariance too large, innovations smaller than expected).

---

### Why NEES Post-Simulation and NIS During Simulation

**NEES (Offline/Post-Processing):**
- Requires knowledge of the true state $x_t^{\text{true}}$, which is only available in simulation with a ground-truth trajectory.
- Cannot be used in real-world deployment without external ground truth (e.g., a motion capture system).
- Provides the most direct assessment of estimation error magnitude relative to filter confidence.

**NIS (Online/Real-Time):**
- Computable entirely from the filter's own quantities: predicted state, measurements, and covariances.
- Can be computed during simulation and in real deployments.
- Reflects the consistency between predicted uncertainty ($P$) and observed residuals.
- Used by the Robust KF/EKF to **detect inconsistency and trigger adaptive responses** in real-time.

**Synergy in This Project:**
- During simulation, we compute **both** NEES and NIS post-hoc.
- NIS is used **within the filter** (during simulation) to drive the Markov and Chebyshev adaptive mechanisms.
- NEES is used **after simulation** to validate overall filter performance and covariance calibration.
- In a real deployment, only NIS would be available, but NIS alone suffices to maintain robustness via the adaptive covariance mechanisms.

[Return to Table of Contents](#table-of-contents)

---

## 7. Experimental Setup

### Trajectories and Scenarios

Three reference trajectories are tested:
1. **Circular**: Robot follows a circle of constant radius at constant angular velocity. Provides steady-state localization error under nominal conditions.
2. **Figure-8**: More complex path with varying curvature. Tests filter robustness to accelerations and higher-frequency maneuvers.
3. **Spiral**: Outward spiral with linearly increasing radius. Tests filter behavior during expanding motion and sustained nonlinearity.

### Noise Regimes

Three noise models are tested to evaluate robustness under different assumptions:

#### Baseline: Gaussian Noise
Pure Gaussian white noise with covariance $Q$ (process) and $R$ (measurement).

**Rationale:** Establishes baseline performance when the filter's noise assumptions are correct.

#### Correlated Gaussian Noise
Gaussian noise with temporal correlation structure. Correlation coefficient $\rho \in \{0.3, 0.5, 0.7, 0.9\}$.

**Model:** 
$$
w_t = \rho w_{t-1} + \sqrt{1 - \rho^2}\, \zeta_t,\quad \zeta_t \sim \mathcal{N}(0, \sigma^2)
$$

**Rationale:** Tests whether filters (which assume white noise) degrade under temporal correlations. Higher $\rho$ means stronger correlation and more severe filter mismatch.

#### Heavy-Tail Mixture Noise
Mixture of Gaussian and outliers to model sensor faults or communication glitches.

**Model:**
$$
w_t \sim \begin{cases}
\mathcal{N}(0, \sigma^2) & \text{w.p. } 1 - \pi \\
\mathcal{N}(0, (\lambda\sigma)^2) & \text{w.p. } \pi
\end{cases}
$$

- **Outlier rate** $\pi \in \{0.01, 0.05, 0.1\}$: Fraction of measurements that are outliers.
- **Outlier scale** $\lambda \in \{3, 5, 10\}$: Ratio of outlier variance to nominal variance.

**Rationale:** Tests robustness to rare, large-magnitude measurement errors. Standard filters (KF, EKF) perform poorly without explicit outlier rejection; robust variants benefit from Markov-based threshold detection.

### Simulation Parameters

All scenarios use the following parameters (defined in `params_*.m` files):

| Parameter | Value | Description |
|-----------|-------|-------------|
| Time step | $\Delta t = 0.1$ s | Discrete time interval |
| Horizon | $T = 400$ s | Total simulation duration |
| Trials per config | 500 | Monte Carlo runs for statistics |
| Robot max speed | $v_{\max} = 1.0$ m/s | Speed constraint in PID |
| Robot max yaw rate | $\omega_{\max} = 1.0$ rad/s | Yaw rate constraint in PID |
| Circular radius | $R = 5$ m | Radius of circular and spiral trajectories |
| Angular frequency | $\omega = 0.2$ rad/s | Oscillation frequency in periodic trajectories |

### Sensor Configuration

The robot has two sensors:

1. **GPS (Position Sensor):** 
   - Measures $[p_x, p_y]$ with covariance $R_{\text{GPS}}$ (typically $\text{diag}(0.1^2, 0.1^2)$).
   - Measurement noise is Gaussian (in baseline) or correlated/heavy-tail (in other regimes).

2. **Odometry (Velocity Sensor):**
   - Measures $[v, \omega]$ (forward speed and yaw rate) with covariance $R_{\text{odo}}$ (typically $\text{diag}(0.05^2, 0.05^2)$).
   - Subject to the same noise regimes as GPS.

Both sensors are fused into a combined measurement $z = [p_x, p_y, v, \omega]^T$.

### Process Noise

Process noise covariance:

$$
Q = \text{diag}(q_1^2, q_2^2, q_3^2, q_4^2)
$$

where $q_i$ values are set to reflect model uncertainty (typically $Q = \text{diag}(0.01^2, 0.01^2, 0.01^2, 0.01^2)$).

### Output and Storage

**Results:** Each trial produces a `.mat` file containing:
- Estimated states: $\hat{x}_{1:T}$, $\hat{P}_{1:T}$
- True trajectory: $x_{1:T}^{\text{true}}$
- Residuals and innovations
- NEES and NIS time series
- Summary statistics

**Organization:** Results are stored in `results/<regime>/<trajectory>/`, e.g., `results/correlated/Circular/correlated_rho0.7_Circular_EKF.mat`.

**Caching:** Existing result files are **not overwritten**; the experiments only run configurations not yet stored. This avoids redundant computation and supports incremental experiment building.

### Metrics Computed

1. **Mean Squared Error (MSE):** Average position and velocity estimation error.
2. **NEES:** Used post-simulation to validate filter calibration.
3. **NIS:** Used online for adaptive covariance triggering and post-simulation for consistency analysis.
4. **Consistency Plots:** Time-averaged NEES and NIS with theoretical bounds.
5. **Error Histograms:** Distribution of position and velocity errors.
6. **Trajectory Plots:** Overlay of true and estimated paths with confidence ellipses.

[Return to Table of Contents](#table-of-contents)

---

## 8. Project Structure
```
prob-sensor-fusion/
├ dynamics/       % Unicycle model, PID, trajectories
├ filters/        % KF, EKF, RobustKF, RobustEKF
├ noises/         % Gaussian, correlated Gaussian, mixture
├ sensors/        % GPS, odometry, combined sensors
├ scripts/        % run_all, run_video_case, sweeps
├ utils/          % Experiment runner, plotting, metrics, params, paths
├ results/        % .mat outputs by scenario/trajectory
├ plots/          % Generated PNG figures
└ video_exports/  % Generated videos
```

[Return to Table of Contents](#table-of-contents)

---

## 9. How to Run

### Prerequisites

- **MATLAB** (R2020b or later)
- **Statistics and Machine Learning Toolbox** (for statistical functions)
- **Simulink** (optional, for visualization)

### Quick Start

1. **Set up paths:**
   ```matlab
   setup_paths
   ```
   This adds all folders (`dynamics/`, `filters/`, `sensors/`, etc.) to the MATLAB path.

2. **Run all experiments:**
   ```matlab
   run_all
   ```
   This executes all 36 configurations (4 filters × 3 trajectories × 3 noise regimes) and generates results and plots. Depending on hardware, this may take **30 minutes to several hours**.

### Running Specific Experiments

#### All Experiments with a Single Noise Regime
To run all filters and trajectories under a specific noise regime:

- **Baseline (Gaussian) noise:**
  ```matlab
  run_baseline
  ```
  Runs: KF, EKF, Robust KF, Robust EKF on Circular, Figure-8, Spiral (12 configs).

- **Correlated noise with ρ = 0.7:**
  ```matlab
  run_all_experiments_single('correlated_rho0.7', @() params_correlated(0.7))
  ```
  Runs: KF, EKF, Robust KF, Robust EKF on Circular, Figure-8, Spiral (12 configs).

- **All correlated noise sweeps (ρ ∈ {0.3, 0.5, 0.7, 0.9}):**
  ```matlab
  run_correlated
  ```
  Runs: 16 configurations (4 rho values × 4 filters) on each trajectory.

- **All heavy-tail noise sweeps:**
  ```matlab
  run_heavytail
  ```
  Runs: combinations of outlier rate and scale for all filters and trajectories.

- **Robustness parameter sweep (Delta & Lambda):**
  ```matlab
  run_robust
  ```
  Sweeps the Markov confidence threshold (delta) and outlier scale (lambda) on heavy-tail scenarios.
  Generates a 3×3 grid of experiments (3 delta values × 3 lambda values) to analyze robustness sensitivity.
  Results saved to `results/robust_sweep/` and plots to `plots/robust_sweep/`.

#### Single Filter on Single Trajectory
To run a specific filter on a specific trajectory and noise regime, use the `ExperimentRunner` class directly:

```matlab
runner = ExperimentRunner();
runner.run_experiment('EKF', 'Circular', @() params_baseline(), 'baseline_gaussian_Circular_EKF');
```

**Parameters:**
- Filter name: `'KF'`, `'EKF'`, `'RobustKF'`, or `'RobustEKF'`
- Trajectory: `'Circular'`, `'Figure8'`, `'Spiral'`, or `'HighCurvature'`
- Parameter function: `@() params_baseline()`, `@() params_correlated(rho)`, or `@() params_heavytail(pi, lambda)`
- Result label: Descriptive string for the output file name

### Tuning Robust Filter Parameters

The robust filters (RobustKF and RobustEKF) provide configurable parameters for fine-tuning robustness behavior. You can customize these when creating the `ExperimentRunner`:

```matlab
% Define custom robust configuration
robust_config = struct();
robust_config.delta = 1e-2;              % Markov outlier threshold (lower = more aggressive)
robust_config.buffer_size = 30;          % Chebyshev window size (larger = slower response)
robust_config.inflation_cap = 4.0;       % Max covariance inflation factor
robust_config.inflation_rate = 1.1;      % Rate of inflation per trigger (faster response)
robust_config.deflation_rate = 0.98;     % Rate of deflation decay
robust_config.markov_inflation = 200;    % How much to inflate R when outlier detected

% Pass configuration to ExperimentRunner
dynamics = RobotDynamics(params.dt, params.Q);
sensors = {gps_sensor, odom_sensor};
runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params, ...
    'Circular', robust_config);

% Run with custom parameters
results = runner.run('RobustEKF');
```

**Tuning Guidelines:**

| Scenario | Recommendation |
|----------|---|
| **Heavy outliers** | Lower $\delta$ (e.g., $10^{-4}$), increase `markov_inflation` (e.g., 500) |
| **Persistent noise mismatch** | Increase `buffer_size` (e.g., 40), higher `inflation_cap` (e.g., 5.0) |
| **Correlated noise** | Larger `inflation_rate` (e.g., 1.15) for faster adaptation |
| **Gaussian baseline (good conditions)** | Use default parameters, minimal robustness needed |

### Understanding the Output

After running experiments:

1. **Result Files:** Stored in `results/<regime>/<trajectory>/` (e.g., `results/baseline/Circular/baseline_gaussian_Circular_EKF.mat`)
   - Contains: `results` struct with fields:
     - `estimate`: Estimated state trajectory $\hat{x}_{1:T}$
     - `covariance`: Estimated covariance $\hat{P}_{1:T}$
     - `true_state`: Ground truth $x_{1:T}^{\text{true}}$ (simulated)
     - `nees`: NEES time series
     - `nis`: NIS time series
     - `mse`: Mean squared error by step

2. **Plot Files:** Stored in `plots/<regime>/<trajectory>/` (e.g., `plots/baseline/Circular/baseline_gaussian_Circular_EKF.png`)
   - Shows 4 subplots:
     - **Top-left:** True trajectory (blue) vs. estimated trajectory (red) with confidence ellipses.
     - **Top-right:** Position error over time.
     - **Bottom-left:** NEES consistency (mean and bounds).
     - **Bottom-right:** NIS consistency (mean and bounds).

3. **Generated Plots:** Automatically generated by `generate_comparison_plots.m` after each scenario run.

### Troubleshooting

**Q: Experiments run slowly or memory usage is high.**
- Reduce `num_trials` in the `params_*` files (e.g., from 500 to 100).
- Run a single scenario instead of all 36.

**Q: MATLAB runs out of memory.**
- Reduce the number of trials or trajectory length.
- Close other applications.

**Q: Plots are not generated.**
- Ensure `plots/` directories exist (created automatically).
- Check that `generate_comparison_plots.m` is called after each scenario.

**Q: Results show only zeros or NaN values.**
- Verify that sensors are correctly initialized (check `CombinedSensor.m`).
- Ensure parameter files are being loaded correctly (check `setup_paths` output).

### Advanced: Customizing Experiments

To modify experiments (e.g., change noise parameters or trajectory), edit the parameter files in `utils/`:

- `params_baseline.m`: Gaussian noise settings
- `params_correlated.m`: Correlated noise settings (modify correlation function)
- `params_heavytail.m`: Heavy-tail mixture settings
- `TrajectoryGenerator.m`: Reference trajectory definitions

After editing, re-run the experiment script. Existing result files with the same label will not be overwritten; delete them manually if you want to recompute.

[Return to Table of Contents](#table-of-contents)

---

## 10. Video Guide

Videos provide an interactive replay of filter estimates over the reference trajectory, visualizing the estimated path, true path, covariance ellipses, and velocity direction.

### Generating Videos

1. **Run the simulation first** to generate result `.mat` files (see "How to Run").

2. **Configure the video script:**
   Open `scripts/run_video_case.m` and set your desired parameters in the **USER CONFIGURATION** section:
   
   ```matlab
   % ========== USER CONFIGURATION ==========
   
   % SCENARIO_LABEL options:
   % Baseline: 'baseline_gaussian'
   % Correlated: 'correlated_rho0.3', 'correlated_rho0.5', 'correlated_rho0.7', 'correlated_rho0.9'
   % Heavy-tail: 'heavytail_pi0.01_lambda10', 'heavytail_pi0.05_lambda10', 'heavytail_pi0.1_lambda10'
   %             (change lambda to 5, 10, 20, etc. as needed)
   SCENARIO_LABEL = 'baseline_gaussian';
   
   % FILTER_TYPE options: 'KF', 'EKF', 'RobustKF', 'RobustEKF'
   FILTER_TYPE = 'RobustEKF';
   
   % TRAJECTORY options: 'Circular', 'Figure8', 'Spiral', 'HighCurvature'
   TRAJECTORY = 'Spiral';
   ```
   
   The results file path is **automatically determined** from these three variables.

3. **Run the video script:**
   ```matlab
   run_video_case
   ```

   The video is automatically saved to `video_exports/<scenario_label>_<trajectory>_<filter_type>.mp4`.

### Batch Video Generation (All Filters)

To generate videos for all 4 filters at once without editing code each time, use the batch video runner:

```matlab
% Mode 1: Generate 4 videos (one per filter) for a specific trajectory
VideoBatchRunner.generateVideosForAllFilters('baseline_gaussian', 'Spiral');

% Mode 2: Generate 16 videos (4 filters × 4 trajectories) for a scenario
VideoBatchRunner.generateVideosForScenario('baseline_gaussian');
```

Or use the convenience script:

```matlab
% Edit run_video_batch.m to set MODE, SCENARIO_LABEL, and TRAJECTORY
% Then run:
run_video_batch
```

This generates videos comparing all filters on the same scenario and trajectory, making it easy to visually compare performance.

### Video Contents

Each frame displays:
- **Green dotted curve:** True reference trajectory (full path for context)
- **Red solid curve:** Estimated trajectory (grows as simulation progresses)
- **Red circle:** Current estimated vehicle position
- **Blue arrow:** Vehicle heading direction

The video plays at 20 fps for fast, clear visualization of filter tracking performance.

### Output Location

Videos are saved to:
```
video_exports/<scenario_label>_<trajectory>_<filter_type>.mp4
```

### Caching

Videos are not overwritten if they already exist. Delete the video file manually if you want to regenerate it.

[Return to Table of Contents](#table-of-contents)
