# UAV OMMPC controller and ROS 2 / PX4 bridge

ROS 2 Jazzy **C++17 / rclcpp** communication and control nodes for PX4 v1.17. `ommpc_node` consumes bridged odometry/status and upper-level reference trajectories, solves OMMPC, and publishes thrust acceleration/body rates to `px4_bridge`. The bridge relays telemetry and translates commands to PX4 offboard inputs. Existing bridge topics, QoS and startup parameters are unchanged.

`uav_ommpc_controller` supplies standalone C++17/Eigen mathematics, nonlinear single-step prediction, discrete local-error linearization and a condensed OMMPC optimizer using OSQP. `uav_ommpc_ros/ommpc_node` connects that library to ROS2 and the existing PX4 bridge. The standalone tracking executable still exercises only the nonlinear mathematical model.

## OMMPC mathematics (no ROS dependency)

Header: `uav_ommpc_controller/include/uav_ommpc_controller/so3_math.hpp`.
These mathematical functions are inline, use fixed-size `Eigen::Vector3d` / `Eigen::Matrix3d`, and operate in double precision. The mathematical headers require only Eigen; the compiled MPC library additionally embeds OSQP. GoogleTest is required only when building tests.

### Definitions and conventions

`SO(3)` is the group of rotation matrices (`R.transpose() * R = I`, `det(R) = 1`); `so(3)` is its Lie algebra of skew-symmetric matrices. A rotation vector `phi = angle * axis` belongs to R³ and is **not** an Euler-angle vector. Rotations are right-handed and active, act on column vectors, and use radians.

Namespaces below are relative to `uav_ommpc_controller`:

| API | Formula / purpose |
| --- | --- |
| `r3::boxplus(x, delta)` | `x + delta` for Euclidean position/velocity components |
| `r3::boxminus(x, reference)` | `x - reference`, actual minus reference |
| `so3::hat(v)` | `[v]×`, with `[v]× w = v × w` |
| `so3::vee(S)` | Inverse of `hat`; input must be skew-symmetric |
| `so3::exp(phi)` | `I + sin(theta)/theta * S + (1-cos(theta))/theta² * S²`, where `S = hat(phi)`, `theta = norm(phi)` |
| `so3::log(R)` | Principal rotation vector, norm in `[0, pi]` |
| `so3::boxplus(R, delta)` | `R * Exp(delta)`, right/body-local perturbation |
| `so3::boxminus(R, R_ref)` | `Log(R_ref.transpose() * R)`, actual relative to reference |
| `so3::left_jacobian(phi)` | `J_l = I + (1-cos(theta))/theta² * S + (theta-sin(theta))/theta³ * S²` |
| `so3::right_jacobian(phi)` | `J_r(phi) = J_l(-phi) = J_l(phi).transpose()` |
| `so3::right_jacobian_inverse(phi)` | Derivative of `Log(Exp(phi)*Exp(delta))` at zero delta, with stable small-angle coefficients |

The Jacobians distinguish additive rotation-vector changes from local rotational increments:

```text
Exp(phi + d) = Exp(J_l(phi) * d) * Exp(phi) + O(norm(d)^2)
            = Exp(phi) * Exp(J_r(phi) * d) + O(norm(d)^2)
```

For a body-to-world rotation, body angular velocity `omega_body` advances attitude as
`R_next = boxplus(R, omega_body * dt)`. This is exact for constant body angular velocity over the interval. No NED/ENU or FRD/FLU conversion is performed here.

For `norm(phi) < 1e-3`, `exp` and the Jacobians use Taylor coefficients through `theta^4` to avoid division by zero and cancellation. Away from zero, `1-cos(theta)` is evaluated as `2*sin(theta/2)^2`. `log` uses Eigen's quaternion-based `AngleAxisd` extraction to avoid dividing by `sin(theta)` near pi. Unlike the reference's first-order tiny-angle exponential, this implementation retains the quadratic Rodrigues term.

**Domain:** inputs must be finite and rotation matrices must already be in SO(3); `vee` expects so(3). These primitives do not validate, normalize or project invalid inputs. Rotation vectors are intended for ordinary control-scale angles, not extreme floating-point magnitudes. `log(exp(phi)) == phi` is a local identity for `norm(phi) < pi`, not a global one. At exactly pi the axis sign is ambiguous, and the principal logarithm is discontinuous across that branch cut. Do not use `I + hat(delta)` as a finite rotation matrix; it is only the first-order approximation.

### Example

```cpp
#include <uav_ommpc_controller/so3_math.hpp>

namespace so3 = uav_ommpc_controller::so3;
const Eigen::Matrix3d R_ref = so3::exp(Eigen::Vector3d(0.2, -0.1, 0.3));
const Eigen::Vector3d delta(0.001, -0.002, 0.003);
const Eigen::Matrix3d R = so3::boxplus(R_ref, delta);
const Eigen::Vector3d error = so3::boxminus(R, R_ref);  // recovers delta
const Eigen::Vector3d omega_body(0.1, -0.2, 0.3);       // rad/s
const Eigen::Matrix3d R_next = so3::boxplus(R, omega_body * 0.01);
```

### Build and test

Standalone, without sourcing ROS (requires a C/C++ toolchain, CMake, Git, `libeigen3-dev`, and `libgtest-dev` for tests). The first configure fetches pinned OSQP/QDLDL sources into the build directory; no system-wide solver installation or sudo is needed:

```bash
cd ~/uav_control_ws
cmake -S src/uav-ommpc-controller/uav_ommpc_controller -B so3-math-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build so3-math-build -j2
ctest --test-dir so3-math-build --output-on-failure
```

Or integrate the pure-CMake package into the existing colcon workspace:

```bash
cd ~/uav_control_ws
colcon build --packages-select uav_ommpc_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select uav_ommpc_controller
colcon test-result --verbose --test-result-base build/uav_ommpc_controller
```

An independent CMake consumer can set `CMAKE_PREFIX_PATH` to
`~/uav_control_ws/install/uav_ommpc_controller`, then use:

```cmake
find_package(uav_ommpc_controller CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE uav_ommpc_controller::uav_ommpc_controller)
```

The exported target now links the compiled `libuav_ommpc_controller.so`; mathematical headers remain usable without linking it if only their inline functions are needed. OSQP is linked statically inside that shared library, so consumers do not need OSQP headers or a separate OSQP package. Solver/ordering license notices install under `share/uav_ommpc_controller/licenses`.

`cmake/FetchOsqp.cmake` pins OSQP 0.6.3 commit `0dd00a578cf1c2691c5c379965d504c75bf6cfad` and its QDLDL submodule (`7d16b70a10a152682204d745d814b6eb63dc5cd2`). For offline builds or reuse between standalone/colcon builds, pass `-DFETCHCONTENT_SOURCE_DIR_OSQP=/absolute/path/to/that/osqp/source` with its submodule populated. The current Jetson colcon build reuses `~/uav_control_ws/so3-math-build/_deps/osqp-src`. OSQP/QDLDL emit upstream CMake minimum-version deprecation warnings; these do not prevent the observed builds and are not suppressed.

SO(3)/R³ coverage includes hat/vee and cross products, Euclidean errors, right-handed quarter turns, zero/tiny rotations, the Taylor-branch boundary, pi and both sides of its branch cut, 500 seeded rotation round trips, right-local error direction, finite-difference Jacobians, the inverse right Jacobian, vector-action derivatives, and 10,000 consecutive body-rate updates. These tests are included in the current colcon verification below.

An independent Release-mode CMake consumer also built against the installed package without sourcing ROS. With Eigen heap allocation disabled and assertions enabled, it recovered `[0.001, -0.002, 0.003]` rad with error `2.76e-17`; 1,000 body-rate updates differed from the closed-form rotation by `6.72e-14` in matrix Frobenius norm. The temporary consumer was removed after verification.

### Mathematical sources

- G. Lu, W. Xu and F. Zhang, [On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems](https://doi.org/10.1109/TIE.2022.3212397), IEEE TIE 70(9), 9192–9202, 2023.
- The accessible author [preprint](https://arxiv.org/html/2106.15233v1), titled *Model Predictive Control for Trajectory Tracking on Differentiable Manifolds*: Section III-D defines Euclidean and Lie-group boxplus/boxminus; Section V defines actual-minus-reference error states. These section references are to the preprint, not verified journal-version equation numbers. Rodrigues' formula is written explicitly above with the quadratic term divided by `theta²`.
- Engineering reference: [`refact-decouple/ommpc_core/include/ommpc_core/so3_math.h`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_core/include/ommpc_core/so3_math.h) and its [quadrotor derivation](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/OMMPC_derivation.pdf). This implementation uses the same hat/vee and left-Jacobian conventions, with explicit local-error operations and small-angle tests.

## Nonlinear single-step dynamics

Header: `uav_ommpc_controller/include/uav_ommpc_controller/dynamic.hpp`.
Namespace: `uav_ommpc_controller::dynamics`. This is a header-only C++17/Eigen implementation using the existing `r3::boxplus` and `so3::boxplus`; no ROS dependency or dynamic-size matrices.

### State, input and frames

| Type / field | Meaning |
| --- | --- |
| `State::p` | World NED position, metres |
| `State::v` | World NED velocity, m/s; not body-frame velocity |
| `State::R` | Body FRD to world NED rotation matrix |
| `ControlInput::thrust_acceleration` | Nonnegative total thrust divided by mass, m/s², acting along body −Z |
| `ControlInput::body_rates` | Body FRD angular velocity `(wx, wy, wz)`, rad/s |
| `predict_step(state, input, dt, gravity = kGravity)` | Returns the next `State`; `dt` is seconds, `gravity` is a positive scalar in m/s² |

The state is `x = (p, v, R)` and the input is `u = (a_T, omega_body)`. Angular velocity is held as an input during a step: there is no prediction of `omega_next`, torque input or inertia model. If supplying measured angular velocity instead of a command, the same constant-rate assumption applies.

Thrust is **not** a force in newtons or a normalized throttle. Given a force `T` and mass `m`, the caller supplies `a_T = T / m`. It includes the thrust needed to counter gravity; it is not net vertical acceleration. The default `kGravity = 9.80665` matches the existing bridge; pass `9.81` explicitly to use the preprint's gravity value.

### Model and discretization

With `e3 = [0, 0, 1]ᵀ` and gravity directed down in NED:

```text
a_k     = [0, 0, gravity]ᵀ - a_T * R_k * e3
p_{k+1} = p_k + dt * v_k
v_{k+1} = v_k + dt * a_k
R_{k+1} = R_k * Exp(dt * omega_body)
```

Both translational updates use the **current** state. This is forward Euler on position/velocity combined with the SO(3) exponential for rotation, following the paper's canonical discrete model. Do not insert `0.5 * a * dt²`, use the updated velocity for position, or use the updated attitude for thrust acceleration without also changing the discretization and subsequent linearization. Rotation is exact for constant body rates over the step; translation remains first-order, not an exact continuous-time integration during changing attitude.

Inputs must be finite, `R` must belong to SO(3), `dt >= 0`, `thrust_acceleration >= 0`, and `gravity > 0`. Validation belongs to the caller, consistent with `so3_math.hpp`; the function does not clamp inputs or repair matrices. It models no drag, wind, motor lag, saturation, body-rate tracking lag or rigid-body torque dynamics, and performs no ENU/FLU conversion.

### Usage

```cpp
#include <uav_ommpc_controller/dynamic.hpp>

namespace dynamics = uav_ommpc_controller::dynamics;
const dynamics::State state{p, v, R};
const dynamics::ControlInput input{thrust_acceleration, w_body};
const dynamics::State next = dynamics::predict_step(state, input, 0.01);
// next.p, next.v, next.R; w_body remains a held input, not a predicted state.
```

The equations follow the author [preprint](https://arxiv.org/html/2106.15233v1), Section IV-A and Section VII-A2/A3, equations (32)–(33) in that preprint, not verified journal-version numbering. The engineering reference is [`OMMPCControllerCore::setStateMatricesAndBounds`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_core/src/controller_core.cpp): its `dt * I` position/velocity block and `Exp(-omega * dt)` attitude-error block are consistent with this discretization. That reference branch uses body +Z thrust with upward-world conventions, so its thrust signs are **not** copied into this NED/FRD implementation.

### Verification

All 8 dynamics tests pass on Jetson in standalone Release and colcon builds. They cover zero time, hover at different yaw angles, free fall, excess thrust, tilted thrust evaluated at the current attitude, right-multiplied body-rate rotation with time scaling, supplied gravity, and 1,000 chained steps checked against discrete ballistics and closed-form attitude.

For dynamics tests alone:

```bash
ctest --test-dir ~/uav_control_ws/so3-math-build -R '^Dynamics\.' --output-on-failure
```

An independent program compiled directly against the installed header without sourcing ROS, with Eigen heap allocation disabled and assertions enabled. Across 1,000 steps of 2 ms, yawing hover had zero position/velocity drift and a rotation-matrix error of `1.48e-14`. A tilted, constant-thrust trajectory matched its discrete closed-form solution with position error `6.31e-14 m` and velocity error `1.21e-13 m/s`. This verifies the mathematical predictor, not physical flight or PX4 integration. The temporary program was removed after verification.

## Affine local-error linearization

`include/uav_ommpc_controller/linearization.hpp` keeps the actual-minus-reference error
`delta_x = [p-p_ref, v-v_ref, Log(R_ref.transpose()*R)]` and input error
`delta_u = [a_T-a_T_ref, omega-omega_ref]`. Rotation errors are local rotation vectors, not Euler angles.

`linearize(reference, input, next_reference, dt, gravity)` now accepts an independently sampled next reference. It returns fixed-size `A` (9×9), `B` (9×4), and `d` (9):

```text
nominal_next = dynamics::predict_step(reference, input, dt, gravity)
d = state_error(nominal_next, next_reference)
delta_x_next = d + A*delta_x + B*delta_u + O(norm([delta_x,delta_u])^2)
```

Let `phi = dt*omega_ref` and `e3=[0,0,1]`. The original discrete derivatives are
`A0 = [[I,dt*I,0], [0,I,dt*a_T*R*hat(e3)], [0,0,Exp(-phi)]]` and
`B0 = [[0,0], [-dt*R*e3,0], [0,dt*J_r(phi)]]`.
The new reference chart requires:

```text
T = diag(I3, I3, J_r_inverse(d_rotation))
A = T*A0
B = T*B0
```

This follows `Log(Exp(d_rotation)*Exp(eta)) = d_rotation + J_r_inverse(d_rotation)*eta + O(norm(eta)^2)`. Merely adding `d` while retaining the old rotation blocks is not the correct first-order derivative for a nonzero attitude defect.

`so3::right_jacobian_inverse(phi)` uses
`I + 0.5*hat(phi) + c*hat(phi)^2`, with
`c=(1-theta/2*cot(theta/2))/theta^2`; near zero it uses `1/12 + theta^2/720 + theta^4/30240`. The principal Log chart must stay away from pi.

```cpp
namespace lin = uav_ommpc_controller::linearization;
const auto model = lin::linearize(reference, reference_input, next_reference, dt);
const auto dx = lin::state_error(actual, reference);
lin::InputError du;
du << input.thrust_acceleration - reference_input.thrust_acceleration,
  input.body_rates - reference_input.body_rates;
const auto predicted = lin::apply_state_error(next_reference, model.predict(dx, du));
```

The obsolete three-argument linearization API is removed, not kept as an alias. Gravity must match the nonlinear model and reference sampler. Zero defect reduces to the original discrete model. `dynamic.hpp` is deliberately unchanged: retaining the residual does **not** make the forward-Euler model an exact continuous-time integrator.

## Continuous polynomial references

`include/uav_ommpc_controller/polynomial_reference.hpp` and `src/polynomial_reference.cpp` implement the ROS-independent `reference::PolynomialReference` sampler.

- A trajectory contains 1–100 segments. Each segment has a finite positive duration and separate `x`, `y`, `z`, `yaw` arrays, each containing 1–10 coefficients (degree 0–9).
- Coefficients are **ascending powers of normalized segment time** `s=(t-segment_start)/duration`. Position is NED metres; yaw is unwrapped radians. Different axes may have different polynomial degrees.
- Simultaneous Horner evaluation returns analytic position, velocity, acceleration and jerk. The r-th derivative is divided by `duration^r`; no finite differences are used to manufacture discrete consistency.
- Internal knots require C3 position and C1 yaw continuity within absolute `1e-6` norms. The sampler selects the right-hand segment at knots. Wrapped yaw discontinuities must be unwrapped by the planner.
- Negative/nonfinite time, invalid coefficients/durations, nonfinite derivatives and discontinuities are rejected. Invalid construction throws `std::invalid_argument`; invalid sample evaluation returns false.

Flatness mapping uses `force=g*e3-a`, `a_T=norm(force)`, `b3=force/a_T`,
`ypsi=[-sin(yaw),cos(yaw),0]`, `b1=normalize(ypsi cross b3)`, `b2=b3 cross b1`.
`R=[b1,b2,b3]`; analytic derivatives from jerk and yaw rate give
`omega=vee(skew_part(R.transpose()*R_dot))`. This retains Euler heading in the **upright** domain. Force norms <=`1e-6`, `b3.z()<=1e-3`, degenerate heading axes or nonfinite results are rejected; no fake attitude or force clipping is substituted. Actuator limits are checked by the controller, not hidden inside the sampler.

`hold_final=false` forbids extrapolation after the final segment. `hold_final=true` is accepted only when terminal velocity, acceleration, jerk and yaw rate are all stationary within `1e-6`; it then extends that terminal pose with hover. A moving endpoint cannot silently become a hover. Receipt freshness at the ROS layer still applies during a hold.

```cpp
namespace ref = uav_ommpc_controller::reference;
ref::Segment hover{1.0, {0.0}, {0.0}, {-2.0}, {0.0}};
ref::PolynomialReference trajectory({hover}, true);
std::vector<uav_ommpc_controller::dynamics::State> states(21);
std::vector<uav_ommpc_controller::dynamics::ControlInput> inputs(20);
if (trajectory.sample_horizon(elapsed_seconds, 0.02, states, inputs)) {
  const auto result = controller.solve(actual, states, inputs);
}
```

`sample_flat` exposes p/v/a/j/yaw/yaw-rate; `sample` returns one full state/input. Individual outputs remain unchanged on failure. `sample_horizon` requires pre-sized N+1/N buffers and does not allocate or resize; callers discard partially filled buffers on failure. Without terminal hold, the whole horizon must fit within the trajectory.


## Condensed on-manifold MPC

Implementation: `uav_ommpc_controller/src/ommpc.cpp`; API: `include/uav_ommpc_controller/ommpc.hpp`.
Namespace: `uav_ommpc_controller::ommpc`. `Controller` owns one persistent OSQP workspace and fixed-horizon buffers. Construct it once per control loop; it is noncopyable and not thread-safe. Settings are copied at construction, not changed mid-solve.

### Reference and error contract

`solve(actual, reference, feedforward)` takes one measured/predicted `dynamics::State`, **N+1 reference states** (`x_ref,0` through `x_ref,N`) and **N feedforward inputs** (`u_ref,0` through `u_ref,N-1`), all at `Settings::dt`.

- Frames, units, right-local attitude error and actual-minus-reference signs match the earlier modules.
- Reference defects are retained in the affine model. Defaults allow position defect norm up to `0.05 m`, velocity defect norm up to `0.25 m/s`, and attitude defect norm up to `0.1 rad` per step, configured by `max_position_defect`, `max_velocity_defect`, `max_attitude_defect`.
- Invalid dimensions/nonfinite states/rotations or out-of-box feedforward remain rejected. A defect exceeding its configured local-model domain returns `ReferenceOutOfDomain`; the obsolete `ReferenceMismatch` status is removed.
- The limits are conservative initial domain guards, not proof of aircraft feasibility. They must remain finite/positive, and the attitude limit must be below pi minus `1e-3`. Do not increase them to hide wrong frames, timestamps or discontinuous references.
- The pure solver still receives full state/input vectors, not polynomial coefficients. The sampler supplies them from analytic derivatives, so normal Euler discretization defects are expected rather than rejected at `1e-6`.

This remains local, one-linearization-per-cycle MPC, without nonlinear SQP iterations. Affine defects do not remove model discretization error or make an actuator-infeasible path trackable; retime the trajectory instead of clipping its analytic derivatives.

### Condensation and QP

From each reference state/input/next-state, obtain `A_k, B_k, d_k` using `linearization::linearize`. Stack
`delta_X = [delta_x1; ...; delta_xN]` and `delta_U = [delta_u0; ...; delta_uN-1]`.
The recursive coefficients are:

```text
H_0 = A_0                         M_0,0 = B_0
H_k = A_k * H_{k-1}               M_k,j = A_k * M_{k-1,j}   (j < k)
                                  M_k,k = B_k               (j > k: zero)
s_0 = 0; s_{k+1} = A_k*s_k + d_k; D = [s_1; ...; s_N]
delta_X = H * delta_x0 + M * delta_U + D
```

The stage cost penalizes state errors `x1..xN-1`, terminal error `xN`, and input errors `u0..uN-1`. The cost of the fixed initial state `x0` is constant and omitted:

```text
Qbar = diag(Q, ..., Q, Q_terminal)     Rbar = diag(R, ..., R)
P = 2 * (M.transpose() * Qbar * M + Rbar)
q = 2 * M.transpose() * Qbar * (H * delta_x0 + D)

minimize  0.5 * delta_U.transpose() * P * delta_U + q.transpose() * delta_U
subject to  u_min - u_ref,k <= delta_u_k <= u_max - u_ref,k,  k = 0..N-1
```

Only **4N decision variables** remain; state equalities are eliminated rather than passed to OSQP. Positive input weights keep the Hessian positive definite even if state weights are zero. `problem()` exposes `M`, `H`, `D`, `P`, `q`, `lower`, `upper`. The Hessian formula is unchanged, but its coefficients change when the transported A/B change. `input_errors()` and `predicted_errors()` expose the optimized stacked errors, owned by the controller and overwritten by the next call.

OSQP 0.6.3 receives the fixed upper-triangular Hessian sparsity pattern and identity constraint matrix. Every cycle updates numerical coefficients, gradient and bounds without recreating the workspace. Warm starts shift the previous **absolute** input sequence, then subtract the new feedforward and project only the initial guess into the new box; stale input-error coordinates are not reused. Dual guesses start at zero. The returned solution is not clipped.

For Hessian construction, symmetric column inner products skip structurally zero causal prefixes. This avoids the temporary heap scratch space observed in Eigen's general matrix-multiply path. The matrix/vector storage is allocated once at construction; a runtime Eigen-allocation guard passed complete solves at horizons 1, 20 and 100. Polishing is disabled to avoid its additional factorization/allocations. This is not a hard-real-time proof or an instrumentation of every allocator inside the external solver.

### Initial tuning and constraints

| Setting | Default |
| --- | --- |
| Horizon / step | `N=20`, `dt=0.02 s` (0.4 s prediction window, intended 50 Hz control) |
| Position weights, N/E/D | `[1000, 1000, 800]` |
| Velocity weights | `[20, 20, 20]` |
| Local attitude weights | `[40, 40, 40]` |
| Terminal state weights | `2 *` the default state-weight vector |
| Input-error weights, thrust / body rates | `[0.5, 1.2, 1.2, 0.6]` |
| Total thrust acceleration | `[1, 12] m/s²` |
| Body angular rates | `±[3, 3, 1.5] rad/s` |
| OSQP iteration limit | `400` |
| OSQP absolute / relative tolerances | `1e-5 / 1e-5` |
| Maximum accepted input-bound violation | `1e-4` in the corresponding input units |
| Position / velocity / attitude defect domain | `0.05 m / 0.25 m/s / 0.1 rad` per step |

State/input weights start from the reference repository's 20-step tuning, with a doubled terminal penalty and no horizon discount. The terminal vector is an independent settings field: modifying `state_weights` later does not automatically change `terminal_weights`; set both when retuning. Supported horizons are 1–100, and positive finite `dt`/gravity and positive finite input weights are required. Invalid settings throw `std::invalid_argument`; OSQP setup failure throws `std::runtime_error`.

The rate bounds match the bridge defaults. The thrust ceiling is deliberately below the bridge's approximate `9.80665 * 0.9 / 0.728744 ≈ 12.11 m/s²` input ceiling for the previously calibrated x500, not the reference implementation's `30 m/s²`. **These are initial model/controller settings, not validated aircraft operating limits.** Changing vehicle calibration, bridge limits or timing requires consistent retuning. Faster trajectories must remain dynamically feasible under these bounds; raising limits is not a substitute for available vehicle thrust or bandwidth.

There are no position, obstacle, tilt, velocity or input-slew constraints; no disturbance estimator, rotor drag or motor/rate-loop lag. The cost does not by itself enforce those omitted physical constraints.

### Result and usage

Only `OSQP_SOLVED`, finite outputs, and original-box violations no greater than `1e-4` yield `Status::Solved` and a populated `Result::command`. `OSQP_SOLVED_INACCURATE`, iteration limits and update/solve failures are rejected. `Result` also provides the native solver status, iterations and measured bound violation.

On any rejected call, `command` is empty, both solution vectors become NaN, and the previous warm start is invalidated. There is **no stale command, zero-thrust fallback, retry or automatic flight action**. The caller must handle failure and telemetry validity. `problem()` is only valid for inspection after a successful call.

```cpp
#include <uav_ommpc_controller/ommpc.hpp>

namespace mpc = uav_ommpc_controller::ommpc;
mpc::Settings settings;
mpc::Controller controller(settings);  // Outside the repeated control loop.

// actual: dynamics::State
// reference: std::vector<dynamics::State>, size settings.horizon + 1
// feedforward: std::vector<dynamics::ControlInput>, size settings.horizon
const auto result = controller.solve(actual, reference, feedforward);
if (result.command)
{
  const auto &u = *result.command;  // u_ref,0 + optimized delta_u0
  // u.thrust_acceleration [m/s²], u.body_rates [rad/s, body FRD].
  // This library does not publish them to PX4.
}
// Empty command: application-specific safety handling is required.
```

### Verification and reproducible tracking example

The polynomial cutover passed **73 colcon-reported tests** (zero errors/failures/skips), plus **8 Python publisher tests**. Coverage includes nonzero rotational-defect finite differences, second-order affine residual convergence, condensed D/gradient/objective comparisons, zero-defect regression, polynomial derivative scaling and continuity, flatness body-rate derivatives, stationary holds, malformed inputs and existing solver/bridge failure handling.

```bash
ctest --test-dir ~/uav_control_ws/build/uav_ommpc_controller --output-on-failure
~/uav_control_ws/install/uav_ommpc_controller/bin/ommpc_tracking
# Rebuilding the standalone CMake project provides the same executable there.
# Disable building this example with -DBUILD_EXAMPLES=OFF.
```

`ommpc_tracking` now creates degree-7 C3 Hermite polynomial pieces approximating a 5 m radius, 5 m/s circle, and evaluates their **analytic derivatives**. It runs two 12-second plants: unchanged Euler dynamics, and continuous zero-order-held dynamics integrated with RK4 at dt/20 (1 ms) and exact constant-body-rate attitude evolution. Both start with position error `[0.2,-0.15,0.1] m` plus velocity/attitude errors. The same 0.15 m post-2-second position-RMSE acceptance remains.

The sampled references have genuine nonzero defects: maximum position/velocity defects were `0.000999989 m` / `0.000999989 m/s`, and attitude defect was `0.000105718 rad`. These would have failed the removed strict `1e-6` test.

Observed installed Release executable on Jetson:

| Measurement | Euler plant | Continuous RK4 plant |
| --- | ---: | ---: |
| Completed solves / failures | 600 / 0 | 600 / 0 |
| Position RMSE, entire run | 0.051798165 m | 0.050700618 m |
| Position RMSE, after 2 s | 0.004218605 m | 0.014361560 m |
| Velocity RMSE, after 2 s | 0.049483382 m/s | 0.014288840 m/s |
| Maximum position error, after 2 s | 0.004230877 m | 0.014416599 m |
| Maximum input-bound violation | 0.000000568 | 0.000003321 |
| Solve-call median | 0.984768 ms | 0.985984 ms |
| Solve-call p95 / maximum | 1.062403 / 1.731064 ms | 1.066243 / 1.559635 ms |
| Calls above 20 ms | 0 | 0 |

Timing covers `Controller::solve`, including affine linearization/condensation/OSQP, not ROS communication or polynomial sampling. A separate compile with Eigen allocation assertions enabled passed **combined horizon sampling and solving** for N=1,20,100 with nonzero D and no Eigen heap allocation during those calls. External solver allocation behavior and hard-real-time guarantees are not inferred from that check.

These are model-only results. In particular, adding reference defects does not eliminate Euler integration bias: the position/velocity tradeoff and the continuous-plant error are now visible instead of being hidden by forward-difference references. No new PX4/Gazebo flight or hardware validation was performed for this cutover.

The condensed formulation follows the author [preprint](https://arxiv.org/html/2106.15233v1), Section V-C, equations (25)–(30), using the existing actual-minus-reference error convention. The reference repository's [`mpc_wrapper.hpp`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_core/include/ommpc_core/mpc_wrapper.hpp) uses an uncondensed sparse QP; this implementation intentionally follows the paper's **condensed** form and reuses its OSQP API rather than copying the per-cycle setup/cleanup pattern. Baseline weights come from [`params.yaml`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_ros/config/params.yaml).

## ROS2 OMMPC node

`uav_ommpc_ros/src/ommpc_node.cpp` owns one controller and preallocated horizon buffers, using a single-threaded executor so subscription callbacks and the control timer cannot race.

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| Subscribe, from bridge | `/uav/state` | `px4_msgs/msg/VehicleOdometry` | best effort, volatile, depth 1 |
| Subscribe, from bridge | `/uav/status` | `px4_msgs/msg/VehicleStatus` | best effort, volatile, depth 1 |
| Subscribe, from planner | `/uav/reference_polynomial` | `uav_ommpc_interfaces/msg/PolynomialTrajectory` | reliable, volatile, depth 1 |
| Publish, to bridge | `/uav/control_input` | `uav_ommpc_interfaces/msg/ThrustRates` | reliable, volatile, depth 1 |

There is no second odometry topic: `/uav/state` already contains the PX4 odometry. The node publishes neither arm/mode commands nor OffboardControlMode directly. Ensure **only one controller publishes `/uav/control_input`**; the bridge has no arbitration between competing publishers. Standard ROS remapping can isolate multiple vehicles or rename these topics.

### Polynomial message, timing and renewal

This is a clean wire-interface cutover: the sampled `ReferenceState`/`ReferenceTrajectory` messages and `/uav/reference_trajectory` subscription are removed. There is no parallel legacy command source.

`PolynomialTrajectory.msg` contains a `std_msgs/Header`, `hold_final`, and up to 100 `PolynomialSegment` entries. Each segment contains `duration` and bounded arrays `x`, `y`, `z`, `yaw` (1–10 ascending normalized-time coefficients). There is no message dt: the node samples at its configured `dt`.

- `header.frame_id` must be `"ned"` and positions use the same local NED origin as odometry. ENU plans must be transformed upstream, not merely relabeled.
- `header.stamp` is the ROS time of the first segment. Republishing the same plan renews its receipt watchdog **without changing that stamp**. A shifted rolling window must update both its coefficients and its start stamp.
- New plans are validated/decoded once on receipt and cached immutably. A malformed replacement clears the previous plan; unsupported degree, discontinuous derivatives, nonfinite values and invalid terminal holds are rejected.
- Each accepted odometry packet records its **ROS reception time**. By default, the node pairs the unmodified state with references at reception time plus `k*dt`, without floor indexing. Optional `state_prediction_enabled` extrapolates measured position/attitude to solve start and samples the reference at that same anchor; see the peak-error audit below. With DDS synchronization enabled, both PX4 timestamp fields are translated to the agent/host clock, not raw boot time.
- Current time, the odometry reception anchor, steady-clock freshness and geometry validity are checked. Future plans wait; expired non-holding plans stop. A complete N-step horizon is required. An explicitly stationary hold may extend beyond the final segment while the receipt lease remains fresh.
- An odometry receipt stamp within **one control period** before the plan start is treated as plan time zero instead of being rejected. This absorbs publisher/subscriber clock skew and scheduling jitter without pretending to know the transport delay; anything older is still refused.
- Keep the existing default `reference_timeout=0.5 s` by periodically renewing the plan. A long geometric duration or `hold_final=true` is not permission to continue after losing the planner. Planner/controller ROS clocks must be consistent.
- Successful solves are still checked against the period deadline and renewed safety gates; ordinary asynchronous planner/control timing is not treated as a failed sample-grid transition.

Example construction for a reference-only planner:

```python
from uav_ommpc_interfaces.msg import PolynomialSegment, PolynomialTrajectory

msg = PolynomialTrajectory()
msg.header.frame_id = "ned"
msg.header.stamp = node.get_clock().now().to_msg()  # Set once for this plan.
msg.hold_final = True
msg.segments = [
    PolynomialSegment(duration=1.0, x=[north], y=[east], z=[down], yaw=[yaw_rad])
]
# Periodically publish msg on /uav/reference_polynomial without restamping it.
```

This is an explicit hover target, not automatic takeoff or origin selection. For moving paths use analytic polynomial coefficients; jerk and yaw rate supply the feedforward body rates. A nonstationary endpoint cannot request a terminal hold.

### State conversion and output gating

- Odometry pose frame must be `POSE_FRAME_NED`. Velocity may be `VELOCITY_FRAME_NED`, or `VELOCITY_FRAME_BODY_FRD`, in which case it is rotated by the body-to-world attitude. Arbitrary-heading FRD navigation frames and unknown frames are rejected; no ENU/FLU conversion is performed.
- Odometry position, velocity and quaternion must be finite. Quaternion norm must be within `1e-3` of one; accepted float32 PX4 quaternions are normalized. Polynomial attitude is computed from analytic acceleration/jerk/yaw instead of received reference quaternions.
- The installed PX4 message labels `quality` as unused; its default zero is **not** used to reject otherwise valid odometry. The pure p/v/R MPC does not include angular-rate states or covariance. Optional initial-state prediction uses finite measured body angular velocity; covariance remains unused.
- PX4 `timestamp_sample` must be positive, no greater than the message timestamp, and strictly advance before refreshing the odometry cache. Status stamps must also advance. The default control path does not compare message stamps with ROS wall time. With prediction enabled, the sample must be in the synchronized ROS clock domain and have age in `[0,state_prediction_max_age]` at control use; invalid/future/too-old samples stop publication, without clamping or a guessed clock offset. Duplicates/out-of-order packets do not refresh freshness. Restart the node after source-clock reset/backward movement.
- Fresh valid odometry, fresh reference receipt and fresh healthy vehicle status are all required. Vehicle status must report rotary-wing operation, not a VTOL transition, no failsafe and no termination mode. A fresh invalid odometry packet invalidates the cached state rather than continuing on its old value.
- A changed odometry `reset_counter` invalidates the cached reference. The planner must publish a new trajectory in the updated local frame; the node cannot determine whether the planner actually corrected its origin. A backward ROS clock jump also invalidates the reference.
- Malformed polynomial metadata/coefficients invalidate the old plan. Invalid geometry, unavailable full horizons, excessive dynamics defects, out-of-bound feedforward or solver failure produce **no control message**. No zero-thrust or last-solution fallback is generated.
- After solving, the node checks freshness and remaining horizon coverage again. A solve exceeding one timer period, an expired input, clock rollback or exhausted reference is discarded. Merely crossing a reference sample boundary during a timely solve is allowed: asynchronous planner/control phases must not suppress otherwise valid commands.

The node deliberately does **not** require armed/Offboard status: valid pre-Offboard inputs let the bridge prime its heartbeat. The unchanged bridge only forwards angular-rate setpoints while PX4 reports Offboard. Neither node requests arming or a flight mode.

Stopping node publication is **not an immediate revocation of the last bridge command**. The bridge can repeat it for up to its `command_timeout` (default 0.5 s), after which heartbeat/setpoints stop and PX4's configured Offboard-loss behavior applies. This existing failover delay remains unchanged; it is not an automatic controlled landing.

### Startup parameters and running

All controller/timeout parameters are **read-only after startup**. Restart the node to change them.

| Parameter | Default / meaning |
| --- | --- |
| `horizon`, `dt` | `20`, `0.02 s`; timer period equals prediction step |
| `gravity` | `9.80665 m/s²` |
| `odometry_timeout` | `0.2 s`, steady-clock age of last advancing valid sample |
| `status_timeout` | `1.5 s`, steady-clock age of last advancing status (PX4 may publish at 2 Hz) |
| `reference_timeout` | `0.5 s`, steady-clock age of latest accepted trajectory message |
| `state_weights`, `terminal_weights` | Nine-element diagonal weights; terminal default is twice the configured state weights |
| `input_weights` | Four-element vector, default `[0.5, 1.2, 1.2, 0.6]` |
| `input_min`, `input_max` | Four-element `[thrust, wx, wy, wz]` bounds; same defaults as the controller library |
| `max_iterations` | `400` |
| `absolute_tolerance`, `relative_tolerance` | `1e-5`, `1e-5` |
| `max_position_defect`, `max_velocity_defect`, `max_attitude_defect` | `0.05 m`, `0.25 m/s`, `0.1 rad` per-step local-model defect limits |

The node accepts dt in `[1e-6,1]` seconds for representable timer configuration; that is not a guarantee of achieving those rates. Keep the intended 0.02 s step unless timing, horizon, reference generation and vehicle limits are retuned together. Timeouts must be finite and positive. Keep MPC input bounds consistent with bridge limits and vehicle calibration.


[You have received this identical output 3 times. Re-reading 'ssh://jetson4/home/xerolab/uav_control_ws/src/uav-ommpc-controller/README.md:raw:1-450' will not change it — use a narrower selector (path:A-B), or proceed with the edit.]
After building and sourcing the underlays/overlay as below:

```bash
# Terminal 1: bridge, with the existing x500 calibration when using that simulator.
ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.728744

# Terminal 2: controller; waits silently with respect to control output until inputs are ready.
ros2 run uav_ommpc_ros ommpc_node --ros-args \
  -p horizon:=20 -p dt:=0.02 \
  -p odometry_timeout:=0.2 -p status_timeout:=1.5 -p reference_timeout:=0.5
```

The PX4/DDS transport and an upper-level trajectory publisher are separate prerequisites. The node does not start them. If using `use_sim_time`, odometry transport, planner and controller must share the intended clock; no control is produced before ROS time is positive. Never run the old test-flight controller concurrently with `ommpc_node`.

### Verification scope

The complete polynomial ROS graph passed using actual installed node/bridge processes, synthetic PX4 telemetry, ROS_DOMAIN_ID=173 and localhost-only discovery. Checks included nonzero acceleration/yaw references, a segment knot, pre-Offboard heartbeat priming, Offboard thrust/rate mapping, malformed coefficients/frames/durations, discontinuity rejection, invalid stationary holds, future/exhausted plans, reference lease expiry, invalid/frozen odometry, failsafe, estimator resets and a real OSQP iteration-limit failure. It observed 419 controller outputs, 704 downstream rates and zero VehicleCommands; logs are at `/tmp/polynomial-ros-7lxgzpyd`.

A second isolated run exercised the **actual migrated Python publisher** in all four modes through the C++ sampler/controller/bridge. All modes produced fresh commands. It also exercised SITL reference evaluation/CSV recording without entering the flight state machine, and verified that a backward ROS clock jump clears a still-fresh plan until a new plan receipt. No flight commands were sent; logs are at `/tmp/polynomial-ros-04djq4mc`. Temporary smoke programs were removed afterward.

The historical PX4/Gazebo flight records later in this README were obtained with the previous sampled-reference implementation; they are not a flight certification of the polynomial cutover.


## Build and environment

Dependencies on this Jetson: ROS 2 Jazzy (`/opt/ros/jazzy`), `px4_msgs` release/1.17 in `~/ros2_px4_ws`, PX4 v1.17.0 in `~/PX4-Autopilot`, Gazebo Harmonic and Micro XRCE-DDS Agent v2.4.3. Build the PX4 message workspace before this overlay. Message definitions must match PX4.

`uav_ommpc_ros` uses `ament_cmake`; `ommpc_node` links the standalone `uav_ommpc_controller` library in addition to ROS2, PX4 and project message dependencies. The interface package now generates timed reference messages using `std_msgs/Header`. When migrating an already-built Python workspace, remove its obsolete generated `build/uav_ommpc_ros` and `install/uav_ommpc_ros` directories before rebuilding; source the ROS/px4_msgs underlays, not the old project overlay.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_px4_ws
colcon build --symlink-install --packages-select px4_msgs
source ~/ros2_px4_ws/install/setup.bash
cd ~/uav_control_ws
colcon build --symlink-install --packages-select uav_ommpc_interfaces uav_ommpc_controller uav_ommpc_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/uav_control_ws/install/setup.bash
```

In every ROS terminal, source the underlays **before** this workspace:

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
```

`~/ros2-px4-env.bash` supplies the existing ROS/PX4-message environment; it does not replace the new overlay. Use `~/px4-env.bash` in the separate PX4 simulator terminal.

```bash
ros2 run uav_ommpc_ros px4_bridge --ros-args \
  -p hover_thrust:=0.728744 -p max_body_rates:="[3.0, 3.0, 1.5]" -p max_throttle:=0.9 \
  -p rate:=50.0 -p command_timeout:=0.5 -p target_system:=1 -p target_component:=1
```

Parameters are read at startup; restart to change them. `rate` is outgoing setpoint/heartbeat frequency in Hz. `command_timeout` is maximum accepted-input age in seconds, measured with a monotonic clock. `hover_thrust` and `max_throttle` must be in (0, 1], `max_body_rates` must contain three positive roll/pitch/yaw limits in rad/s, rate and timeout must be positive, and target IDs in [1, 255].

## Public interface

All topics use volatile durability and depth 1; no commands are retained for future subscribers.

| Topic | Message | Reliability | Meaning |
| --- | --- | --- | --- |
| `/uav/control_input` | `uav_ommpc_interfaces/msg/ThrustRates` | reliable | External controller input; stream at least 50 Hz |
| `/uav/state` | `px4_msgs/msg/VehicleOdometry` | best effort | Unmodified PX4 odometry |
| `/uav/status` | `px4_msgs/msg/VehicleStatus` | best effort | Unmodified PX4 vehicle status |
| `/uav/command_ack` | `px4_msgs/msg/VehicleCommandAck` | best effort | Unmodified PX4 command acknowledgements |
| `/uav/vehicle_command` | `px4_msgs/msg/VehicleCommand` | reliable | Explicit mode/arm/land/etc. request |
| `/uav/reference_polynomial` | `uav_ommpc_interfaces/msg/PolynomialTrajectory` | reliable | Timed normalized polynomial segments for `ommpc_node` |

`ThrustRates` fields:

- `float64 thrust_acceleration`: finite, nonnegative **total** thrust acceleration in m/s², along body **−Z**. Includes gravity compensation: nominal level hover is `9.80665`, not zero.
- `float64[3] body_rates`: finite roll, pitch, yaw angular rates in rad/s, in body **FRD** (forward, right, down). These are rates, not Euler angles.

Telemetry remains in raw PX4 frames, without an ENU/FLU transform: position/velocity are NED (north, east, down) when the frame fields report NED; `q` is **w, x, y, z**; angular velocity is body FRD. Consumers must check `pose_frame`, `velocity_frame`, estimator validity and freshness. PX4 timestamps are forwarded unchanged.

The bridge subscribes with best-effort QoS to `/fmu/out/vehicle_odometry` (message version 0), `/fmu/out/vehicle_status_v1`, and `/fmu/out/vehicle_command_ack`. It publishes reliable volatile `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_rates_setpoint`, and `/fmu/in/vehicle_command`. Only `body_rate` is enabled in `OffboardControlMode`; all other flags remain false.

```text
roll, pitch, yaw = clip(body_rates, -max_body_rates, +max_body_rates)   # per axis
throttle = min(hover_thrust * thrust_acceleration / 9.80665, max_throttle)
thrust_body = [0, 0, -throttle]
```

Defaults are `max_body_rates=[3.0, 3.0, 1.5]` rad/s and `max_throttle=0.9` (keeping attitude-control headroom). Limiting is silent to PX4 but logs a throttled warning once per second; the controller is not otherwise notified.

For example, in a sourced ROS terminal (this publishes input only; it does not arm):

```bash
ros2 topic pub -r 50 /uav/control_input uav_ommpc_interfaces/msg/ThrustRates \
  '{thrust_acceleration: 9.80665, body_rates: [0.0, 0.0, 0.0]}'
```

This is an **approximate normalized-throttle mapping**, not a physical motor/thrust model or guaranteed acceleration tracking. The node's generic default is `hover_thrust=0.60`; explicitly use **0.728744 for this installed Gazebo x500**. Its model has mass 2.06430769 kg, four motor constants 8.54858e-6, and linear ESC speed range 150–1000 rad/s (`THR_MDL_FAC=0`): `omega_hover=sqrt(mass*g/(4*k))=769.4326`, so normalized hover is `(omega_hover-150)/850=0.728744`. PX4's `MPC_THR_HOVER=0.60` is an initial controller estimate, not a sufficient calibration for this external feedforward path. Other vehicles, battery conditions and payloads require calibration. The mapping is exact only at hover; with this motor model thrust grows with the square of rotor speed, so non-hover accelerations deviate. There is no slew-rate or tilt limiting.

Forwarded vehicle commands preserve their command and parameters but replace timestamp, `source_system=250`, `source_component=190`, configured target IDs, and `from_external=True`. Generated command/setpoint timestamps use ROS clock microseconds (wall clock by default; do not enable simulated ROS time without arranging its clock source).

## Control ownership and safety

The bridge **never automatically arms or changes flight mode**. External software must explicitly request and confirm these actions, satisfy PX4 offboard entry requirements, and monitor status/acknowledgements. Stream setpoints before requesting offboard mode. PX4 must be connected through a running Micro XRCE-DDS Agent and expose the expected DDS topics.

Nonfinite input values and negative thrust are rejected without refreshing the timeout or replacing the previous command. Otherwise the last accepted, limited input is repeated at the configured rate.

| Condition | `offboard_control_mode` heartbeat | `vehicle_rates_setpoint` |
| --- | --- | --- |
| No accepted input yet, or input older than `command_timeout` | stopped | stopped |
| Fresh input, PX4 `nav_state` is not Offboard | published | **not published** |
| Fresh input, PX4 `nav_state` is Offboard | published | published |

The heartbeat must flow before PX4 can enter Offboard, so it is not gated by mode. Rate setpoints are gated because PX4's `mc_rate_control` uses the newest `vehicle_rates_setpoint` from any publisher; outside Offboard they would compete with PX4's own attitude controller. The gate follows the latest received `VehicleStatus` (PX4 publishes it immediately on a mode change).

The bridge does not send zero thrust, land, disarm, retry, or choose recovery modes. After the input times out, PX4 holds the last setpoint until its offboard-loss timeout (`COM_OF_LOSS_T`, default 1.0 s) and then applies its configured offboard-loss failsafe. Telemetry and explicit vehicle commands continue. This is not a complete safety system. Ctrl-C does not itself land or disarm a vehicle.

## Unit tests

Bridge input validation, limits and timeout logic live in `uav_ommpc_ros/src/command_state.hpp` and use injected time in tests. `src/ommpc_inputs.hpp` tests odometry decoding, polynomial metadata/construction, continuous reception-time anchors, explicit holds and expiry. These unit tests do not require running ROS/PX4 processes.

```bash
cd ~/uav_control_ws
colcon test --packages-select uav_ommpc_ros
colcon test-result --verbose --test-result-base build/uav_ommpc_ros
```

## Reference test publisher and OMMPC SITL experiments

`tools/reference_test_publisher.py` publishes only `/uav/reference_polynomial`, using the same polynomial message as an upper-level planner. It never arms, changes modes or publishes `/uav/control_input`.

| Mode | Reference behavior |
| --- | --- |
| `hover` | Fixed NED position and measured/supplied yaw |
| `target_step` | After `--start-delay`, switch the entire horizon to a displaced hover equilibrium; default offset `[0.6,0.3,0] m` |
| `constant_velocity` | Smooth speed ramp, constant-speed straight segment, smooth braking, then endpoint hold |
| `circle` | Smooth angular-speed ramp, then a horizontal circle at fixed yaw |
| `hover_to_hover` | Degree-7 blend between two hover equilibria with zero velocity, acceleration and jerk at both ends; duration `35/16 * distance / speed`, so `--speed` is the exact peak speed |

All five geometry modes now publish real polynomial coefficients; `polynomial` is the transport representation, not a separate geometry selector.

`hover_to_hover` is the smoothness benchmark. Peak speed equals `--speed` exactly halfway through the displacement, and duration scales as `distance/speed`, so raising `--speed` shortens the blend and increases peak acceleration roughly with `speed²`. The displacement comes from `--offset N E D`.

### Trajectory construction and publisher CLI

The publisher uses analytic p/v/a/j to build normalized degree-7 Hermite pieces on shared global knots, splitting at speed-ramp boundaries. Velocity, acceleration and body rates for diagnostics are evaluated from those polynomials, not forward differences. It checks C3/C1 continuity, terminal holds and sampled feedforward bounds `[1,12] m/s²` / `±[3,3,1.5] rad/s`. These sampled checks are not a certificate of between-sample actuator feasibility; the controller checks every input it uses.

The target step is a change of **desired equilibrium between messages**, not a state teleport inside one horizon. Its instantaneous reference error is expected in the step-test metrics. The trajectory before/after the step is dynamically consistent in each message.

Line/circle speed ramps integrate a quintic smootherstep. For the line, `--line-duration` is the constant-speed duration; terminal displacement is `normalize(direction)*speed*(line_duration+ramp)` and motion finishes at `start_delay+line_duration+2*ramp`. Defaults give a 3.3 m North displacement, peak reference speed 0.3 m/s, and 16 s until endpoint hold. Circle starts at the origin with tangent North; its center is `origin+[0,radius,0]`, and `p=origin+radius*[sin(theta),1-cos(theta),0]`. Yaw remains fixed, not tangent-facing.

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
cd ~/uav_control_ws/src/uav-ommpc-controller

# Offline feasibility checks; they do not publish or control a vehicle.
python3 tools/reference_test_publisher.py --mode hover --check
python3 tools/reference_test_publisher.py --mode target_step --check
python3 tools/reference_test_publisher.py --mode constant_velocity --check
python3 tools/reference_test_publisher.py --mode circle --speed 0.25 --check
python3 tools/reference_test_publisher.py --mode hover_to_hover --speed 0.4 --check
python3 -m unittest discover -s tools -p test_reference_test_publisher.py -v

# Reference-only publishing; waits for valid odometry to capture unspecified origin/yaw.
python3 tools/reference_test_publisher.py --mode hover
python3 tools/reference_test_publisher.py --mode target_step --offset 0.6 0.3 0
python3 tools/reference_test_publisher.py --mode constant_velocity --speed 0.3 --direction 1 0 0
python3 tools/reference_test_publisher.py --mode circle --radius 0.6 --speed 0.25
python3 tools/reference_test_publisher.py --mode hover_to_hover --offset 2 0 0 --speed 0.4
```

Run only **one reference publisher** at a time. `--origin N E D` and `--yaw RAD` override captured odometry values; default yaw preserves heading. The removed `--dt`/`--samples` arguments are replaced by `--window` (default 4 seconds) and `--segment-duration` (default 0.5 seconds); `--rate` remains 25 Hz. Window length must cover the controller horizon and transport delay. Overlapping windows retain common knots and analytic geometry.

`tools/sitl_hover_to_hover_sweep.sh` repeats a full takeoff → `hover_to_hover` → native-landing flight for each speed in `SPEEDS`, restarting PX4 between runs so every run starts landed and disarmed. `REPEATS` sets the number of runs per speed and `LABEL` names the evidence directories (`<LABEL>-<speed>ms-r<k>`). Each run consumes a complete simulated flight; it is not a substitute for a dedicated hardware test campaign.

`tools/analyze_sitl_sweep.py DIR...` summarises any set of those runs: per-run tracking, max tilt, worst control gap, control samples at the thrust/rate limits, and per-speed mean/std so repeatability is visible without opening the CSVs.

`tools/pd_so3_node.py` is a **test-only** PD + SO(3) baseline for controller comparison. It subscribes to the same `/uav/reference_polynomial` and publishes the same `ThrustRates`, with the same gating (fresh advancing odometry, healthy status, receipt lease, estimator-reset and clock-rollback invalidation, sampling at the odometry receipt time), so trajectory, vehicle, initial condition and interface are identical. Its law is the repository's existing PD + SO(3) test controller extended with reference feedforward: `a_cmd = a_ref + kp(p_ref-p) + kd(v_ref-v)` clipped per axis, `b3` from `g*e3-a_cmd`, `rates = clip(k_rot*vee(R^T R_des - R_des^T R), ±1 rad/s)` and `thrust = clip(force·R[:,2], 1, 12)` with the defaults `kp=[1.2,1.2,1.8]`, `kd=[1.8,1.8,2.1]`, `k_rot=4`. It never arms or changes modes.

Repeated cached plans retain their start stamp and renew the receipt lease. As a rolling window shifts, its start stamp advances by the same amount as its coefficient domain; publication time never silently restarts the path. `--duration SECONDS` stops publication but does **not** land. Ctrl-C, lease expiry or generation failure leads to the existing node/bridge timeout behavior, not a controlled recovery. The reference-only publisher is not a flight safety state machine.

### SITL-specific tuning and time-boundary correction

**Do not use the ideal-model benchmark defaults as validated flight tuning.** The first real Gazebo/PX4 hover attempt (`hover-01`) hit the unchanged 0.6 rad tilt-abort threshold during takeoff, then landed and disarmed through native PX4 NAV_LAND. The failed result, CSV and bag were preserved.

The separately installed `uav_ommpc_ros/config/ommpc_sitl.yaml` lowers feedback bandwidth for the simulated PX4 inner loop, without changing the controller library defaults:

- `horizon=30`, `dt=0.02` (0.6 s window).
- State weights `[30,30,60,20,20,30,80,80,40]`, terminal weights twice those values.
- Input weights `[4,20,20,5]`; original thrust/rate bounds unchanged.
- Bridge calibration remains `hover_thrust=0.728744`, `max_throttle=0.9`.

For context, the hover horizontal finite-horizon position/rate feedback gain drops from approximately 20.86 to 0.287 rad/s per metre, while attitude damping remains approximately 4.28 rad/s per radian. This is a conservative low-speed test configuration, not a high-speed or real-hardware qualification.

A later attempt (`tracking-01`) was correctly blocked **before arming** because the node discarded timely solutions whenever solve execution crossed a reference sample boundary. That made output depend on timer phase. `ommpc_node.cpp` now retains the solve-period/freshness/full-horizon checks without requiring an unchanged sample index. A disarmed regression swept reference timestamp offsets 0,3,6,9,12,15,18 ms: each 0.6 s observation produced 30–31 commands, maximum gap below 0.029 s. These changes do not suppress the warning or bypass a timeout; they correct the overly strict phase condition.

### Reproduce the actual flight tests

Start the existing DDS Agent, headless `gz_x500`, GCS heartbeat and calibrated bridge using the prerequisites in the next section. Do not start the old `sitl_flight_test.py` PD controller concurrently. In a separate sourced ROS terminal:

```bash
ros2 run uav_ommpc_ros ommpc_node --ros-args --params-file \
  "$(ros2 pkg prefix uav_ommpc_ros)/share/uav_ommpc_ros/config/ommpc_sitl.yaml"
```

Then run the flight utility instead of a separate reference publisher:

```bash
cd ~/uav_control_ws/src/uav-ommpc-controller

# Incremental hover first, then the remaining three phases.
python3 tools/sitl_ommpc_test.py --confirm-sitl --modes hover --record-bag \
  --output-dir ~/uav_control_ws/ommpc-flight-results/NEW_HOVER_RUN
python3 tools/sitl_ommpc_test.py --confirm-sitl \
  --modes target_step constant_velocity circle --record-bag \
  --output-dir ~/uav_control_ws/ommpc-flight-results/NEW_TRACKING_RUN

# Alternatively run all four phases in one flight:
python3 tools/sitl_ommpc_test.py --confirm-sitl --record-bag
```

`--confirm-sitl` is a mandatory operator assertion: DDS message types cannot prove hardware identity. The operator must ensure this ROS domain reaches only the intended simulator. The utility refuses an initially armed/moving/non-landed vehicle and requires advancing raw `/fmu` and bridged telemetry. It prestreams references, requests Offboard/arm with matching accepted acknowledgements and observed status, ramps the hover target to 2 m, runs each phase, then requests native NAV_LAND and waits for fresh land detection plus disarmed status. It never publishes a PD control command, force-arms or disables preflight/failsafe parameters.

Safety/acceptance checks remain explicit: fresh odometry <=0.2 s and status <=1.5 s; active control/rate gap <=0.3 s; abort above 2 m tracking error or 0.6 rad tilt. Each endpoint must hold for >=5 s within 0.35 m position error, 0.3 m/s speed, 0.25 rad tilt and 0.2 rad yaw error. Line/circle phase RMSE must be <=0.35 m position and <=0.3 m/s velocity; every trajectory phase maximum position error must be <=1 m. Accepted OMMPC input tolerance is 1e-4 in corresponding input units, matching the solver contract.

### Metrics and recordings

The runner writes `result.json`, aggregate `trace.csv`, phase JSON/CSV, analytically evaluated reference diagnostics in `reference.csv`, actual OMMPC commands in `control_input.csv`, and PX4-facing outputs in `bridge_rates.csv`. Tracking errors use the reference evaluated continuously at each odometry reception time. CSV fields now include `reference_elapsed_s`, not a floor-grid index/sample age; full polynomial coefficients are retained in the reference topic when recording a bag. Missing values remain null rather than fabricated zeros.

`--record-bag` records 13 reference/telemetry/control/command/acknowledgement topics into `bag/` and closes the recorder via SIGINT with bounded cleanup. Check `rosbag.log` and `ros2 bag info RUN_DIR/bag`. Bags include command topics: **never replay them into a live aircraft or an active control domain**. Analyze offline or use an isolated domain and explicit non-command topic selection.

The node writes per-solve iteration/status and timing to `solve_trace_path` (default `/tmp/ommpc_solve_trace.csv`). The flight runner copies the flight interval to `solve_timing.csv`. Fields include solve-finish ROS nanoseconds, odometry receipt nanoseconds, DDS publication/sample microseconds, steady-clock wait, reference start/anchor, prediction duration and reference/planned input components. `planned_*` is the QP solution **before** the publication safety recheck, not proof that it reached PX4. Actual-command analysis retains bag/ULog matching. This trace is not a new ROS topic.

### Observed PX4/Gazebo results — polynomial version

`tools/sitl_ommpc_test.py` drove the real SITL stack (PX4 v1.17, Gazebo Harmonic x500, Micro XRCE-DDS, `px4_bridge hover_thrust=0.728744`, `ommpc_node` with `ommpc_sitl.yaml`) through the **polynomial** interface only. Every flight below started from a fresh PX4 process, a disarmed and stationary vehicle, and ended in native `NAV_LAND` with observed disarmed status.

Run `poly-four-modes-02` (`--modes hover target_step constant_velocity circle`, 92.4 s, `passed=true`, takeoff height 2.04 m):

| Phase | Position RMSE (m) | Max error (m) | Velocity RMSE (m/s) | Max tilt (rad) | Max speed (m/s) | Bound violations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Polynomial takeoff | 0.0325 | 0.0492 | 0.0111 | 0.0033 | 0.3898 | 0 |
| Hover, 5 s stable | 0.0255 | 0.0343 | 0.0105 | 0.0040 | 0.0157 | 0 |
| Target step `[0.6,0.3,0] m` | 0.3146 | 0.6754 | 0.0861 | 0.0442 | 0.2029 | 0 |
| Straight line, 0.3 m/s | 0.0462 | 0.0871 | 0.0139 | 0.0194 | 0.3089 | 0 |
| Circle, radius 0.6 m / 0.25 m/s | 0.0612 | 0.1246 | 0.0395 | 0.0483 | 0.2574 | 0 |

The target step deliberately commands an instantaneous equilibrium change, so its large RMSE is the expected step response, not tracking failure. The moving phases tracked within 9 cm maximum error and 4 cm/s velocity RMSE. Across the whole run the bridge emitted 4,182 rate setpoints for 4,207 OMMPC commands and no input-bound violation was recorded. One 0.109 s control gap appeared during the circle phase, marginally above the 0.1 s advisory threshold and consistent with Python-side scheduling under rosbag recording; the acceptance limit is 0.3 s.

These are **simulated** flights. They establish low-speed SITL tracking with the polynomial reference path only; actuator-model mismatch, estimator error, inner-loop delay and hardware safety still require separate real-aircraft validation.

#### Degree-7 hover-to-hover smoothness and speed sweep

`tools/sitl_hover_to_hover_sweep.sh` flew the same 2 m `hover_to_hover` blend five times, restarting PX4 before each point so every flight began landed and disarmed. Each run includes a polynomial takeoff to 2 m, the blend, a 5 s settle within 0.35 m / 0.3 m/s / 0.25 rad / 0.2 rad, then native landing.

| Commanded peak speed (m/s) | Peak reference speed observed | Position RMSE (m) | Max position error (m) | Velocity RMSE (m/s) | Max tilt (rad) | Max thrust (m/s²) | Result |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.4 | 0.4000 | 0.0515 | 0.1030 | 0.0165 | 0.0137 | 9.8143 | pass |
| 0.8 | 0.8000 | 0.0807 | 0.1757 | 0.0318 | 0.0498 | 9.8154 | pass |
| 1.2 | 1.2000 | 0.0728 | 0.1747 | 0.0457 | 0.1258 | 9.8448 | pass |
| 1.6 | 1.6000 | 0.0978 | 0.2429 | 0.0845 | 0.2499 | 9.9949 | pass |
| 2.0 | 2.0000 | 0.1201 | 0.3201 | 0.1327 | 0.3968 | 10.3082 | pass |

Every point held zero input-bound violations, zero control gaps above 0.1 s, and a full 5 s settled window before landing. Tracking degrades monotonically with speed while the reference stays exactly on the requested peak: position RMSE grows from 5 cm to 12 cm and peak tilt from 0.014 to 0.397 rad. At 2.0 m/s the tilt reaches roughly 44% of the unchanged 0.6 rad abort threshold, so that point is the practical edge of this low-gain `ommpc_sitl.yaml` tuning rather than a demonstrated ceiling.

The first sweep attempt exposed a real bug: `hover_to_hover` durations were floored at `ramp` (3 s), so 1.6 and 2.0 m/s silently published the *same* 1.458 m/s reference. Duration is now `35/16 * distance / speed` with no floor, the observed peak reference speed tracks the request exactly, and a unit test pins that regression.

### Repeatability and PD + SO(3) comparison

Three repeats per speed for each controller, same 2 m `hover_to_hover` blend, same PX4 x500 SITL, same reference publisher, same bridge (`hover_thrust=0.728744`), and a fresh landed/disarmed vehicle before every run. Only the controller process differed. All 18 flights passed.

| Controller | Speed (m/s) | Runs | Position RMSE mean ± std (m) | Max position error (m) | Max tilt (rad) |
| --- | ---: | ---: | ---: | ---: | ---: |
| OMMPC | 0.8 | 3 | 0.0812 ± 0.0103 | 0.165–0.204 | 0.051 |
| PD + SO(3) | 0.8 | 3 | 0.0717 ± 0.0175 | 0.121–0.131 | 0.066 |
| OMMPC | 1.6 | 3 | 0.0944 ± 0.0063 | 0.212–0.265 | 0.245 |
| PD + SO(3) | 1.6 | 3 | 0.1149 ± 0.0039 | 0.300–0.318 | 0.181 |
| OMMPC | 2.0 | 3 | 0.1326 ± 0.0077 | 0.324–0.381 | 0.401 |
| PD + SO(3) | 2.0 | 3 | 0.2119 ± 0.0132 | 0.590–0.620 | 0.198 |

Repeatability: no run-to-run drift or outlier in either controller. OMMPC standard deviation is 5–13% of its mean; PD+SO(3) is 3–24%. At 0.8 m/s PD+SO(3) is slightly more accurate but more variable; from 1.6 m/s upward OMMPC is clearly better, and at 2.0 m/s its position RMSE is 37% lower with less than two-thirds the peak error.

Tilt, interruptions and saturation:

| Observation | OMMPC (2.0 m/s) | PD + SO(3) (2.0 m/s) |
| --- | --- | --- |
| Max body rate commanded | 1.29–1.30 rad/s | 1.00 rad/s (its own clamp) |
| Samples at its rate clamp | 0% (limits are ±3/±3/±1.5) | 2.0%, 12.1%, 2.0% of the three runs |
| Max thrust commanded | 10.31–10.32 m/s² | 9.86–9.91 m/s² |
| Samples at the 12 m/s² thrust limit | 0% | 0% |
| Worst control gap | 0.084–0.089 s | 0.094–0.120 s |
| Runs with any gap > 0.1 s | 0 of 3 | 1 of 3 |

Neither controller saturates the **interface** limits at any speed; OMMPC never reaches a clip, while PD+SO(3) saturates its own `±1 rad/s` clamp (its existing tuning, tighter than the ±3/±3/±1.5 interface) and its per-axis acceleration clip at the two highest speeds. That asymmetry explains the divergence: PD+SO(3) prioritises gentle attitude over tracking, so its tilt stays near 0.19 rad while its position error grows. Occasional gaps just above 0.1 s appear in both and are Python-side scheduling/rosbag effects, far below the 0.3 s acceptance limit.

Attribution caveat: OMMPC additionally benefits from a 30-step prediction horizon and per-step feedforward, while PD+SO(3) is a reactive law with the repository's low-gain default tuning. The comparison therefore answers "does the added MPC machinery beat the existing PD + SO(3) baseline under identical trajectory, vehicle and initial conditions" — at 1.6–2.0 m/s it does, at 0.8 m/s the two are within run-to-run noise. Both are simulated results.

### HKU MaRS SUPER (`gz_super`) vehicle profile and mass discrepancy note

In addition to the default `x500` model (`hover_thrust=0.728744`, `uav_ommpc_ros/config/ommpc_sitl.yaml`), this workspace and `~/PX4-Autopilot` include the **HKU MaRS SUPER (YunQue)** 280 mm quadrotor (`gz_super`, airframe `4022_gz_super`, Gazebo models `Tools/simulation/gz/models/super_base` and `super`, controller config `uav_ommpc_ros/config/ommpc_super_sitl.yaml`), derived from [`hku-mars/SUPER-Hardware`](https://github.com/hku-mars/SUPER-Hardware), [`hku-mars/SUPER`](https://github.com/hku-mars/SUPER) and Ren et al., *Science Robotics* 10(98), eado6187 (2025):

| Parameter | `x500` (preserved default) | `super` (`gz_super`) | Source / note |
| --- | --- | --- | --- |
| PX4 SITL target / airframe | `gz_x500` (`4001_gz_x500`) | `gz_super` (`4022_gz_super`) | `~/PX4-Autopilot` |
| Wheelbase / rotor `(px, py)` | `500 mm` / `(±0.174, ±0.174) m` | `280 mm` / `(±0.098995, ±0.098995) m` | Measured from `CarbonFiber/5mm_main_board.STEP` |
| Total simulated mass | `2.06430769 kg` | **`1.64 kg`** (`1.58 kg` base + `4 × 0.015 kg` rotors) | **Discrepancy note:** `hku-mars/SUPER` planner configs (`super_planner/config/*.yaml`, `traj_opt.flatness.mass`) specify **`1.64 kg`** (adopted here), whereas the *Science Robotics* paper (p. 13) reports **`1.50 kg`** takeoff weight |
| Propulsion / `motorConstant` | `8.54858e-6` (`1000 rad/s`) | `2.36e-5` (`1000 rad/s`, `23.6 N` max/motor, `94.4 N` total) | T-Motor F90 KV1300 + HQProp 7×4×3 on 6S (`BAT1_N_CELLS=6`) |
| Calibrated `hover_thrust` | `0.728744` | **`0.309128`** (for `1.64 kg`; `0.287942` if using `1.50 kg`) | `omega_hover = sqrt(1.64 * 9.80665 / (4 * 2.36e-5)) = 412.7587 rad/s`, `(412.7587 - 150) / 850 = 0.309128` |
| OMMPC ROS config | `config/ommpc_sitl.yaml` (`input_max=[12, 3, 3, 1.5]`) | `config/ommpc_super_sitl.yaml` (`input_max=[20, 3, 3, 1.5]`) | Bridge ceiling at `hover_thrust=0.309128, max_throttle=0.9` is `28.55 m/s² > 20.0 m/s²` |
| Linear rotor drag (`dh, dv`) | Not modeled | **Not modeled (omitted by design for now)** | `hku-mars/SUPER` uses `dh=0.35, dv=0.35`; current OMMPC dynamics/sampler remain drag-free |

To run `gz_super` in SITL alongside the existing `x500` workflow:

```bash
# Terminal 2: PX4 and headless Gazebo with SUPER model.
source ~/px4-env.bash
cd ~/uav_control_ws/src/uav-ommpc-controller
bash tools/start_super_sitl.sh

# Terminal 3: bridge calibrated for 1.64 kg SUPER.
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.309128

# Terminal 4: OMMPC with SUPER SITL parameters.
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
ros2 run uav_ommpc_ros ommpc_node --ros-args --params-file \
  "$(ros2 pkg prefix uav_ommpc_ros)/share/uav_ommpc_ros/config/ommpc_super_sitl.yaml"

# Terminal 5: SITL flight test using the super profile (max_thrust=20 m/s²).
cd ~/uav_control_ws/src/uav-ommpc-controller
python3 tools/sitl_ommpc_test.py --confirm-sitl --vehicle super --modes hover hover_to_hover --speed 0.8 \
  --output-dir ~/uav_control_ws/ommpc-flight-results/super-hover-0.8ms
```

Observed `gz_super` SITL verification (`~/uav_control_ws/ommpc-flight-results/super-hover-0.8ms`, `passed=true`, max takeoff height `2.031 m`, confirmed native `NAV_LAND` and disarm):

| Phase (`gz_super`, `1.64 kg`) | Position RMSE (m) | Max position error (m) | Velocity RMSE (m/s) | Max tilt (rad) | Max bridge throttle | Bound violations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Polynomial takeoff (`2.0 m`) | 0.0242 | 0.0402 | 0.0115 | 0.0038 | 0.3117 | 0 |
| Hover (`5 s` settled) | 0.0368 | 0.0470 | 0.0137 | 0.0034 | 0.3090 | 0 |
| `hover_to_hover` (`0.8 m/s`) | 0.0508 | 0.1142 | 0.0464 | 0.1917 | 0.3131 | 0 |

#### SUPER 3x3 speed scan, complex trajectories, Q/R/horizon tuning, and bottleneck diagnosis

All trajectories enforce kinematic bounds v <= 4.0 m/s, a <= 8.0 m/s², j <= 20.0 m/s³, and support circle, figure8 (Lissajous figure-8), helix (3D wave orbit) and hover_to_hover. Baseline parameters (uav_ommpc_ros/config/ommpc_super_orig.yaml, N=30) were compared against the SUPER-tuned configuration (uav_ommpc_ros/config/ommpc_super_sitl.yaml, N=20, state_weights=[300,300,400, 85,85,80, 80,80,60], terminal_weights=[4500,4500,6000, 255,255,240, 160,160,120], input_weights=[1.2, 10, 10, 4]):

| Trajectory & speed | Baseline RMSE (m) | Tuned RMSE (m) | Baseline max error (m) | Tuned max error (m) | Tuned lag (ms) | Tuned max tilt (rad) | Tuned solve mean (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| hover_to_hover 0.8 m/s (n=3) | 0.0642 ± 0.0081 | **0.0351 ± 0.0021** | 0.1633 | **0.0776** | +4.0 | 0.0578 | 1.24 |
| hover_to_hover 1.6 m/s (n=3) | 0.0998 ± 0.0025 | **0.0528 ± 0.0022** | 0.2624 | **0.1464** | -30.7 | 0.2064 | 1.26 |
| hover_to_hover 2.0 m/s (n=3) | 0.1188 ± 0.0097 | **0.0637 ± 0.0015** | 0.3090 | **0.1762** | -49.3 | 0.3387 | 1.22 |
| hover_to_hover 4.0 m/s (4.5 m) | 0.2055 | **0.1301** | 0.5536 | **0.3792** | -41.0 | 0.5354 | 1.21 |
| circle 1.6 / 2.0 / 3.0 / 4.0 m/s | 0.135 / 0.127 / 0.190 / 0.229 | **0.055 / 0.057 / 0.079 / 0.107** | 0.319 / 0.233 / 0.349 / 0.399 | **0.126 / 0.102 / 0.135 / 0.198** | +18 / +5 / -4 / -3 | 0.232–0.577 | 1.25–1.47 |
| figure8 1.6 / 2.0 / 3.0 / 4.0 m/s | 0.166 / 0.202 / 0.293 / 0.392 | **0.105 / 0.124 / 0.186 / 0.254** | 0.325 / 0.423 / 0.760 / 0.887 | **0.197 / 0.263 / 0.427 / 0.564** | -91 to -113 | 0.189–0.515 | 1.21–1.51 |
| helix (3D) 1.6 / 2.0 / 3.0 / 4.0 m/s | 0.118 / 0.151 / 0.266 / 0.306 | **0.056 / 0.062 / 0.091 / 0.113** | 0.280 / 0.362 / 0.584 / 0.647 | **0.121 / 0.196 / 0.169 / 0.245** | +10 to -9 | 0.210–0.779 | 1.15–1.23 |

High-speed tracking bottleneck identification from recorded telemetry:
1. **Inner-loop angular-rate response**: attitude-difference estimates suggest 44–60 ms of end-to-end rate lag. Transport, recording timestamps, filtering and motor dynamics are confounded; this is not an isolated identification of the PX4 rate loop.
2. **Throttle/thrust mismatch**: the linear bridge law applied to the Gazebo quadratic motor law has hover slope 1.273. The controlled mapping comparison below tests its effect with unchanged OMMPC parameters.
3. **Rotor drag**: acceleration residuals suggest an effective horizontal damping near 0.074 s⁻¹ in this simulation, but also contain timing, estimator and thrust-model errors. The planner's `dh/dv=0.35` values must be checked against its equations and units before reuse. No experiment established percentage contributions of these three effects.

#### Optional quadratic inverse thrust mapping: unchanged OMMPC A/B comparison

The original linear mapping remains the **default** (`quadratic_thrust_model=false`).
Only `px4_bridge` changes the acceleration-to-throttle conversion; the optimizer,
prediction model, Q/R/terminal weights, N=20 and dt=0.02 s remain unchanged.
Enable the calibrated SUPER mapping explicitly:

```bash
ros2 run uav_ommpc_ros px4_bridge --ros-args \
  -p hover_thrust:=0.309128 -p quadratic_thrust_model:=true \
  -p min_motor_speed:=150.0 -p max_motor_speed:=1000.0
```

For `span = omega_max - omega_min`, the mapping is:

```text
omega_hover = omega_min + span * hover_thrust
omega_desired = omega_hover * sqrt(thrust_acceleration / 9.80665)
throttle = clamp((omega_desired - omega_min) / span, 0, max_throttle)
```

This is the exact static inverse of the **calibrated equal-rotor quadratic model**,
not an exact inverse of the entire PX4 allocator, motor transients or real aircraft.
It assumes normalized throttle maps linearly to rotor speed (the tested Gazebo
ESC range); do not combine it blindly with PX4 thrust linearization or a different
ESC law. Differential rotor speeds used for attitude control also change total
thrust. Recalibrate for another vehicle, payload or battery condition.
Below idle thrust the requested acceleration is physically unreachable: output
clamps to zero throttle and reports `limited`, not zero physical rotor thrust.
The upper throttle clamp, signed body-rate limits, input rejection, watchdog and
Offboard ownership remain in effect.

`tools/sitl_thrust_mapping_ab.sh` runs four cases × two mappings × three repeats.
It requires the existing simulator-only DDS/GCS environment and refuses to
overwrite result directories. `CASE_FILTER`, `FIRST_RUN`, `REPEATS`, `MAPPINGS`
and `OUT_ROOT` select additional runs. The script uses an exploratory tilt abort
of **1.0 rad** and phase RMSE thresholds of **1.0 m / 1.0 m/s**, not the older
0.6 rad / 0.35 m / 0.3 m/s criteria. Do not equate its `passed` flag with the
older qualification. Logs and CSV evidence are under
`~/uav_control_ws/ommpc-flight-results/thrmap-{linear,quadratic}-*-r*`.
These runs did not record rosbag. Analyze with:

```bash
python3 tools/analyze_thrust_mapping.py
```

A/B observations, mean of three completed trajectory runs per cell (phase RMSE includes the initial hold and final settling; moving Z RMSE uses reference speed >0.05 m/s):

| Case | Linear position RMSE (m) | Quadratic position RMSE (m) | Linear moving Z RMSE (m) | Quadratic moving Z RMSE (m) | Linear velocity RMSE (m/s) | Quadratic velocity RMSE (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Helix, 3.0 m/s, R=1.6 m, Az=0.40 m | 0.0882 | 0.0812 | 0.0704 | 0.0575 | 0.1327 | 0.1075 |
| Sharp3D, 3.85 m/s, R=1.65 m, Az=0.60 m | 0.1688 | 0.1619 | 0.0414 | 0.0352 | 0.1930 | 0.1799 |
| Hover-to-hover, 4.0 m/s, displacement 4.5 m | 0.1276 | 0.1285 | 0.0127 | 0.0152 | 0.1411 | 0.1408 |
| S-curve, 3.9 m/s, R=1.6 m, two periods | 0.1837 | 0.1819 | 0.0228 | 0.0242 | 0.1934 | 0.1894 |

The initial campaign completed 23/24 flights. Quadratic S-curve r2 aborted
before entering the requested trajectory because a command/rate gap exceeded
0.3 s; native landing and disarm were observed. Its evidence is retained, and
the analysis counts it as a failed attempt with missing trajectory metrics.
An additional quadratic S-curve r4 completed successfully. Its aggregate uses
r1/r3/r4, with **3 successful trajectories out of 4 attempts**, not 3/3 success.
The full campaign therefore has 24 completed flights out of 25 attempts.
The final bridge build passed 21 ROS-package tests, including inverse mapping,
lower/upper throttle clipping and rejected-input watchdog behavior; the r4
flight exercised that final build. No OMMPC weights or model were changed.

The clearest benefit is vertical tracking in Helix: moving Z RMSE decreased
18.3%, velocity RMSE 18.9%, and mean NED-down bias changed from -0.0202 m
to -0.0008 m. Its odometry-derived thrust gain estimate changed from 1.326
to 1.008; differentiation, timestamp alignment and drag limit identification
accuracy. Horizontal point-to-point position RMSE did not improve. This does
not demonstrate a universal tracking improvement or establish causal error
fractions. Mapping groups ran sequentially rather than randomized/interleaved.


#### SUPER rate-response identification before delay compensation

`tools/sitl_rate_identification.py` is a simulator-only additive excitation relay.
OMMPC output is remapped to `/uav/sysid/raw_control`; the relay forwards each new
command once to `/uav/control_input`, adding excitation to one axis at a time.
It never repeats stale commands or clips the identification input to manufacture
an apparently valid experiment. Production controller/bridge code and Q/R/N are
unchanged. Quadratic thrust mapping remains enabled for these runs.

After a normal 2 m takeoff and settled hover, each axis receives a 24 s chirp
from 0.4 to 6 Hz, followed by independent 12 s alternating doublets (0.3 s positive,
0.3 s negative, 1.4 s zero). Chirp edges have a 1 s cosine taper. Injected roll/
pitch amplitudes are 0.22 rad/s and yaw amplitude is 0.18 rad/s; the **total**
setpoint also contains OMMPC feedback and is the input used for identification.
Existing 0.6 rad tilt, 2 m position-error and freshness aborts remain enabled.
This is closed-loop small-signal identification, not a high-rate/aggressive-flight
qualification or a real-hardware experiment.

Reproduce in a simulator-only ROS domain with existing DDS/GCS tmux windows:

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
python3 tools/run_rate_identification.py --confirm-sitl --repeats 3 \
  --output-root ~/uav_control_ws/ommpc-flight-results/NEW_RATE_ID

# This Jetson has pyulog in the PX4 venv and scipy/numpy in system dist-packages.
PYTHONPATH=/usr/lib/python3/dist-packages ~/.venvs/px4-v1.17/bin/python \
  tools/analyze_rate_identification.py \
  ~/uav_control_ws/ommpc-flight-results/NEW_RATE_ID
```

Observed campaign: `~/uav_control_ws/ommpc-flight-results/rate-id-01/r{1,2,3}`.
All three flights completed with native landing/disarm and zero ULog dropouts.
Maximum whole-flight tilt was 0.0731, 0.0912 and 0.0668 rad respectively.
Each directory retains `px4.ulg`, ROS bag, `excitation.csv`, flight CSV/JSON,
controller snapshot/hash, and aligned per-axis chirp/doublet CSVs.
Root `identification.json` retains all fitted candidate models and validation
scores; `transport_timing.json` contains separate ROS-side timing observations.

Input is ULog `vehicle_rates_setpoint` on its recorded timestamp; output is
`vehicle_angular_velocity.xyz` on `timestamp_sample`, **not differentiated attitude**.
The input interval is about 20 ms and gyro interval 8–12 ms. DDS conversion
subtracts the synchronized offset and clamps future setpoint timestamps to PX4
receipt time. Consequently the fitted delay is an effective timestamp-to-filtered-
gyro delay, not an independently isolated motor delay. ROS phase boundaries are
mapped using `timesync_status`; internal input/output fitting stays on PX4 time.

| Axis | Selected model | Gain K, mean | Delay L, three runs (ms) | Dynamics, three runs | Held-out doublet fit (%) |
| --- | --- | ---: | --- | --- | --- |
| Roll | K exp(-Ls)/(Ts+1) | 1.0557 | 17.93 / 12.67 / 16.51 | T=27.12 / 28.26 / 27.16 ms | 93.66 / 96.95 / 96.63 |
| Pitch | K exp(-Ls)/(Ts+1) | 1.0559 | 16.32 / 12.72 / 12.79 | T=26.31 / 28.51 / 27.69 ms | 96.58 / 96.05 / 96.90 |
| Yaw | K wn² exp(-Ls)/(s²+2*zeta*wn*s+wn²) | 1.0492 | 15.96 / 15.45 / 18.19 | wn=9.295 / 9.297 / 9.284 rad/s; zeta≈0.489 | 98.63 / 99.09 / 99.16 |

Fit is `100*(1-||y-yhat||/||y-mean(y)||)`, not classification accuracy or R².
Models were fitted on chirps only; doublets were not used in fitting.
The r1 model transferred without refitting to r2/r3 doublets with fit ranges
94.45–96.78% (roll), 96.23–97.10% (pitch), 97.65–98.90% (yaw).
Roll/pitch second-order candidates improve validation only slightly, so the
first-order form is preferred for simplicity. Yaw first-order validation was
only 77.86–78.95%; its delay-only candidate hit the 120 ms search bound and is
not an identified 120 ms physical delay. The yaw second-order response is
required to represent its underdamped dynamics.

Do not implement a universal 50 ms pure-delay shift from the previous correlation
estimate: roll/pitch low-frequency lag includes about 27.5 ms of dynamics **in
addition to** approximately 14–16 ms of fitted delay. Yaw is substantially slower.
The apparent relay-output-to-bridge-stamp median waits were 1.86 / 11.20 / 2.21 ms;
the apparent odometry-sample-to-bag-receipt median ages were 35.21 / 11.96 / 15.16 ms.
These ROS-side values include synchronization/recording effects, not just network
transport. Whole-flight timesync offset excursions were 14.62 / 2.38 / 5.19 ms.
Do not add these medians into a fixed compensation constant.

Fine-grid fitted decimals are not sub-millisecond certainty. Sampling, DDS clock
translation/clamping and delay-vs-lag model ambiguity require sensitivity checks
on the order of 5–10 ms before compensation. Three runs provide repeatability,
not a formal confidence interval. Direct closed-loop fitting may retain feedback/
noise bias; independent doublets and cross-flight validation reduce but do not
eliminate this risk. No delay compensation has been implemented by this experiment.

#### Switchable Roll/Pitch filtered rate compensation

`uav_ommpc_ros/src/roll_pitch_compensation.hpp` implements a fixed-size,
allocation-free, causal prefilter in `px4_bridge`. It leaves OMMPC, its Q/R/N,
yaw commands, thrust mapping and flight ownership unchanged. The original rate
path remains the default: **`rate_compensation_enabled=false`**.
The switch and its settings are read-only ROS startup parameters; land/disarm
and restart the bridge to switch configurations, not an in-flight toggle.

```bash
# Original rate path (quadratic thrust still enabled).
ros2 run uav_ommpc_ros px4_bridge --ros-args \
  -p hover_thrust:=0.309128 -p quadratic_thrust_model:=true \
  -p rate_compensation_enabled:=false

# Experimental SUPER Roll/Pitch compensation.
ros2 run uav_ommpc_ros px4_bridge --ros-args \
  -p hover_thrust:=0.309128 -p quadratic_thrust_model:=true \
  -p rate_compensation_enabled:=true \
  -p rate_compensation_gain:="[1.08, 1.08]" \
  -p rate_compensation_time_constant:="[0.0275, 0.0275]" \
  -p rate_compensation_filter_time_constant:=0.020
```

For each compensated axis, `r` is the desired rate, `q` is its low-pass state,
and `h` is the measured interval between accepted commands:

```text
dq = (r - q_previous) / (Tf + h)
q  = q_previous + h*dq
u  = (q + T*dq) / Kc
```

This is backward-Euler discretization of
`F(s)=(1+T*s)/(Kc*(1+Tf*s))`. It approximately offsets the identified first-order
lag while filtering high-frequency input changes; it **does not implement the
noncausal inverse of pure dead time**, and it does not predict vehicle state.
Yaw bypasses the filter. Original per-axis limits apply before and after it.
Rejected inputs update neither the filter nor the watchdog; first input,
expired lease and nonadvancing timestamps prime the filter without a derivative
kick. There is no command repetition or fallback generated by this module.

The fitted finite-band gain was about 1.056. The experimentally chosen inverse
gain `Kc=1.08` and softer `Tf=20 ms` trade some amplitude tracking for reduced
step overshoot. Thus the prefilter DC gain is `1/Kc`, not unity: do not interpret
overshoot reduction alone as better tracking. Report rate RMSE and plateau
tracking error as well. These settings are SUPER SITL results, not generic
aircraft calibration or a real-flight safety claim.

Pilot evidence is retained separately under `rate-comp-id-01` and
`rate-comp-candidates`: the initial `Kc=1.056, Tf=12 ms` setting reduced effective
lag but increased direct-step overshoot from approximately 5–6% to 12–14%.
It was rejected before the final repeated comparison. Final controller parameters
are fixed; only the bridge compensation switch differs in the OFF/ON campaign.

Reproduce alternating comparisons (existing simulator-only DDS/GCS prerequisites):

```bash
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite rates \
  --repeats 3 --output-root ~/uav_control_ws/ommpc-flight-results/NEW_RATE_AB
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite tracking \
  --repeats 3 --output-root ~/uav_control_ws/ommpc-flight-results/NEW_TRACKING_AB
```

The response campaign uses chirps plus the optional `--step-validation` protocol:
on each axis, 300 ms positive/negative constant rate plateaus and 200 ms zero
are followed by restored OMMPC feedback for the remainder of a 2 s cycle.
This permits actual step-response measurement rather than calling an additive
excitation plus changing feedback a constant step. Roll/Pitch amplitudes remain
0.22 rad/s and yaw 0.18 rad/s. Analyze **pre-compensation desired rate** from
`excitation.csv` against ULog gyro; comparing gyro with the compensated PX4
setpoint would measure the unchanged plant, not compensation benefit.

`tools/analyze_rate_compensation.py` reports chirp RMSE, normalized RMSE,
effective best-fit time shift, gain, 10/50/90% response times, and overshoot
normalized by each measured step transition. Plateau error uses the last
45 ms before each plateau ends, not a claim of infinite-time steady-state error.
`tools/analyze_rate_compensation_tracking.py` reports whole-phase and moving-only
position RMSE, maximum error, velocity RMSE, tilt, actual applied rate saturation
and solve times. Missing/failed runs are not labeled successful.
The high-speed suite uses fixed 0.9 rad tilt abort, 0.35 m phase position RMSE,
0.30 m/s velocity RMSE and 1 m maximum-error acceptance for both configurations.
It is not the earlier 0.6 rad tilt acceptance.

Final evidence is in `rate-comp-id-final` (6 response flights) and
`rate-comp-track-final` (24 trajectory flights), under the workspace flight
results directory. All 30 flights completed with native landing/disarm.
OFF/ON order was reversed on alternate repeats. All controller snapshots have
SHA256 `a7d3708d5b7d10a99b5cbe9e39fad7a9fed13e5ad9ed992ea221c2c5e8f5098b`.
The final ROS package build passed 25 tests covering the unchanged bypass,
yaw/thrust invariants, bounded lead boost, timeout reset, input rejection and
invalid/nonrepresentable filter gains.

There are two different latency measurements, and they must not be conflated:

- **Source-to-gyro:** desired-rate relay timestamp to ULog gyro, including bridge
  scheduling and synchronized clock uncertainty. Mean best-fit chirp lag was
  47.0→46.0 ms (roll), 46.0→46.3 ms (pitch). This does **not** demonstrate a
  reliable end-to-end latency reduction.
- **Bridge-aligned:** associate the desired rate with its actual bridge output
  event, then measure on native PX4 timestamps. `extract_rate_bridge_reference.py`
  matches the unchanged yaw payload to source commands, and the applied
  three-axis payload to ULog. OFF runs match all three source rates. Native
  matching coverage was 90.6–99.9%, with no ambiguous source matches in the
  roll/pitch excitation phases; skipped logger samples and repeated payloads
  remain limitations. This diagnostic excludes bridge scheduling wait.

Bridge-aligned means over 3 flights per configuration:

| Metric | Roll OFF | Roll ON | Pitch OFF | Pitch ON |
| --- | ---: | ---: | ---: | ---: |
| Chirp rate RMSE (rad/s) | 0.1872 | 0.1631 | 0.1789 | 0.1507 |
| RMSE / desired RMS | 0.6839 | 0.6184 | 0.6293 | 0.5702 |
| Effective best-fit lag (ms) | 41.33 | 36.33 | 41.33 | 34.67 |
| Step 90% response time (ms) | 59.82 | 53.78 | 60.06 | 54.43 |
| Step overshoot (% of step transition) | 5.42 | 0.65 | 5.37 | 0.54 |
| End-of-plateau relative error (%) | +6.33 | -1.61 | +6.16 | -1.67 |

Overshoot reduction is clear, but the resulting small undershoot must also be
reported. The apparent 5–7 ms bridge-aligned lag improvement is comparable with
sample/timestamp uncertainty; three repeats are not a formal confidence claim.
Yaw is unmodified; its bridge-aligned 90% response time remained approximately
241 ms OFF versus 240 ms ON.

High-speed tracking results (3 flights per cell; position/velocity RMSE are
whole named phase including ramps and settle; worst error is worst of all 3):

| Case | Position RMSE OFF→ON (m) | Moving-only position RMSE OFF→ON (m) | Worst position error OFF→ON (m) | Velocity RMSE OFF→ON (m/s) |
| --- | --- | --- | --- | --- |
| Hover-to-hover, 4.0 m/s | 0.1320→0.1261 | 0.1888→0.1791 | 0.3817→0.3714 | 0.1410→0.1319 |
| Helix, 3.0 m/s | 0.0792→0.0814 | 0.0924→0.0944 | 0.1594→0.1693 | 0.1045→0.1056 |
| S-curve, 3.9 m/s | 0.1814→0.1794 | 0.2450→0.2448 | 0.4422→0.4404 | 0.1901→0.1837 |
| Sharp3D, 3.85 m/s | 0.1655→0.1719 | 0.2192→0.2273 | 0.3997→0.3716 | 0.1873→0.1869 |

Position RMSE improved 4.5% in hover-to-hover and 1.1% in S-curve, but worsened
2.8% in Helix and 3.8% in Sharp3D. There is **no demonstrated universal trajectory
tracking improvement**. Both configurations had zero applied-rate saturation
samples and zero input-bound violations. Maximum tilt was 0.7444 rad OFF and
0.7878 rad ON. OMMPC mean solve times remained about 1.08–1.55 ms and observed
maxima below 4.69 ms; solver timing differences are not attributed to this
constant-size bridge prefilter.

The module remains default-off. Treat it as an experimental response-shaping
option, not a globally better controller or exact dead-time cancellation.
The final campaign retained bags, ULogs, CSV traces, controller snapshots,
matching-quality reports, `comparison.json`, `comparison_bridge.json` and
`tracking_comparison.json`. No failed pilot was overwritten.

Analysis commands (the bridge extractor needs both ROS and pyulog):

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
PY=~/.venvs/px4-v1.17/bin/python
ROOT=~/uav_control_ws/ommpc-flight-results
$PY tools/analyze_rate_compensation.py "$ROOT/rate-comp-id-final"
$PY tools/extract_rate_bridge_reference.py "$ROOT/rate-comp-id-final"
$PY tools/analyze_rate_compensation.py "$ROOT/rate-comp-id-final" --at-bridge
python3 tools/analyze_rate_compensation_tracking.py "$ROOT/rate-comp-track-final"
```

#### Native-clock tracking diagnosis and peak-error windows

`tools/analyze_tracking_diagnosis.py` evaluates bagged polynomial references
against ULog **simulation ground truth**, rather than treating estimator receipt
time as the physical state time. It adds:

- First maximum position error, native PX4 time, phase-relative time,
  flight-relative time and mapped ROS nanoseconds.
- Signed tangent error `dot(reference-position, reference_velocity/|velocity|)`;
  positive means behind the reference. Cross-track is the norm of the
  perpendicular **3-D vector**, not a signed planar error. Its components are
  also exported. At reference speed <=0.05 m/s both decompositions are undefined
  (`null`/blank), while total position error remains valid.
- Full-phase and maximum ±1 s PNG/PDF curves: desired/actual position and
  velocity, total/tangent/cross/axis errors, PX4 setpoint versus gyro, attitude
  error, controller state age where covered, and lateral dynamics residual.
  Boundary-clipped windows report their actual before/after coverage.
- A control-use decomposition: estimator error at sample time, physical motion
  during sample age, and held-state error at solve finish. These are vectors
  whose sum is exact; their RMS magnitudes are **not additive error percentages**.

Clock/frame rules:

1. For protocol-2 `timesync_status`, `native = ROS + estimated_offset`.
   The analyzer interpolates the measured clock relation and refuses
   extrapolation/nonmonotonic clocks. RTT and offset variation remain in the
   report. Ground-truth timestamps are Gazebo bridge callback times, so this
   does not establish sub-millisecond physical sensor timing.
2. Outgoing DDS translates **both** `timestamp` and `timestamp_sample`.
   Controller receipt/solve timestamps are recorded inside the C++ node;
   recorder callback time is not substituted for controller receipt.
3. Ground truth uses Gazebo world origin; the EKF uses local NED origin. A
   translation is fixed from stationary pre-arm data, never fitted over the
   moving trajectory. Position/origin resets require an explicit transform and
   are rejected. Heading/velocity resets are reported separately; the aggregate
   odometry reset counter is not automatically a position-frame change.
4. Published EKF control states are output-predicted to the IMU sample time;
   do not subtract an additional fusion-horizon delay. No best-fit lag shift
   changes primary tracking errors. Inner-loop best-fit shifts are labeled
   correlation diagnostics, not isolated dead time.

Tracking is sampled on a uniform 20 ms native grid; reported maxima are sampled
maxima, not continuous-time certificates. The dynamics diagnostic differentiates
truth velocity with a centered 7-point cubic Savitzky–Golay filter and rotates
`a_truth - [0,0,9.8]` into body coordinates. Its x/y components remove unknown
axial thrust. They still contain differentiation/interpolation error and
unmodeled lateral forces; they are **not automatically all aerodynamic drag**.
This uses actual world gravity 9.8 m/s², not the controller's 9.80665.
The actual SUPER motor SDF specifies rotor drag `8.06428e-5` per rotor and
motor response 12.5 ms up / 25 ms down. ESC RPM telemetry echoes commands,
not measured post-filter rotor speed.

The diagnostic suite keeps Q/R/N, vehicle, quadratic thrust mapping and rate
compensation OFF fixed. With `--repeats 2` it runs eight flights: 1.0 and
3.9 m/s straight lines once in each direction, plus two Sharp3D 3.85 m/s and
two S-curve 3.9 m/s runs. Straight-line pairs are mirrored-direction trials,
not identical-direction repetitions. Existing result directories are refused.

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
ROOT=~/uav_control_ws/ommpc-flight-results/NEW_TIMING_DIAGNOSIS
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite diagnostics \
  --repeats 2 --output-root "$ROOT"
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
~/.venvs/px4-v1.17/bin/python tools/analyze_tracking_diagnosis.py "$ROOT"
python3 tools/analyze_rate_compensation_tracking.py "$ROOT"
```

Each flight gains `diagnostics/report.json`, `<phase>_errors.csv`,
`<phase>_peak.png`, `<phase>_peak.pdf`, controller timing/decomposition CSV,
and `<phase>_dynamics.csv`. The campaign gains `diagnosis_comparison.json`.
The existing OFF/ON aggregator includes the corrected `native_truth_*` metrics
when available and explicitly labels its original receipt-time metrics;
it does not pool the two definitions.

##### Observed timing diagnosis and GPS-delay intervention

Evidence: `timing-diagnosis-20261028` (8 baseline flights) and
`timing-diagnosis-gps0-20261028` (4 intervention flights), under the workspace
flight-results directory. All 12 landed/disarmed, all ULogs had zero dropouts,
and controller-used sample IDs matched bagged states in 100% of recorded solves.
The Release ROS build and 25 ROS-package tests passed; 13 diagnostic numerical
tests cover clock sign/drift/bounds, heading versus position resets, projection,
stationary errors and peak-window edges. Actual PNG/PDF output was inspected.

The strongest tested issue is **the SITL GPS measurement-delay configuration**,
not just late delivery of the state. Baseline ULogs contain
`EKF2_GPS_DELAY=110 ms`. In this checkout:

- `PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp:713-799` stamps
  GPS sample/publication at the NavSat callback and adds noise, without a
  corresponding 110 ms measurement queue.
- `PX4-Autopilot/src/modules/ekf2/EKF/estimator_interface.cpp:180-182` subtracts
  the configured GPS delay before buffering the measurement.

On 3.9 m/s straight-line cruise, the same-sample estimator position was ahead
of truth by an equivalent median **101–112 ms** (position discrepancy projected
on velocity, not transport latency). Temporarily setting only the requested
GPS-delay override to 0 ms reduced that discrepancy to about **-7 to -8 ms**.
The control law, Q/R/N, rate gains, motor/drag model and compensation-OFF setting
were unchanged. All controller hashes match
`a7d3708d5b7d10a99b5cbe9e39fad7a9fed13e5ad9ed992ea221c2c5e8f5098b`.

True position RMSE is the mean of two full-phase run RMSEs. Maximum error is the
worst sample of the two runs. Straight-line trials use opposite directions.

| Case | Position RMSE, GPS delay 110→0 ms (m) | Observed RMSE reduction | Worst position error 110→0 ms (m) | Same-sample estimator position RMSE 110→0 ms (m) |
| --- | --- | ---: | --- | --- |
| Straight line, 3.9 m/s | 0.3242→0.0519 | 84.0% | 0.5881→0.1180 | 0.2876→0.0462 |
| Sharp3D, 3.85 m/s | 0.1862→0.0798 | 57.2% | 0.4957→0.2231 | 0.1413→0.0522 |

These percentages are **observed tracking improvements, not causal shares of
the original error**. Campaigns were sequential and each case has only two
runs. Flight counters differ; one Sharp3D baseline also had a different
automatically calibrated `CAL_BARO0_OFF` (-2.4414 versus 0), so its vertical
improvement is not attributed solely to GPS delay. The other three pairs have
no initial parameter differences apart from GPS delay and flight counters.
Exact differences are preserved in the intervention's `baseline_comparison.json`.
No 0 ms S-curve test was performed; do not extrapolate the measured percentage.

Peak examples from the 110 ms baseline (time since the named phase starts):

| Run | Peak time (s) | Total error (m) | Signed tangent (m) | Cross-track (m) |
| --- | ---: | ---: | ---: | ---: |
| line-3.9-r2 | 6.28 | 0.5881 | +0.5844 | 0.0659 |
| scurve-3.9-r1 | 6.60 | 0.5362 | +0.0729 | 0.5313 |
| sharp3d-3.85-r2 | 6.12 | 0.4957 | +0.2120 | 0.4481 |

The straight-line peak occurs during cruise, with small angular-rate error;
the turn peaks have much larger cross-track error. Some peaks occur at rest:
for `line-1.0-r1`, 0.1105 m at 16.42 s has no defined tangent/cross-track split.

Remaining limitations after fixing the experiment's GPS-delay assumption:

- Controller sample age at solve finish remains about 13–35 ms by per-run
  median in the intervention (baseline medians 7–33 ms). It did not uniformly
  improve. Receipt anchoring and state propagation are still worth testing,
  separately from estimator measurement timing.
- Roll/Pitch native setpoint-to-gyro effective lag remains roughly 26–50 ms
  across these runs. Yaw is slower; some correlation optima touch the ±150 ms
  search boundary and must not be read as identified dead time. These
  timestamps retain DDS translation/clamping and sampling limitations.
- Mirrored straight-line trials yield effective lateral drag about
  0.079 s^-1, with opposing acceleration around 0.08 m/s² at 1 m/s and
  0.31 m/s² at 3.9 m/s. This is consistent with the rotor-drag model.
  Sharp-turn residual fits are poor; do not label their entire residual drag.
- The intervention improved tracking while leaving inner-loop/drag models
  unchanged and sometimes increasing state age. Correct the **SITL-specific
  measurement timing** before judging new control-model changes. Then compare
  state propagation and actual rate dynamics explicitly; add drag feedforward
  with independently validated units. Do not copy GPS delay 0 to real hardware
  without measuring that sensor's actual latency.

Reproduce the isolated four-flight hypothesis test:

```bash
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite diagnostics \
  --repeats 2 --cases line-3.9 sharp3d-3.85 --gps-delay-ms 0 \
  --output-root ~/uav_control_ws/ommpc-flight-results/NEW_GPS_DELAY_TEST
```

The runner uses PX4's startup environment override, saves the parameter store
beside the evidence, and restores it only after confirmed landing/disarm and
PX4 shutdown. It refuses to restore over a live PX4 process. At the end of that
diagnostic campaign, parameter files were verified byte-identical to their
backups and the original 110 ms baseline was restored. The permanent
SUPER-specific startup correction below supersedes that historical default.

##### Corrected SUPER SITL timing default

`tools/start_super_sitl.sh` is the shared SUPER simulator startup entry point.
It always sets `PX4_PARAM_EKF2_GPS_DELAY=0` before starting `gz_super`, including
when an older parameter store or inherited `PX4_PARAM_EKF2_GPS_DELAY` says 110.
This is a **Gazebo SUPER configuration**, not a change to PX4 firmware defaults,
real-aircraft GPS configuration, OMMPC dynamics or reference time anchoring.

Both Python campaign runners and all three shell sweep scripts use this entry
point for SUPER. Non-SUPER targets retain their existing startup path. Normal
campaigns use 0 ms without `--gps-delay-ms`; they no longer temporarily apply 0
and then restore 110. The explicit diagnostic override still saves/restores
the prior parameter store, allowing historical 110 ms comparisons.

```bash
# Normal SUPER SITL startup; activate the PX4 Python environment automatically.
bash tools/start_super_sitl.sh

# Revalidate the fixed default: 8 flights, no temporary GPS-delay override.
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite diagnostics \
  --repeats 2 --output-root ~/uav_control_ws/ommpc-flight-results/NEW_CORRECTED_TIMING

# Only for a deliberate historical comparison, with store restoration afterward:
python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite diagnostics \
  --repeats 1 --cases line-3.9 --gps-delay-ms 110 \
  --output-root ~/uav_control_ws/ommpc-flight-results/NEW_HISTORICAL_GPS110
```

Use the shared launcher instead of a bare `make ... gz_super` command: bypassing
it does not apply the repository's timing configuration. Record the effective
`EKF2_GPS_DELAY` from the ULog, and do not mix 0/110 ms runs in one statistic.
The corrected validation keeps quadratic thrust mapping enabled and Roll/Pitch
compensation disabled; controller Q/R/N and the vehicle/inner-loop/drag models
are unchanged.

Permanent-default validation evidence is
`~/uav_control_ws/ommpc-flight-results/timing-corrected-20261028`.
Eight fresh flights ran **without** `--gps-delay-ms`, while deliberately
inheriting `PX4_PARAM_EKF2_GPS_DELAY=110` in the parent environment. Every ULog
recorded an effective delay of **0 ms**. After the campaign, both
`parameters.bson` and `parameters_backup.bson` contained `EKF2_GPS_DELAY=0`;
the corrected setting was not rolled back. All eight flights passed the
existing flight criteria and landed/disarmed; ULogs had zero dropouts and
controller-used sample timestamps matched the recorded states in 100% of solves.
The shell launchers passed syntax checks; the diagnostic CLI generated the
actual CSV/JSON/PNG/PDF outputs, and the S-curve/Sharp3D figures were inspected.

The comparison below uses the original 110 ms `timing-diagnosis-20261028`
baseline and the same native-clock ground-truth analysis, with no fitted time
shift. RMSE is the mean of two full-phase run RMSEs, not moving-only RMSE;
maximum error is the worse of the two runs. Straight-line trials are opposite
directions, while S-curve and Sharp3D repeat the same reference geometry.

| Case | Position RMSE 110→0 ms (m) | Observed RMSE reduction | Worst position error 110→0 ms (m) |
| --- | --- | ---: | --- |
| Straight line, 1.0 m/s | 0.0898→0.0510 | 43.2% | 0.1869→0.0892 |
| Straight line, 3.9 m/s | 0.3242→0.0515 | 84.1% | 0.5881→0.1321 |
| S-curve, 3.9 m/s | 0.2590→0.0862 | 66.7% | 0.5362→0.2045 |
| Sharp3D, 3.85 m/s | 0.1862→0.0699 | 62.5% | 0.4957→0.2598 |

Moving-only position RMSE became 0.0558, 0.0571, 0.1093 and 0.0922 m,
respectively. Full-phase velocity RMSE became 0.0288, 0.0471, 0.0960 and
0.1539 m/s. There were zero recorded input-bound violations. Maximum recorded
tilt was approximately 0.696 rad; per-run mean solve times were 1.075–1.557 ms,
with a worst observed solve of 6.203 ms versus the unchanged 20 ms period.

The 3.9 m/s cruise equivalent estimator position lead was now about -4.9 and
-12.2 ms, rather than the baseline's +101–112 ms. This geometric diagnostic
is not transport delay. Remaining sample age at solve finish still had
per-run medians of 11.9–34.3 ms; correcting GPS measurement timing does not
remove transport/scheduling age or inner-loop dynamics.

All eight controller hashes remain
`a7d3708d5b7d10a99b5cbe9e39fad7a9fed13e5ad9ed992ea221c2c5e8f5098b`,
and trajectory arguments match the baseline. Initial PX4 parameter differences
were GPS delay, flight counters and, in four pairs, automatic barometer
calibration. Thus these are repeatable observed improvements in this SITL
setup, not universal performance claims or causal error-share estimates.
No state propagation, rate-dynamics model, drag feedforward or Q/R retuning
was introduced. Further changes should be compared against this corrected
0 ms baseline, not the old timing-mismatched baseline.

`configuration_verification.json` preserves the effective/persisted values
and exact parameter differences. `baseline_comparison.json` preserves both
full/moving metrics and peak/timing evidence; each flight retains its complete
ULog, rosbag, CSV and peak-window plots. The corrected default remains
SUPER-SITL-only; real GPS latency still requires measurement.

##### Peak-error audit and optional measured-state prediction

`tools/analyze_state_prediction.py` and `tools/analyze_peak_control.py` audit
the corrected 0 ms baseline before any further controller change. They reuse
the native-clock mapping and fixed pre-arm origin. Output lives outside Git:
`timing-corrected-20261028/<case>/diagnostics` and the campaign's
`peak-control/` directory. Main tracking errors are never shifted to minimize
them.

Findings from the original four turning flights:

- Moving state-position error at control use fell from about 0.088/0.082 to
  0.060/0.064 m in S-curve, and 0.081/0.067 to 0.074/0.062 m in Sharp3D,
  when replaying constant-velocity position and measured-rate attitude
  extrapolation. Roll/Pitch SO(3)-error components also improved.
  This is an offline **state-error** comparison, not a predicted tracking
  improvement. Past-command thrust acceleration helped velocity prediction
  in S-curve but not consistently in Sharp3D, so it was not included.
- Native command-to-gyro effective lag in peak windows was roughly 30–52 ms
  for Roll/Pitch. Yet total commands already led reference feedforward on
  dominant axes by roughly 36–100 ms in diagnostic correlations. The QP
  correction includes state error, nominal discretization defects and horizon
  anticipation (`free_response=H*x0+D`); `command-reference` is **not isolated
  feedback**. No separate feedforward advance was added.
- Fixed straight-line drag coefficient 0.079 s^-1 predicts lateral RMS of
  only about 0.146 m/s² in Sharp3D peak windows versus observed projected
  residual 0.90–1.00 m/s², with poor fits. Straight cruise supports approximately
  0.080–0.081 s^-1. Rotational dynamics, callback timestamps and numerical
  differentiation remain confounded; the full turning residual is not called
  drag. Gazebo truth velocity itself comes from backward position differences.
- Input bounds were not active in the four original turn phases. The discrete
  reference defects were about 0.68–0.86 mm position per step, not an explanation
  by themselves of the 20–26 cm flight peaks.

Native setpoint-to-bag matching uses exact float32 payloads, rejecting ambiguous
repeated payloads rather than choosing the nearest. Command-to-solve association
still has no explicit solve ID: it accepts only one successful completion in
the preceding 10 ms, one-to-one. The original turn windows had 99/99 matched
grid samples; low-excitation straight data can be ambiguous and is reported
as such. New traces supply the exact reference anchor and reference inputs;
`planned_*` never substitutes for an actually matched applied command.

The new `uav_ommpc_ros/src/state_prediction.hpp` applies only:

```text
age = ROS_solve_start - synchronized_odometry_sample
p_used = p_sample + age * v_sample
v_used = v_sample
R_used = R_sample * Exp(age * measured_body_rates)
reference[k] = reference(ROS_solve_start + k*dt)
```

It does not use ideal commanded rates as measured gyro, infer acceleration,
change the MPC dynamics/Q/R/N, compensate actuator delay, or modify thrust/yaw
mapping. State and reference anchors change together; this experiment does not
separately identify their individual contributions. Every cycle predicts from
the cached **raw** measurement, never recursively from the previous prediction.
Fixed-size Eigen state and stack storage avoid per-cycle allocation.

Startup-only ROS parameters:

- `state_prediction_enabled=false` preserves the original path.
- `state_prediction_max_age=0.08` seconds bounds extrapolation and must not
  exceed `odometry_timeout` when enabled.

```bash
ros2 run uav_ommpc_ros ommpc_node --ros-args \
  --params-file ~/uav_control_ws/src/uav-ommpc-controller/uav_ommpc_ros/config/ommpc_super_sitl.yaml \
  -p state_prediction_enabled:=true

python3 tools/run_rate_compensation_ab.py --confirm-sitl --suite state-prediction \
  --repeats 3 --output-root ~/uav_control_ws/ommpc-flight-results/NEW_STATE_AB
```

The suite alternates prediction OFF/ON order on successive repetitions, with
GPS delay 0 ms and bridge rate compensation OFF throughout. Optional `--cases`
selects the existing straight-line controls. The aggregator groups prediction
and bridge compensation flags separately, never pooling OFF/ON state modes.
The timing report distinguishes the raw held-state error from the actually
extrapolated controller-input error at its anchor and at solve finish.

The Release build and ROS package tests passed (`30 tests, 0 failures`).
An isolated real-node smoke exercised both paths, checked the logged anchor
invariants, and verified that future-at-use stamps/nonfinite gyro stop output
in prediction mode while the default bypass remains unchanged.
The analysis tests passed (13 existing timing/projection tests plus 6 new
matching/lag/reference-boundary tests). Reference reconstruction mirrors the
node's one-period prestart tolerance; it does not suppress invalid times or
apply a best-fit shift.

One later validation launch timed out before the flight runner started:
Gazebo from a previously landed flight had survived shutdown. Its log and
`startup_failure.json` are retained in `peak-state-validation-20261028`.
The shared shutdown now waits for the **owned process group**, escalating
SIGINT→SIGTERM→SIGKILL only after confirmed landing; it does not stop unrelated
process groups. A real isolated-process regression covered a signal-resistant
orphan and verified that an unrelated process stayed alive. Recovery runs use
a separate evidence root instead of overwriting the failed launch.

**Repeated validation did not establish a general tracking improvement.
State prediction remains default-OFF; it is not adopted as the new SUPER
baseline.** The favorable first pair was retained, but so were the adverse
repeats and straight-line regression.

Evidence roots under `~/uav_control_ws/ommpc-flight-results`:

- `peak-state-pilot-20261028`: 4 completed pilot flights and the combined
  `tracking_comparison.json` / `final_state_comparison.json`.
- `peak-state-validation-20261028`: 3 completed flights and 1 pre-flight
  simulator-startup failure, preserved separately.
- `peak-state-recovery-20261028`: 9 completed recovery/repeat/control flights.

Total: **16 completed, landed/disarmed flights**, plus the one startup failure
(not counted as a successful flight). Each turn case has 3 OFF and 3 ON runs;
each straight control has only one OFF/ON pair. GPS delay is 0 ms, bridge rate
compensation is OFF, and every controller snapshot has SHA256
`a7d3708d5b7d10a99b5cbe9e39fad7a9fed13e5ad9ed992ea221c2c5e8f5098b`.
Logged prediction durations and reference anchors were checked against the
actual sample timestamps; OFF retains the exact receipt-time anchor.

Native-clock truth results (mean of per-run metrics; `worst peak` is the
largest individual run peak, not the mean of peaks):

| Case | Full-phase position RMSE OFF→ON (m) | Moving-only RMSE OFF→ON (m) | Mean peak OFF→ON (m) | Worst peak OFF→ON (m) |
| --- | --- | --- | --- | --- |
| S-curve, 3.9 m/s, n=3 per mode | 0.0847→0.0943 | 0.1031→0.1141 | 0.1874→0.1970 | 0.2172→0.2113 |
| Sharp3D, 3.85 m/s, n=3 per mode | 0.0915→0.0830 | 0.1137→0.1065 | 0.2469→0.2275 | 0.2938→0.2432 |
| Straight, 1.0 m/s, n=1 per mode | 0.0462→0.0383 | 0.0508→0.0406 | 0.0906→0.0831 | 0.0906→0.0831 |
| Straight, 3.9 m/s, n=1 per mode | 0.0580→0.0691 | 0.0540→0.0604 | 0.1171→0.1098 | 0.1171→0.1098 |

The third Sharp3D pair reversed the pilot benefit: peak 0.1875→0.2420 m.
The third S-curve pair also worsened: 0.1485→0.2113 m. Thus lower worst-case
values in the small aggregate are not proof of a repeatable peak reduction.
Do not retune gains to the favorable pilot or describe this as universally
improved tracking.

Prediction reduced controller-input position error at solve finish on average:
0.0806→0.0684 m in S-curve and 0.0809→0.0522 m in Sharp3D, but this did not
guarantee smaller trajectory error. Moving tangent RMSE fell
0.0741→0.0650 / 0.0836→0.0653 m, while cross-track RMSE increased
0.0713→0.0933 / 0.0770→0.0836 m, respectively. This is not simply a trajectory
that can be fixed by one time shift.

Automatic barometer calibration differed in seven of the eight pairs, in
addition to flight counters; no rate-loop/vehicle parameter changes were
introduced. Horizontal-only RMSE shows the same mixed pattern:
S-curve 0.0734→0.0843 m, Sharp3D 0.0871→0.0788 m.
`parameter_comparison.json` and `horizontal_vertical_comparison.json` preserve
the exact differences. Three turning repeats and single straight pairs do
not establish confidence intervals or causal error percentages.

All completed runs had zero recorded input-bound violations and zero ULog
dropouts. Maximum observed tilt was 0.7162 rad, per-run mean solve time
1.076–1.423 ms and worst solve 4.307 ms. Actual-node smoke, math tests and
process-group regression passed; no claim is made of real-aircraft validation.
One final Sharp3D ON peak window had 8/99 unmatched command-chain grid points:
they remain gaps in feedforward/correction plots. Ground-truth tracking
statistics do not depend on that command association.

Decision: retain the original control path and the proven GPS timing fix.
The remaining inner-loop discrepancy is independently observed, but the QP
already advances total commands relative to nominal reference; adding another
arbitrary FF lead risks double compensation. A subsequent controlled design
should test explicit actual-rate dynamics and consistent velocity propagation
in prediction. The straight-line drag evidence can inform a later consistent
model/reference change, not an inflated coefficient fitted to all turn
residuals. These are next-experiment priorities, not proven error shares.

Reproduce the complete interrupted-campaign aggregation without mixing modes,
double-counting paths, or pooling different controller/GPS-delay configurations:

```bash
ROOT=~/uav_control_ws/ommpc-flight-results
python3 tools/analyze_rate_compensation_tracking.py "$ROOT/peak-state-pilot-20261028" \
  --additional-root "$ROOT/peak-state-validation-20261028" \
  --additional-root "$ROOT/peak-state-recovery-20261028"
```

### Historical PX4/Gazebo results — sampled-reference version

Successful evidence directories on Jetson:

- `~/uav_control_ws/ommpc-flight-results/hover-02`
- `~/uav_control_ws/ommpc-flight-results/tracking-02`

Both runs report `passed=true`, accepted Offboard/arm/land transitions, and confirmed native landing plus disarm. The following are **entire named-phase** metrics; line/circle include ramping and final endpoint settling, while step includes its deliberate target jump.

| Phase | Position RMSE (m) | Maximum error (m) | Velocity RMSE (m/s) | Maximum tilt (rad) | Maximum control gap (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Hover, >=5 s stable | 0.06052 | 0.07557 | 0.01574 | 0.00323 | 0.02806 |
| Target step, `[0.6,0.3,0] m` | 0.30681 | 0.66193 | 0.08321 | 0.03859 | 0.03320 |
| Straight line, 0.3 m/s | 0.05482 | 0.09127 | 0.01438 | 0.01820 | 0.04026 |
| Circle, radius 0.6 m / 0.25 m/s | 0.04470 | 0.10144 | 0.03193 | 0.04728 | 0.06647 |

Every phase observed zero input-bound violations and zero control gaps above 0.1 s. Maximum observed model-estimated speeds were 0.3112 m/s on the straight line and 0.2537 m/s on the circle. The circle phase covered more than one revolution after its smooth start. Controller logs show no solver-failure warnings during the successful tracking sequence; one estimator-reset event during takeoff paused control until the next reference, as designed.

Both successful MCAP bags were inspected: hover 4.3 MiB / 9,120 messages; tracking 15.6 MiB / 29,429 messages. Node logs are copied into the corresponding experiment directories. Failed attempts `hover-01` and `tracking-01` are retained, not overwritten. At the time of those sampled-reference flights, the six-test trajectory suite and 61-test colcon report passed. The new polynomial verification results above are separate; these flight records were not rerun or relabeled as polynomial flights.

These are actual simulated PX4/Gazebo flights through the OMMPC ROS node, **not** merely matched-model closed-loop examples. They establish low-speed SITL behavior only; actuator-model mismatch, estimation, inner-loop delay and hardware safety require separate real-aircraft validation.

## SITL flight test

Use only the simulator for these utilities. Start these prerequisites in separate terminals; the flight test does not launch them:

```bash
# Terminal 1: DDS transport.
MicroXRCEAgent udp4 -p 8888
```

```bash
# Terminal 2: PX4 and headless Gazebo.
source ~/px4-env.bash
cd ~/PX4-Autopilot
HEADLESS=1 make -j4 px4_sitl gz_x500
```

```bash
# Terminal 3: communication node.
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/local_setup.bash
ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.728744
```

A separate test-only GCS heartbeat is needed for PX4's GCS-present safety check:

```bash
cd ~/uav_control_ws/src/uav-ommpc-controller
~/.venvs/px4-v1.17/bin/python tools/sitl_gcs_heartbeat.py
```

This helper sends an actual GCS heartbeat on UDP port 18570, not vehicle commands or parameter changes. In another terminal, source the ROS underlays and new workspace overlay. The flight utility requires NumPy in that ROS Python environment (`sudo apt install python3-numpy` if missing):

```bash
cd ~/uav_control_ws/src/uav-ommpc-controller
python3 tools/sitl_flight_test.py
# Optional: --output-dir /absolute/path/to/results
```

The default result directory is `~/uav_control_ws/flight-test-results`. Consult `--help` for options and generated output for measured results. The utility is a **test controller**, not the user's OMMPC algorithm. It uses telemetry and explicit vehicle commands to exercise the bridge, preserves measured initial yaw, and later requests a yaw offset of +0.25 rad. Never run either test utility against real hardware; no real-hardware verification has been performed.

### Observed SITL verification

On the installed Jetson stack, both packages built successfully in Release mode; the installed bridge is a native ARM64 ELF executable, and the 6 `CommandState` unit tests pass.

A live, **disarmed** check against PX4 SITL (`cpp-run-02/communication.json`) observed:

| PX4 state / input | Heartbeat messages | Rate setpoints |
| --- | ---: | ---: |
| Fresh input, not Offboard (1.5 s) | 76 | 0 |
| Fresh input, Offboard (1.0 s) | 50 | 50 |
| Fresh input, after switching to Hold (1.0 s) | 50 | 0 |
| Input stopped, after timeout (0.5 s) | 0 | 0 |

In Offboard, input `[0.1, -0.2, 0.3]` rad/s at 1 g produced `[0.1, -0.2, 0.3]` and thrust `-0.728744`; input `[4, -5, 2]` rad/s at 30 m/s² was limited to `[3, -3, 1.5]` and thrust `-0.9`.

The gated C++ bridge then passed the unchanged flight acceptance with `hover_thrust=0.728744` and default limits (`cpp-run-02/result.json`, `trace.csv`; earlier evidence remains in `cpp-run-01` and `run-04`). PX4 received 1020 heartbeats but only 893 rate setpoints, consistent with rate setpoints being withheld during the roughly 2 s pre-Offboard stream.

| Observed measurement | Value |
| --- | --- |
| Maximum height above initial ground | 2.056 m |
| Initial hover stable window | 5.039 s |
| Initial hover position RMS error | 0.083 m |
| Second hover stable window | 5.036 s |
| Second hover position RMS error | 0.125 m |
| Second target displacement | North 1.0 m, East 0.5 m, yaw +0.25 rad |
| Final landing | PX4 native NAV_LAND, land detection and ground disarm observed |

All three nonzero body-rate axes and positive thrust were observed at the PX4 ROS input, and actual measured motion satisfied the hold criteria. Offboard, arm and land commands received accepted PX4 acknowledgements. No preflight/failsafe parameters were disabled or force-arming used.

The test keeps odometry freshness at 0.5 s and allows 1.5 s for VehicleStatus: PX4 Commander normally publishes unchanged status at 2 Hz (`Commander.cpp`), so a 0.5 s status timeout would reject normal operation. The 0.60 initial hover estimate did not lift this motor model; the model-derived calibration above is required. Neither the flight script, acceptance criteria nor PX4 safety parameters were changed.