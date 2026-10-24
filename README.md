# UAV OMMPC controller and ROS 2 / PX4 bridge

ROS 2 Jazzy **C++17 / rclcpp** communication packages for PX4 v1.17. A separate controller supplies total thrust acceleration and body angular rates; the bridge relays telemetry and translates commands to PX4 offboard inputs. The Python bridge has been removed; `ros2 run uav_ommpc_ros px4_bridge` and all public topics, messages, QoS and startup parameters remain unchanged. Only the SITL test utilities use Python.

`uav_ommpc_controller` supplies standalone C++17/Eigen mathematical primitives, nonlinear single-step prediction, discrete local-error linearization and a condensed OMMPC optimizer using OSQP. The optimizer is a library, **not wired to ROS/PX4**; its included tracking executable exercises only the nonlinear mathematical model.

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

On Jetson, all 13 SO(3)/R³ tests passed in both standalone Release and colcon builds. Coverage includes hat/vee and cross products, Euclidean errors, right-handed quarter turns, zero/tiny rotations, the Taylor-branch boundary, pi and both sides of its branch cut, 500 seeded rotation round trips, right-local error direction, finite-difference Jacobians, vector-action derivatives, and 10,000 consecutive body-rate updates.

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

## Discrete local-error linearization

Header: `uav_ommpc_controller/include/uav_ommpc_controller/linearization.hpp`.
Namespace: `uav_ommpc_controller::linearization`. The implementation is header-only, uses fixed-size double-precision Eigen matrices, and reuses `dynamic.hpp` and `so3_math.hpp`.

### Error coordinates and API

The 9-dimensional `StateError` is ordered as
`[delta_p_world(3), delta_v_world(3), delta_theta_local(3)]`, where
`delta_theta_local = Log(R_ref.transpose() * R)`. The 4-dimensional `InputError` is
`[delta_thrust_acceleration, delta_omega_body(3)]`. Both errors are **actual minus reference**. Rotation errors are in radians, not Euler-angle differences or nine flattened rotation-matrix entries. Position/velocity errors are in metres and m/s.

| API | Purpose |
| --- | --- |
| `state_error(actual, reference)` | Extract the compound R³ × R³ × SO(3) local state error |
| `apply_state_error(reference, error)` | Reconstruct a state with `p_ref + dp`, `v_ref + dv`, `R_ref * Exp(dtheta)` |
| `linearize(reference, reference_input, dt)` | Return `LinearModel` containing `A` (9×9) and `B` (9×4) |
| `model.predict(delta_x, delta_u)` | Compute `delta_x_next = A * delta_x + B * delta_u` |

The next error is measured about **the propagated reference**
`reference_next = dynamics::predict_step(reference, reference_input, dt)`, not the old reference. Consequently, reconstruction must use `apply_state_error(reference_next, delta_x_next)`. The same gravity value must be used for reference and actual nonlinear propagation; its additive acceleration cancels from the Jacobians, so `linearize` needs no gravity argument.

### Discrete Jacobians

Let `R`, `a_T` and `omega` denote the reference attitude and input, `h = dt`,
`e3 = [0, 0, 1]ᵀ` and `phi = h * omega`. The matrices are:

```text
    [ I   h*I   0                  ]        [ 0          0          ]
A = [ 0   I     h*a_T*R*hat(e3)     ],   B = [ -h*R*e3    0          ]
    [ 0   0     Exp(-phi)          ]        [ 0          h*J_r(phi) ]
```

All unshown blocks are zero. `J_r(phi) = J_l(phi).transpose()` uses the existing SO(3) right Jacobian. The orientation block `Exp(-phi)` transports the current right-local attitude error into the next reference's local coordinates. The signs in the velocity blocks follow `v_dot = g - a_T * R * e3`, consistent with the project's NED/FRD convention.

These are analytic derivatives of the **implemented discrete** nonlinear step, not the approximation `A = I + h*A_continuous` for every block. In particular, the attitude blocks retain the exponential and right Jacobian at nonzero angular rates. Position has no same-step input/attitude coupling because `dynamic.hpp` uses `p_next = p + h*v`; gravity and the reference position/velocity do not affect the Jacobians.

The approximation is
`delta_x_next = A*delta_x + B*delta_u + O(||[delta_x, delta_u]||²)`
locally about zero error. There is no affine reference defect: this API assumes the next reference is generated by the same nonlinear step. An arbitrary externally supplied next reference that violates that dynamics is not supported by simply changing the reconstruction reference; the error chart and linearization would need to be reconsidered. For a time-varying trajectory, recompute the model at each reference state/input and propagate errors between those successive reference charts.

### Usage

```cpp
#include <uav_ommpc_controller/linearization.hpp>

namespace dyn = uav_ommpc_controller::dynamics;
namespace lin = uav_ommpc_controller::linearization;
const double dt = 0.01;

// actual/reference are dyn::State; input/reference_input are dyn::ControlInput.
const auto model = lin::linearize(reference, reference_input, dt);
const lin::StateError dx = lin::state_error(actual, reference);
lin::InputError du;
du << input.thrust_acceleration - reference_input.thrust_acceleration,
  input.body_rates - reference_input.body_rates;

const lin::StateError dx_next = model.predict(dx, du);
const auto reference_next = dyn::predict_step(reference, reference_input, dt);
const dyn::State predicted_next = lin::apply_state_error(reference_next, dx_next);
```

The same finite-input, valid-SO(3), nonnegative-time and physical-input preconditions as `dynamic.hpp` apply; no validation, clamping or matrix repair is performed. Error perturbations must be small, and the principal attitude chart must stay away from its pi branch cut. Nominal orientation need not be near identity: large absolute attitudes are valid. `apply_state_error` preserves the rotation-manifold structure but does not turn the first-order model into an exact nonlinear predictor.

### Sources and verification

The author [preprint](https://arxiv.org/html/2106.15233v1) defines actual-minus-reference errors in (17)–(18), the discrete error linearization in (20)–(23), manifold transport in Table I, and quadrotor derivatives in (34). Equation numbers here refer to the preprint, not verified journal-version numbering. The reference repository's [`setStateMatricesAndBounds`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_core/src/controller_core.cpp) supplies the same block structure and `J_l(phi).transpose()` rotation-input block; its body +Z thrust signs are not copied into this NED/FRD model.

All 8 linearization tests pass on Jetson in standalone Release and colcon builds. They check chart packing/reconstruction, zero time, hover tilt/thrust signs, transport to the next tangent frame, all columns of the 9×9 and 9×4 Jacobians against nonlinear central differences, zero-thrust boundary behavior, second-order joint-perturbation residuals, and 20 chained models along a moving reference. Central differences include zero/tiny rates, both sides of the SO(3) Taylor threshold, a step rotation near pi, a nominal attitude near pi, and different common gravity values; maximum-entry discrepancies are below `3e-9`.

```bash
ctest --test-dir ~/uav_control_ws/so3-math-build -R '^Linearization\.' --output-on-failure
```

An independent program compiled against the installed header without sourcing ROS, with Eigen heap allocation disabled and assertions enabled. It reconstructed full predicted states and compared them with nonlinear steps at joint-perturbation scales `0.02`, `0.01`, `0.005`. Velocity residuals were `5.4873e-5`, `1.3715e-5`, `3.4285e-6` m/s, and attitude residuals were `2.3841e-6`, `5.9603e-7`, `1.4901e-7` rad: each halving reduced these residuals by approximately four. Position errors were below `6.3e-16 m`; reconstructed-rotation orthogonality errors were below `4.7e-16`. The temporary consumer was removed afterward. This is mathematical-model verification, not flight or MPC-solver validation.

## Condensed on-manifold MPC

Implementation: `uav_ommpc_controller/src/ommpc.cpp`; API: `include/uav_ommpc_controller/ommpc.hpp`.
Namespace: `uav_ommpc_controller::ommpc`. `Controller` owns one persistent OSQP workspace and fixed-horizon buffers. Construct it once per control loop; it is noncopyable and not thread-safe. Settings are copied at construction, not changed mid-solve.

### Reference and error contract

`solve(actual, reference, feedforward)` takes one measured/predicted `dynamics::State`, **N+1 reference states** (`x_ref,0` through `x_ref,N`) and **N feedforward inputs** (`u_ref,0` through `u_ref,N-1`), all at `Settings::dt`.

- Frames, units, right-local attitude error and actual-minus-reference signs match the earlier modules.
- Each reference transition must satisfy `dynamic.hpp` with the configured gravity. Position, velocity and rotation-vector defect norms must each be at most `1e-6`. Reference inputs must lie inside the configured box.
- The controller rejects mismatched dimensions, nonfinite data, invalid rotation matrices, out-of-box references and dynamically inconsistent reference transitions. It does not silently regenerate or repair the trajectory.
- A position-only trajectory is not a complete reference: velocity, attitude, thrust acceleration and body rates are required. `examples/ommpc_tracking.cpp` demonstrates constructing them consistently using forward position/velocity differences, `R*e3 = (g-a)/||g-a||`, `a_T = ||g-a||`, and `omega = Log(R_k.transpose()*R_{k+1})/dt`.

As with `linearization.hpp`, this is a local one-linearization-per-cycle MPC. Small tracking errors and dynamically feasible references are assumed; no nonlinear SQP iterations or affine trajectory-defect model are added.

### Condensation and QP

From each reference state/input, obtain `A_k, B_k` using `linearization::linearize`. Stack
`delta_X = [delta_x1; ...; delta_xN]` and `delta_U = [delta_u0; ...; delta_uN-1]`.
The recursive coefficients are:

```text
H_0 = A_0                         M_0,0 = B_0
H_k = A_k * H_{k-1}               M_k,j = A_k * M_{k-1,j}   (j < k)
                                  M_k,k = B_k               (j > k: zero)
delta_X = H * delta_x0 + M * delta_U
```

The stage cost penalizes state errors `x1..xN-1`, terminal error `xN`, and input errors `u0..uN-1`. The cost of the fixed initial state `x0` is constant and omitted:

```text
Qbar = diag(Q, ..., Q, Q_terminal)     Rbar = diag(R, ..., R)
P = 2 * (M.transpose() * Qbar * M + Rbar)
q = 2 * M.transpose() * Qbar * H * delta_x0

minimize  0.5 * delta_U.transpose() * P * delta_U + q.transpose() * delta_U
subject to  u_min - u_ref,k <= delta_u_k <= u_max - u_ref,k,  k = 0..N-1
```

Only **4N decision variables** remain; state equalities are eliminated rather than passed to OSQP. Positive input weights keep the Hessian positive definite even if state weights are zero. `problem()` exposes `M`, `H`, `P`, `q`, `lower`, `upper` for inspection after a successful solve. `input_errors()` and `predicted_errors()` expose the optimized stacked errors, owned by the controller and overwritten by the next call.

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

All **39 tests** passed in standalone Release and colcon builds: the earlier 29 math/model tests plus 10 MPC tests. MPC coverage includes every condensed prediction column versus recursive time-varying propagation, Hessian/gradient/terminal indexing and factor-of-two conventions versus recursive cost, comparison to an unconstrained LDLT optimum, hover feedforward, active input bounds, changing-reference workspace reuse, invalid input/reference rejection and clearing stale solutions, and forced one-iteration solver failure.

```bash
ctest --test-dir ~/uav_control_ws/so3-math-build -R '^Ommpc\.' --output-on-failure
~/uav_control_ws/install/uav_ommpc_controller/bin/ommpc_tracking
# Or use ~/uav_control_ws/so3-math-build/ommpc_tracking.
# Disable building this example with -DBUILD_EXAMPLES=OFF.
```

`ommpc_tracking` simulates a 12-second, 5 m radius circle at analytic speed 5 m/s and fixed 2 m altitude/yaw. It uses the **nonlinear** `dynamic.hpp` plant, exact discrete reference transitions, initial position error `[0.2, -0.15, 0.1] m`, and nonzero velocity/attitude errors. It stops on solver failure, invalid output or bound violation and returns failure if post-2-second position RMSE exceeds 0.15 m. Latency is measured over complete `Controller::solve` calls (including validation, linearization, condensation, QP updates and solve), not OSQP iterations alone. It reports 20 ms deadline misses but does not run as a paced real-time process.

Observed final installed Release executable on Jetson:

| Measurement | Result |
| --- | ---: |
| Completed solve calls / failures | 600 / 0 |
| Position RMSE, entire run | 0.049724555 m |
| Position RMSE, after 2 s | 0.000006082 m |
| Maximum position error, entire run | 0.277367370 m |
| Maximum position error, after 2 s | 0.000059362 m |
| Peak model speed | 5.525948859 m/s |
| Maximum input-bound violation | 0.000006914 |
| Solve-call median / p95 / maximum | 0.820637 / 1.144456 / 2.331090 ms |
| Solve calls above 20 ms | 0 |

The near-zero steady-state error is a **matched-model, disturbance-free simulation result**, not a claim of micrometre flight accuracy. No ROS/PX4/SITL or real aircraft was exercised by this example. An independent CMake consumer also linked the installed exported target without ROS and obtained correctly directed corrective thrust/rates.

The condensed formulation follows the author [preprint](https://arxiv.org/html/2106.15233v1), Section V-C, equations (25)–(30), using the existing actual-minus-reference error convention. The reference repository's [`mpc_wrapper.hpp`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_core/include/ommpc_core/mpc_wrapper.hpp) uses an uncondensed sparse QP; this implementation intentionally follows the paper's **condensed** form and reuses its OSQP API rather than copying the per-cycle setup/cleanup pattern. Baseline weights come from [`params.yaml`](https://github.com/OliverWu515/ommpc_controller/blob/refact-decouple/ommpc_ros/config/params.yaml).

## Build and environment

Dependencies on this Jetson: ROS 2 Jazzy (`/opt/ros/jazzy`), `px4_msgs` release/1.17 in `~/ros2_px4_ws`, PX4 v1.17.0 in `~/PX4-Autopilot`, Gazebo Harmonic and Micro XRCE-DDS Agent v2.4.3. Build the PX4 message workspace before this overlay. Message definitions must match PX4.

`uav_ommpc_ros` uses `ament_cmake` and links against `rclcpp`, `px4_msgs` and `uav_ommpc_interfaces`. When migrating an already-built Python workspace, remove its obsolete generated `build/uav_ommpc_ros` and `install/uav_ommpc_ros` directories before rebuilding; source the ROS/px4_msgs underlays, not the old project overlay.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_px4_ws
colcon build --symlink-install --packages-select px4_msgs
source ~/ros2_px4_ws/install/setup.bash
cd ~/uav_control_ws
colcon build --symlink-install --packages-select uav_ommpc_interfaces uav_ommpc_ros \
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

Input validation, limits and timeout logic live in `uav_ommpc_ros/src/command_state.hpp` and are tested with injected time (no ROS or PX4 needed):

```bash
cd ~/uav_control_ws
colcon test --packages-select uav_ommpc_ros
colcon test-result --verbose --test-result-base build/uav_ommpc_ros
```

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
