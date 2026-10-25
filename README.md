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
- Each accepted odometry packet records its **ROS reception time**. The node pairs that state with reference samples at reception time plus `k*dt`, without floor indexing. PX4 boot timestamps are used only for freshness/replay checks. This does not compensate transport delay before receipt or predict an old state forward using an assumed actuator input.
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
- The installed PX4 message labels `quality` as unused; its default zero is **not** used to reject otherwise valid odometry. Measured angular velocity and covariance are not consumed by this p/v/R model.
- PX4 `timestamp_sample` must be positive, no greater than the message timestamp, and strictly advance before refreshing the odometry cache. Status timestamps must also advance. These boot-time values are never compared to ROS wall time. Duplicates/out-of-order packets do not refresh freshness. Restart the node after PX4 reboots and its timestamps return to a smaller epoch.
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

The current node does not publish per-solve iteration/latency diagnostics. Stable solving is evidenced here by actual continuous OMMPC output, gap metrics and retained node logs; these are not fabricated per-QP timing or failure counters.

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
cd ~/PX4-Autopilot
HEADLESS=1 make -j4 px4_sitl gz_super

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