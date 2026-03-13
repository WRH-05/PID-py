# Full Python PID Library Blueprint (Microcontroller + I/O + Optional PID→RST Modeling)

This document is a **practical, step-by-step implementation guide** for building a full-featured PID library in Python that can run on typical development board microcontrollers (through MicroPython/CircuitPython where possible) and can also run on desktop Python for tuning, simulation, logging, and model generation.

It is intentionally split into two tracks:

- **Embedded Track** (lightweight, real-time, microcontroller-safe)
- **Host Track** (heavier analysis, plotting, model fitting, optional RST tools)

That split is the key to making this robust and maintainable.

---

## 0) Feasibility and Important Reality Check

Your idea is good and possible.

- A **full PID library with rich I/O support** is very realistic.
- A complete **"collect data → identify model → generate RST parameters"** workflow is also realistic, but usually best done mostly on a host PC due to CPU/RAM constraints.
- On small boards, you usually run:
  - fast control loop,
  - safe logging,
  - basic telemetry.
- On the host PC, you run:
  - model fitting,
  - optimization,
  - RST polynomial solving,
  - plots and report generation.

So the right design is **one library with modular subpackages**, where microcontroller code and host tools share data structures and file format.

---

## 1) Define Scope and Feature Set First

Before coding, define exactly what “full functionality” means.

### Core control features

1. P, I, D with runtime gain changes
2. Sample-time aware update (`dt`)
3. Output limits
4. Integrator limits (anti-windup)
5. Derivative-on-measurement (reduces setpoint kick)
6. Derivative low-pass filtering
7. Setpoint weighting (`beta`, optional `gamma`)
8. Setpoint ramp/slew limiter
9. Manual/auto mode with bumpless transfer
10. Optional feedforward term
11. Reverse/direct action mode

### Embedded I/O features

1. Sensor input adapters (ADC, I2C, SPI, UART)
2. Actuator output adapters (PWM, DAC, GPIO, serial command)
3. Timestamp source abstraction (ticks ms/us)
4. Fault detection hooks (sensor timeout, range error)
5. Optional command channel (UART/CAN/Wi-Fi)

### Data and modeling features

1. Fast in-loop logging with ring buffer
2. Flush logs as CSV or binary packets
3. Session metadata (gains, limits, board, sensor scale)
4. Host-side identification utilities
5. Optional RST design helper (ARX/transfer function → R,S,T)
6. One-command workflow function to go from logs to model summary

---

## 2) Recommended Project Layout

Use this structure:

```text
pidlib/
  __init__.py
  core/
    pid.py               # PID class, anti-windup, filters, modes
    filters.py           # LPF, moving average (small and fast)
    limits.py            # clamp, slew limit helpers
    types.py             # dataclasses / typed config
  io/
    base.py              # Sensor/Actuator/Clock interfaces
    adc_sensor.py
    encoder_sensor.py
    pwm_actuator.py
    uart_transport.py
  runtime/
    loop.py              # ControlLoop runner (fixed dt or timed)
    safety.py            # fault handling policies
  log/
    ring_buffer.py
    recorder.py          # write CSV/JSONL/binary
    schema.py            # sample and metadata schema
  model/
    dataset.py           # load/clean/sync
    id_arx.py            # ARX fit (light)
    id_tf.py             # transfer function fit (host)
    rst.py               # optional RST solver helpers
    workflow.py          # one-shot: logs -> model -> controller hints
  examples/
    mcu_dc_motor.py
    mcu_temp_control.py
    host_replay_and_tune.py
    host_pid_to_rst.py
  tests/
    test_pid_core.py
    test_anti_windup.py
    test_dt_jitter.py
    test_logging_schema.py
```

If you want one package for both microcontrollers and desktop:

- Keep `core`, `io/base`, `runtime`, and minimal logging microcontroller-safe.
- Put NumPy/SciPy-only features behind guarded imports in `model`.

---

## 3) Step-by-Step Build Plan

## Step 1 — Create strict control data model

Define small dataclasses (or lightweight classes if MicroPython constraints require it):

- `PIDConfig`: `kp`, `ki`, `kd`, `output_min`, `output_max`, `i_min`, `i_max`, `d_filter_alpha`, `beta`, `direction`, `sample_time_s`
- `PIDState`: `integral`, `prev_error`, `prev_measurement`, `prev_output`, `last_t`
- `PIDTerms`: `p`, `i`, `d`, `ff`, `u_raw`, `u_sat`

Why this matters:

- clear debugging,
- deterministic serialization,
- easier log replay and tuning.

## Step 2 — Implement robust PID core

Design method signature:

```python
def update(self, setpoint: float, measurement: float, now_s: float | None = None,
           feedforward: float = 0.0) -> float:
    ...
```

Algorithm details (recommended):

1. Compute `dt`; if no timestamp given, use configured fixed sample time.
2. Error: `e = setpoint - measurement` (or invert for reverse action).
3. Proportional term with setpoint weighting:
   - `p = kp * (beta * setpoint - measurement)`
4. Integral term:
   - `integral += ki * e * dt`
   - clamp integral to `[i_min, i_max]`
5. Derivative on measurement:
   - `raw_d = -(measurement - prev_measurement) / dt`
   - LPF: `d_filt = alpha * prev_d + (1-alpha) * raw_d`
   - `d = kd * d_filt`
6. Raw command: `u_raw = p + integral + d + feedforward`
7. Saturate: `u = clamp(u_raw, output_min, output_max)`
8. Optional anti-windup back-calculation:
   - `integral += kaw * (u - u_raw) * dt`
9. Save state, return `u`.

Add control methods:

- `set_gains(kp, ki, kd)`
- `set_output_limits(min_u, max_u)`
- `set_integral_limits(i_min, i_max)`
- `set_mode(auto: bool, manual_output: float | None)`
- `reset(integral=0.0, output=0.0)`

## Step 3 — Add timing and loop infrastructure

Create `ControlLoop` that handles:

- fixed period execution,
- sensor read,
- controller update,
- actuator write,
- logging callback.

Pseudo:

```python
while loop_running:
    t = clock.now_s()
    y = sensor.read()
    u = pid.update(sp, y, now_s=t)
    actuator.write(u)
    logger.sample(t=t, sp=sp, y=y, u=u)
    clock.sleep_until_next_period()
```

Important for microcontrollers:

- avoid allocations in loop,
- pre-allocate buffers,
- avoid slow string formatting each cycle.

## Step 4 — Build I/O abstraction layer

Use small interfaces:

```python
class Sensor:
    def read(self) -> float: ...

class Actuator:
    def write(self, value: float) -> None: ...

class Clock:
    def now_s(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...
```

Then implement adapters:

- `ADCSensor(pin, scale, offset, filter=None)`
- `EncoderSensor(timer_or_counter, counts_per_unit)`
- `PWMMotor(pin_pwm, pin_dir=None, duty_min=0, duty_max=65535)`
- `SerialActuator(uart, formatter)`

This gives portable logic and board-specific implementations.

## Step 5 — Add safety and fault handling

Include policies:

- sensor value out of range,
- stale sensor timestamp,
- actuator saturation timeout,
- emergency stop input.

Suggested behavior:

- on fault: disable auto mode and set safe output.
- expose status flags for telemetry.

## Step 6 — Build data logging that works on MCU and host

Minimum sample schema:

- `t` (seconds)
- `sp` (setpoint)
- `y` (measurement)
- `u` (controller output)
- optional `p`, `i`, `d`, `sat`, `fault`

Use ring buffer on board:

- fixed-size list/array,
- overwrite oldest or stop when full (configurable),
- flush in lower-priority task.

Export formats:

- CSV (human-friendly),
- binary packets (fast, lower bandwidth),
- JSONL metadata header once per run.

## Step 7 — Provide host-side dataset loader/cleaner

`model/dataset.py` should:

1. Load logs from CSV/binary
2. Validate required columns
3. Resample or align to fixed `dt`
4. Drop or mark bad samples
5. Compute helper columns (error, du, dy)

Return clean structure used by both PID tuning and RST functions.

## Step 8 — Implement identification utilities

Start simple and robust:

- ARX model fitting using least squares
- optional 1st/2nd order transfer function approximation

Practical baseline ARX form:

$$A(q^{-1})y(k)=B(q^{-1})u(k-n_k)+e(k)$$

where you estimate A and B coefficients from logged data.

For microcontroller compatibility:

- keep fitting on host side by default,
- allow lightweight fallback fitter for tiny datasets.

## Step 9 — Optional RST module design

RST controllers are often designed from a discrete plant model:

$$R(q^{-1})u(k)=T(q^{-1})r(k)-S(q^{-1})y(k)$$

You can add an `rst.py` module that:

1. Accepts identified model coefficients
2. Accepts desired closed-loop polynomial specs
3. Solves Diophantine equation for `R` and `S`
4. Computes `T` for reference tracking gain
5. Returns implementable difference equation coefficients

Reality check:

- this is absolutely possible on host Python,
- not always practical to run full symbolic/numeric solving on small MCU,
- best workflow: **fit + RST design on host**, deploy coefficients to board.

## Step 10 — Build one-function workflow (your key request)

Create a high-level function like:

```python
result = pidlib.model.workflow.collect_and_model(
    source="serial://COM5",
    duration_s=20,
    sample_rate_hz=100,
    model_type="arx",
    rst=True,
)
```

Expected outputs:

- cleaned dataset,
- identified model coefficients,
- quality metrics (MSE, fit%),
- optional RST coefficients,
- suggested PID retune hints,
- export files for deployment.

This gives the simple pipeline you asked for:

1. collect data,
2. run one function,
3. get model/RST/controller values.

---

## 4) Reference API Design (Concrete)

## Core control API

```python
pid = PID(
    kp=1.2, ki=0.4, kd=0.05,
    sample_time_s=0.01,
    output_limits=(-1.0, 1.0),
    integral_limits=(-0.5, 0.5),
    derivative_filter_alpha=0.85,
    setpoint_weight_beta=1.0,
    anti_windup="back_calculation",
    kaw=0.3,
)

u = pid.update(setpoint=target, measurement=meas, now_s=t)
```

## Runtime loop API

```python
loop = ControlLoop(
    controller=pid,
    sensor=sensor,
    actuator=actuator,
    clock=clock,
    period_s=0.01,
    logger=logger,
)

loop.run(duration_s=10)
```

## Logging + model + optional RST API

```python
summary = model_from_log(
    log_path="run_001.csv",
    model_type="arx",
    arx_orders=(2, 2, 1),
    do_rst=True,
    rst_spec={"settling_s": 0.8, "damping": 0.75}
)
```

---

## 5) Typical Microcontroller Usage Example

## Example A — DC motor speed loop (MicroPython-style)

```python
from pidlib.core.pid import PID
from pidlib.runtime.loop import ControlLoop
from pidlib.io.adc_sensor import ADCSensor
from pidlib.io.pwm_actuator import PWMMotor
from pidlib.log.recorder import Recorder

pid = PID(
    kp=0.8, ki=0.2, kd=0.02,
    sample_time_s=0.01,
    output_limits=(0.0, 1.0),
    integral_limits=(0.0, 0.6),
)

sensor = ADCSensor(pin=26, scale=3000.0/65535.0, offset=0.0)  # e.g., RPM mapping
actuator = PWMMotor(pin_pwm=15)
logger = Recorder(capacity=2000)

loop = ControlLoop(pid, sensor, actuator, period_s=0.01, logger=logger)
loop.set_setpoint(1200.0)
loop.run(duration_s=8.0)
logger.flush_csv("motor_run.csv")
```

## Example B — Closed-loop temperature control

- Sensor: thermistor/RTD via ADC
- Actuator: SSR duty cycle (slow PWM)
- Sample period: 0.5 to 2.0 seconds
- Derivative often disabled or heavily filtered

---

## 6) Host-Side Data→Model→RST Example Workflow

```python
from pidlib.model.workflow import collect_then_identify

result = collect_then_identify(
    transport="serial://COM7",
    duration_s=30,
    sample_rate_hz=100,
    channels=["t", "sp", "y", "u"],
    model_type="arx",
    model_orders=(2, 2, 1),
    compute_rst=True,
    rst_target={"wn": 8.0, "zeta": 0.7},
)

print(result.model)
print(result.rst if result.rst is not None else "RST skipped")
print(result.metrics)
```

Expected output (conceptually):

- model: identified A/B polynomials
- rst: R, S, T coefficients (optional)
- metrics: fit %, RMSE, stability flags
- artifacts: cleaned CSV, JSON report

---

## 7) How to Keep “Many Options” Without Making It Hard to Use

Use three API tiers.

### Tier 1 — Super simple

```python
u = pid_simple_step(sp, y)
```

Minimal args, default settings.

### Tier 2 — Practical defaults

Instantiate class with common safety defaults and limits.

### Tier 3 — Expert mode

Enable advanced knobs:

- anti-windup method selection,
- derivative source and filter,
- setpoint weighting,
- feedforward channel,
- model-assisted tuning,
- optional RST design options.

This keeps beginner onboarding easy while preserving full flexibility.

---

## 8) Testing Plan (Very Important)

Create deterministic tests before hardware tests.

1. **Unit tests**
   - P/I/D term correctness
   - saturation behavior
   - anti-windup behavior
   - manual/auto transitions
2. **Numerical stability tests**
   - small `dt`, large `dt`, jitter
   - sensor noise impact on derivative
3. **Simulation tests**
   - first-order plant step response
   - second-order plant with delay
4. **Hardware-in-the-loop tests**
   - verify loop timing
   - fail-safe behavior on disconnected sensor
5. **Model workflow tests**
   - known synthetic dataset returns expected ARX coefficients
   - RST solver sanity checks

---

## 9) Performance and Portability Guidelines

For microcontrollers:

- avoid dynamic allocation in tight loop,
- avoid pandas/matplotlib/scipy on-device,
- use integer/fixed-point option if board has no FPU,
- keep ISR code minimal; run control at scheduled task level unless strict need.

For host tools:

- freely use NumPy/SciPy/pandas,
- produce plots and reports,
- generate deployment-ready coefficient files.

---

## 10) Suggested Implementation Order (Practical Roadmap)

Build in this exact order:

1. `core/pid.py` (basic + limits)
2. add derivative filter + anti-windup
3. `runtime/loop.py`
4. basic sensor/actuator adapters
5. ring-buffer logger + CSV export
6. example app on one real board
7. host dataset loader + ARX fitter
8. add one-command modeling workflow
9. optional RST solver module
10. docs + examples + tests expansion

Do not start with RST first; start by stabilizing PID + logging pipeline.

---

## 11) RST Decision Guidance (When to Include vs Skip)

Include RST now if:

- you can run host-side tooling,
- you care about formal polynomial control design,
- you want reproducible model-based workflows.

Skip RST for first release if:

- you need quick deploy on tiny MCUs,
- your users mostly need practical PID only,
- your logs/timing infrastructure is not mature yet.

Good compromise:

- ship PID + logging first,
- add RST as **optional submodule** in v2.

---

## 12) Minimal Viable API You Can Implement First

If you want to start now with a clean core:

- `PID.update()`
- `PID.set_mode()`
- `ControlLoop.run()`
- `Recorder.sample()` + `flush_csv()`
- `fit_arx_from_csv()`
- `collect_and_model()` wrapper (RST optional flag)

That already delivers your key differentiator:

> collect PID data easily, run one function, get model output and optional RST coefficients.

---

## 13) Packaging and Documentation Recommendations

- Package name: `pidlib`
- Keep docs with:
  - quickstart,
  - board-specific setup notes,
  - tuning cookbook,
  - data-modeling cookbook,
  - optional RST guide.
- Add `examples/` that can run unmodified with common boards.
- Provide serial protocol spec for telemetry/logging.

---

## 14) Final Guidance

Your full vision is feasible if you architect it in layers.

- **Definitely do PID + I/O + logging** in the base library.
- **Do RST as an optional host-side module**, integrated into the same workflow function.
- Keep one consistent data format from board to host so your “simple workflow” stays simple.

If done this way, you can produce a library that is practical for beginners and powerful for advanced control users.
