# Kart Physics Model – Equations and Assumptions

All quantities are SI. The axis system is ISO 8855: x forward, y left, z up. The body frame
origin is the combined kart + driver centre of gravity (CoG).

This document describes **what is implemented**. Anything not implemented yet is listed as a
limitation in §10.

---

## 1. Integration scheme

* Outer step h = 1/240 s, split into n = 4 substeps (Δt = 1/960 s).
* Integration is **semi-implicit (symplectic) Euler**. Velocities are updated from forces
  evaluated at the current state, and positions are then updated with the new velocities.
* The transient tyre slip states use an **implicit** first-order update, which is
  unconditionally stable (§3.2).
* Wheel/axle/engine spin uses explicit torque integration with **Coulomb clamping** for brakes
  and a **stick/slip state machine** for the clutch (§5), so locking cannot overshoot or reverse.
* **Two stability measures, both found by the automated tests:**
  1. The tyre's low-speed viscous damping is applied to wheel spin **implicitly** (linearised
     backward Euler: I_eff = I + Δt·D·R²). Treated explicitly, its rate D·R²/I ≈ 2,100 s⁻¹ gave
     rate·Δt ≈ 2.2 at 960 Hz. That is past the explicit-Euler limit, and it produced a standstill limit cycle
     with a ±430 N phantom force.
  2. The tyre is split into `Evaluate` (forces from the current state) and `Integrate` (slip state
     advanced with the **post-update** wheel speed). This symplectic ordering removed a weakly unstable
     54 Hz wheel/tyre oscillation seen during coast-down at about 8 m/s.

## 2. Chassis

### 2.1 Rigid body

The chassis is one rigid body of mass *m* with diagonal body-frame inertia
I = diag(Ixx, Iyy, Izz):

* m·v̇ = Σ F_tyre + m·g + F_aero
* I·ω̇ = Σ τ − ω × (I·ω)  (Euler's equations, body frame)
* q̇ = ½ q ⊗ (0, ω)  (followed by renormalisation)

Karts have **no suspension**. All vertical compliance comes from:
1. tyre radial stiffness (§3.1), and
2. chassis torsional compliance (§2.2).

### 2.2 Chassis torsional flex (two twist DOFs, Milestone 2)

**Change in M2:** M1 put all frame compliance at the front, which made the rear axle effectively rigid to the
main body. About 90 % of lateral load transfer then went to the rear axle, overloading the outside
rear and causing oversteer and spins well below the grip limit (diagnosed with the driver-bot test). M2 adds
a **rear-section twist DOF** with the same equations about a point at the rear axle. The measured
front-to-rear frame stiffness is the series combination 1/k_frame = 1/k_front + 1/k_rear. Splitting the
literature 18,000 Nm/rad equally (36,000 Nm/rad each, an ASSUMPTION) gives a front/rear lateral transfer
split of roughly 45/55. Inside-rear lift still emerges, but now mostly through caster jacking at larger
steering angles (about 1.75 g in the step-steer tests).

The original single-section description follows; it applies to each section.

The ladder frame twists mostly between the front section (stub axles, front wheels) and the
heavy rear/centre section (seat, driver, engine, rear axle). This is modelled as a single
rotational DOF φ: the front section rotates relative to the main body about the body x-axis,
through the twist-axis point (front axle x, y = 0, frame height).

* I_φ·φ̈ = M_front,x − k_t·φ − c_t·φ̇

where M_front,x is the x-moment of both front tyre forces about the twist axis. The main body
receives the front tyre forces in full, but the x-moment they carry about the twist axis is
replaced by the spring moment (k_t·φ + c_t·φ̇). This makes the frame torsion spring sit **in
series** with the front tyre springs in the warp mode.

Consequence: weight-jacking from caster (§4.2) and lateral load transfer are shared between the
front and rear axles according to k_t, tyre stiffnesses and track widths. **Inside-rear wheel
unloading and lift are not scripted.** A rear wheel's vertical force is
max(0, k·δ + c·δ̇), and it reaches zero when the geometry and loads say so.

### 2.3 Aerodynamics

F_aero = −½·ρ·CdA·|v|·v, applied at the CoG. Aero pitch moment and lift are not modelled
(negligible for karts at < 130 km/h; this is an ASSUMPTION).

## 3. Tyres

### 3.1 Contact geometry and vertical force

The wheel is a disc with centre **c**, unit axle axis **a** and unloaded radius R₀. A ground
query below the wheel returns a contact plane (point **g**, normal **n**) and a surface.

* centre height above plane: s = (c − g)·n
* disc reach toward plane: r_n = R₀·√(1 − (n·a)²)
* penetration: δ = r_n − s
* contact point: p = c − R₀·û, where û = normalise(n − (n·a)·a)
* vertical force: F_z = max(0, k_r·δ + c_r·δ̇)  (zero when δ ≤ 0 → wheel lifted)

Effective rolling radius: R_e = R₀ − δ/3 (standard approximation for radial-ish carcasses;
this is an ESTIMATE).

The contact half-length for the brush model comes from the chord of the deflected circle:
a = √(2R₀δ − δ²), with a lower bound of 5 mm.

### 3.2 Transient slip (relaxation length)

Steady-state slip definitions are singular at standstill, so first-order lagged slip states are
used instead (Pacejka, *Tyre and Vehicle Dynamics*, ch. 7; Bernard & Clover, SAE 950311):

* σ_x·κ̇ = (ω·R_e − V_x) − |V_x|·κ
* σ_y·ṡ_y = V_y − |V_x|·s_y

V_x and V_y are the contact-point velocities along the wheel heading and lateral axes, and σ_x
and σ_y are the relaxation lengths. In steady state κ → (ωR − V_x)/|V_x| and s_y → V_y/|V_x| = −tan α.
The implicit update is:

κ⁺ = (κ + Δt/σ_x·(ωR_e − V_x)) / (1 + Δt·|V_x|/σ_x)

At standstill the states integrate slip **displacement**, so the tyre behaves like a spring and a
kart can stand on a slope. A low-speed viscous term (below 1.5 m/s) damps the
spring–inertia oscillation.

### 3.3 Combined-slip brush model (parabolic pressure distribution)

The theoretical slips are

* σ_x = κ/(1+κ), σ_y = s_y/(1+κ), with 1+κ clamped to ≥ 0.05.

Load- and condition-dependent parameters:

* μ = μ₀ · f_surface · f_temp(T) · f_wear(w) · [1 − k_μ·(F_z/F_z0 − 1)] · f_slide(v_s)
* f_slide = 1 − d_s·(1 − e^(−v_s/v_ref)), where v_s is the contact sliding speed. This gives the
  observed falloff of friction past the peak.
* C_x = c_x·F_z0·(F_z/F_z0)^p, C_y = c_y·F_z0·(F_z/F_z0)^p (slip stiffnesses in N per unit slip)

Normalised slip (anisotropic brush):

* θ_x = C_x/(3μF_z), θ_y = C_y/(3μF_z)
* λ_x = θ_x·σ_x, λ_y = θ_y·σ_y, λ = √(λ_x² + λ_y²)

Resultant force magnitude:

* λ < 1: F = μF_z·(1 − (1 − λ)³)  (adhesion + partial sliding)
* λ ≥ 1: F = μF_z  (full sliding)

Components:

* F_x = F·λ_x/λ, F_y = −F·λ_y/λ

The small-slip check: F ≈ C·σ, i.e. the linear cornering/longitudinal stiffness is recovered.

Aligning moment (brush, parabolic):

* M_z = μF_z·a·λ_y·(1 − λ)³ for λ < 1, and 0 when fully sliding.

As slip → 0, the pneumatic trail → a/3. It collapses as the tyre saturates, which is the
physical cue for understeer in the force feedback.

**Rolling resistance:** F_rr = −C_rr·F_z·tanh(V_x/0.5) along the heading.

Anisotropic force direction is a simplification. The exact anisotropic brush solution has
slightly different force directions in the transition region. Camber thrust is not modelled
yet (M2).

### 3.4 Temperature (single lumped node per tyre)

C_th·Ṫ = η_h·P_slide + P_roll − h(V)·(T − T_air) − h_track·(T − T_track)

* P_slide = |F_x·V_sx| + |F_y·V_sy| (frictional power at the contact)
* P_roll = C_rr·F_z·|V_x| (hysteresis)
* h(V) = h₀ + h₁·|V| (forced convection)
* Grip window: f_temp = 1 − (1 − f_min)·min(1, ((T − T_opt)/ΔT)²)

### 3.5 Wear

ẇ = k_w·P_slide, with f_wear = 1 − w·(1 − f_wear,min), w ∈ [0, 1].

## 4. Steering

### 4.1 Column to road wheels

δ_ref = θ_column / ratio. The ideal Ackermann angles for the turn radius implied by δ_ref
(wheelbase L, kingpin spacing w) are δ_in = atan(L/(L/tan δ_ref − w/2)) and
δ_out = atan(L/(L/tan δ_ref + w/2)). The applied angles blend parallel and Ackermann steering:
δ = δ_ref + A·(δ_ackermann − δ_ref), where A ∈ [0, 1] is a setup parameter. Static toe is added.

### 4.2 Kingpin kinematics (caster, KPI, spindle offset)

Each front wheel rotates by δ about the kingpin axis **k**, which is tilted back by the caster
angle ν and inward by the kingpin inclination σ, through the kingpin pivot **P**. The wheel centre
and axle are rotated with Rodrigues' formula:

c' = P + R(k, δ)·(c − P), a' = R(k, δ)·a

The new contact point follows from §3.1. Because **k** is not vertical, steering moves the
lowest point of the wheel vertically relative to the chassis. With positive caster, the outside
wheel is pushed down and the chassis is lifted on that side. This geometric weight-jacking is
the kart's primary mechanism for unloading the inside rear wheel. It **emerges from the
geometry** and is not added as a load-transfer term.

### 4.3 Kingpin and column torque

For each front wheel, the moment about its kingpin axis is

M_k = ((p − P) × F_tyre)·k + M_z·(n·k)

where F_tyre = F_x·f + F_y·l + F_z·n is the full contact force. This includes the
mechanical-trail, pneumatic-trail and weight-jacking (vertical load × caster/KPI) contributions,
all from one expression. Column torque = (M_k,L·∂δ_L/∂θ + M_k,R·∂δ_R/∂θ), evaluated numerically.

## 5. Powertrain

### 5.1 Engine

* T_engine = τ·T_WOT(ω_e) − (1 − τ)·T_drag(ω_e)
* τ is the effective throttle: first-order lag on the pedal (time constant τ_t), with an idle
  controller that opens the throttle as the rpm falls below idle.
* T_drag = d₀ + d₁·ω_e (closed-throttle pumping + friction: ESTIMATE)
* Rev limiter: ignition cut (τ → 0) above the limit, with hysteresis.
* I_e·ω̇_e = T_engine − T_clutch

### 5.2 Clutch

Torque capacity T_cap:
* Centrifugal: shoe normal force = m·r·ω² − F_spring, so
  T_cap = T_max·clamp((ω_e² − ω_start²)/(ω_full² − ω_start²), 0, 1). Capacity is independent of the pedal.
  During a standing start the engine settles where T_cap(ω_e) = T_engine(ω_e), about 4,100 rpm with the
  ASSUMPTION parameters. That is why the launch is sensitive to the (estimated) low-rpm torque.
* Manual (KZ): T_cap = T_max·(1 − pedal).
* Direct drive: rigid coupling (always locked).

**Stick/slip:**
* *Slipping:* T_c = T_cap·sign(ω_e − ω_in), and both sides are integrated separately. If the
  slip speed changes sign during the substep, the sides are locked at the momentum-conserving
  speed.
* *Locked:* the engine and drivetrain are integrated as one inertia, I_e·G² + I_axle. The
  required clutch torque is T_c = T_e − I_e·G·α_axle, and the clutch unlocks if |T_c| > T_cap.

### 5.3 Gearing and chain

G = (gear ratio × primary ratio) × (axle sprocket / engine sprocket). Single-speed karts use one
gear with ratio 1. Chain and drivetrain efficiency η: T_axle = η·G·T_c when the engine drives,
and G·T_c/η when the axle back-drives (engine braking).

### 5.4 Rear axle

The rear axle is **solid**: both rear wheels share one angular velocity ω_r. There is no
differential, so the inside rear tyre has to scrub in a corner unless it is unloaded. This is
the physical reason inside-rear lift matters in a kart.

I_axle·ω̇_r = η·G·T_c − R_e,L·F_x,RL − R_e,R·F_x,RR − T_brake

## 6. Brakes

* T_brake = p·T_max·f_fade(T_disc), where p is the pedal position in [0, 1]
* Coulomb treatment: if |ω|·I ≤ T_brake·Δt then ω = 0 (locked); otherwise ω −= sign(ω)·T_brake·Δt/I
* Disc thermal model: C_d·Ṫ = |T_brake·ω| − h_d(V)·(T − T_air)
* Fade: f_fade = 1 − k_fade·max(0, T − T_fade)

## 7. Track surfaces

| Surface | Grip factor | Rolling resistance multiplier | Notes |
|---|---|---|---|
| Asphalt (green) | 1.00 | 1.0 | baseline |
| Rubbered line | 1.08 | 1.0 | ASSUMPTION; dynamic evolution in M5 |
| Dirty asphalt | 0.85 | 1.1 | ASSUMPTION |
| Wet asphalt | 0.60 | 1.0 | ASSUMPTION; wet tyres in M5 |
| Kerb | 0.90 | 1.0 | ASSUMPTION; geometric profile gives the impacts |
| Grass | 0.45 | 4.0 | ASSUMPTION |

Bumps are represented geometrically (heightfield or kerb profile), never as random forces.

## 8. Force feedback

The device command is the simulated **column torque** (§4.3), scaled by a user gain to Nm at the
wheel and normalised by the device's peak torque. It is then processed:

* an optional low-pass filter (default 120 Hz cut-off, i.e. effectively unfiltered at 240 Hz)
* a slew-rate limit
* soft clipping with a clipping meter

Road texture and kerb impacts reach the wheel because F_z and the contact point vary with real
geometry. No synthetic vibration is added.

## 9. Validation tests (automated, `Tests/`)

* Static load distribution equals the configured weight distribution (±1 %).
* Brush model: small-slip slope equals C, saturation equals μF_z, combined slip stays inside the
  friction circle.
* Transient slip converges to the steady-state slip.
* Clutch: no drive below engagement rpm, lock-up above it, momentum conserved at lock.
* Braking: the wheel locks and stays locked without reversing.
* Lateral load transfer in steady cornering matches m·a_y·h/t within tolerance (static check).
* Inside rear lift occurs in a steady-state corner above a threshold lateral acceleration
  (reported, not forced).
* Energy check: undriven, unbraked coast-down decelerates monotonically.
* Timestep sensitivity: 960 Hz versus 1920 Hz trajectories agree within tolerance.

## 10. Known limitations (honest list)

* Tyre parameters are **engineering estimates**. No kart tyre test data has been obtained yet
  (see `RotaxMaxBaseline.md` §5).
* There is no camber thrust or turn-slip, and the tyre has a single thermal node.
* The chassis has a single torsional mode. Bending and the seat-stay/axle compliance are not modelled.
* Engine torque between the two verified datasheet points is an estimate shaped curve.
* The steering column is position-controlled (from input), not torque-driven by the driver model.
