# Drone Delivery Path Optimization: Minimizing Energy/Time Subject to No-Fly-Zone and Battery Constraints

**Course:** Optimization Techniques — "Optimization in the Wild" mini-project

## Note on scope

This version replaces the earlier RL/gym-pybullet-drones draft. This project is framed as a **direct constrained trajectory optimization problem** (decision variables = a sequence of waypoints), not a reinforcement-learning policy-training problem. This is the framing that matches the marks distribution sheet (Problem Formulation → Convexity/KKT → GD vs Heavy-Ball convergence referencing condition number κ → Reflection → Viva) and matches the two objectives you want to focus on: **minimize energy or time for a delivery path, subject to no-fly-zone and battery constraints.**

---

# 1. Problem Formulation (20 marks)

## 1.1 Decision variables

The drone's path is represented as a discretized sequence of `N+1` waypoints in 2D (extendable to 3D):

```
x = (P_0, P_1, P_2, ..., P_N),    P_i = (x_i, y_i) ∈ R^2
```

`P_0` = fixed start location (warehouse), `P_N` = fixed goal location (delivery address). The `N-1` interior waypoints `P_1, ..., P_{N-1}` are the actual decision variables — so the optimization vector is:

```
x = (P_1, P_2, ..., P_{N-1}) ∈ R^{2(N-1)}
```

## 1.2 Objective function f(x)

Two real-world objectives, combined so the project can study both individually and jointly:

**Time-minimizing path** (segment length over assumed cruising speed `v`):

```
T(x) = Σ_{i=0}^{N-1}  ||P_{i+1} - P_i|| / v
```

**Energy-minimizing path** (simplified model first — flagged and extended in Section 7):

```
E(x) = Σ_{i=0}^{N-1}  c · ||P_{i+1} - P_i||
```

where `c` is an energy-per-meter constant. Combined weighted objective (lets you dial between "cheapest" and "fastest" delivery):

```
f(x) = α · E(x) + (1 - α) · T(x),      α ∈ [0, 1]
```

`α = 1` → pure energy minimization, `α = 0` → pure time minimization. This is the realistic, tunable version an actual delivery service would use (battery cost vs. delivery SLA).

## 1.3 Constraints, in standard form

```
minimize    f(x)
subject to  g_k(P_i) = d_safety - dist(P_i, F_k) ≤ 0     for every waypoint P_i, every no-fly-zone F_k
            h(x) = E(x) - E_battery ≤ 0                   (battery budget)
            P_0, P_N fixed (boundary conditions)
            ||P_{i+1} - P_i|| ≤ s_max                     (max per-segment speed/step, optional)
```

- `F_k` = no-fly-zone `k`, modeled as a circle (center `c_k`, radius `r_k`) for tractability.
- `d_safety` = minimum clearance distance from any NFZ boundary (realistic addition — see Section 7.5).
- `E_battery` = usable battery energy budget for the trip.

## 1.4 Scoping

- Domain: urban last-mile delivery, radius ~2–10 km, 3–6 no-fly-zones (no-fly zones around airports, stadiums, restricted airspace, or ephemeral ones like accident sites), `N` in the 10–100 range.
- Units: `E` in Wh (or a proxy "energy units"), `T` in minutes, distances in meters, `d_safety` in meters (e.g. 20 m per the practical margin note below).
- This is realistically scoped: not trivial (nonlinear, multiple non-convex constraints, real trade-off between two objectives) and not unsolvably complex (2D waypoint formulation, closed-form gradients, solvable with GD/Heavy-Ball in a homework timeframe).

---

# 2. Convexity / KKT Analysis (25 marks)

## 2.1 Hessian of the objective

`T(x)` and `E(x)` are both sums of Euclidean-norm terms — these are convex (norms are convex, sums of convex functions are convex) but **not everywhere twice differentiable** (the norm has a kink at `P_{i+1} = P_i`, which won't occur in practice for a real path). Away from that degenerate point, each segment-length term contributes a Hessian block:

```
∇² ||P_{i+1} - P_i||  =  (1 / ||P_{i+1} - P_i||) · (I - u_i u_i^T)
```

where `u_i` is the unit vector along the segment. This is **positive semi-definite** (it projects onto the direction orthogonal to the segment), so `f(x)` itself is convex on its smooth domain. This should be verified numerically in the report by evaluating the Hessian at a candidate path and checking eigenvalues ≥ 0.

## 2.2 Why the overall problem is non-convex

The objective is convex, but the **feasible set is not**:

```
g_k(P_i) = d_safety - dist(P_i, F_k) ≤ 0   ⟺   dist(P_i, F_k) ≥ d_safety
```

This says "stay *outside* a disk" — the complement of a convex set, which is non-convex. With multiple no-fly-zones this carves the feasible region into disconnected "corridors." So:

```
convex objective + non-convex feasible region  ⟹  non-convex problem overall
```

This is the correct, defensible convexity conclusion for the report — don't force a false claim that the whole problem is convex.

## 2.3 Lagrangian and KKT conditions

```
L(x, λ, μ) = f(x) + Σ_k Σ_i λ_{k,i} · g_k(P_i) + μ · h(x)
```

KKT stationarity, primal feasibility, dual feasibility, and complementary slackness:

```
∇_x L = 0
g_k(P_i) ≤ 0,  h(x) ≤ 0
λ_{k,i} ≥ 0,  μ ≥ 0
λ_{k,i} · g_k(P_i) = 0,   μ · h(x) = 0
```

**Active vs. inactive constraints** — identify these from your actual solved path:
- A no-fly-zone constraint `g_k(P_i)` is **active** at any waypoint that sits exactly on the `d_safety` boundary (the path "hugs" the obstacle) — `λ_{k,i} > 0`.
- It's **inactive** for waypoints/zones the path doesn't come near — `λ_{k,i} = 0`.
- The battery constraint `h(x)` is active only if the optimal path would want to use more energy than `E_battery` allows (i.e., the unconstrained-by-battery optimum already respects the budget → inactive; a tight budget forces a shorter/slower route → active).

Because the feasible region is non-convex, KKT points here are only guarantees of **local** stationarity, not global optimality — this must be stated explicitly (this is exactly the "local optimum ≠ guaranteed global optimum" limitation, formalized).

---

# 3. Implementation & Convergence Comparison: GD vs Heavy-Ball (25 marks)

## 3.1 Making it solvable with unconstrained methods

Convert the constrained problem into an unconstrained one GD/Heavy-Ball can handle, via a **quadratic penalty** (simplest, recommended first) or an **augmented Lagrangian** (better, optional stretch):

```
F(x) = f(x) + ρ · Σ_k Σ_i max(0, d_safety - dist(P_i, F_k))²  + ρ · max(0, E(x) - E_battery)²
```

Increase `ρ` across outer iterations (or use a fixed large `ρ`) so violating the NFZ/battery constraints gets increasingly costly.

## 3.2 Algorithms

```
Gradient Descent:      x_{t+1} = x_t - η ∇F(x_t)

Heavy-Ball:             x_{t+1} = x_t - η ∇F(x_t) + β (x_t - x_{t-1})
```

## 3.3 Condition number κ

Estimate `κ = λ_max / λ_min` of the (numerically computed) Hessian of `F` along the solution path. Since the penalty terms add curvature that's highly directional near the constraint boundary, `κ` will typically be **large/ill-conditioned** near active constraints and closer to well-conditioned in open space.

- **Predicted result:** Heavy-Ball should converge noticeably faster than plain GD when `κ` is large (its momentum term dampens oscillation across the narrow/steep directions that make vanilla GD zig-zag), with the gap shrinking as `κ → 1`.
- Report must **explicitly connect the convergence plots to κ** — e.g., "GD required ~3.5× more iterations to reach the same tolerance in the high-`ρ`, high-`κ` regime near the NFZ boundary, consistent with GD's O(κ) vs. Heavy-Ball's O(√κ) iteration complexity for ill-conditioned convex objectives."

## 3.4 Experiment plan

- Run both optimizers from the same initialization (e.g., straight-line path clipped away from obstacles) and same `η`, sweep `β` for Heavy-Ball.
- Multiple random seeds / multiple start-goal-NFZ configurations, report mean ± variance.
- **Discretization trade-off study** (from the limitations notes): repeat the whole experiment at `N = 10, 25, 50, 100` and plot:
  - final objective value vs. `N` (accuracy of the trajectory)
  - wall-clock time / iterations-to-converge vs. `N` (cost)
  - This directly demonstrates the "small N = fast but crude, large N = accurate but expensive" trade-off and gives you a real figure for the report instead of just asserting it.
- Plots needed: convergence curves (objective vs. iteration) for GD vs Heavy-Ball, at a few `N`/`ρ` settings, clearly labeled with axes, legend, and the `κ` value annotated.

---

# 4. Reflection & Insight (15 marks)

Talking points to build this section around (write your own, in your own words, after running the experiments):

- How did the measured convergence gap between GD and Heavy-Ball track the estimated `κ`? Where did the theory (O(κ) vs O(√κ)) match or diverge from what you observed, and why (e.g., non-smoothness from the penalty term, or the non-convex feasible region causing different local optima to be found by the two methods)?
- Did GD and Heavy-Ball ever converge to genuinely **different local optima** (different NFZ-avoidance routes)? That's a direct, concrete consequence of non-convexity worth calling out.
- Concrete extension to propose: pick one from Section 7 (e.g., moving to a receding-horizon/MPC formulation for dynamic no-fly-zones, or replacing the penalty method with an augmented Lagrangian for exact constraint satisfaction) and briefly sketch how it would change the formulation.

---

# 5. Viva Prep (15 marks)

Be ready to explain, unprompted and fluently:

- Why the decision variables are the interior waypoints, not the whole path (boundary points are fixed).
- Why `f(x)` is convex but the *problem* isn't (Section 2.2) — this is the single most important conceptual point graders will probe.
- Walk through deriving the Hessian of one segment-length term live if asked.
- Why a penalty method was used to make this solvable by unconstrained GD/Heavy-Ball, and what its limitation is (constraints are only approximately satisfied unless `ρ → ∞`; an augmented Lagrangian fixes this).
- What `κ` means physically here and why it explains the GD vs Heavy-Ball gap.
- If AI assistance was used (e.g., for boilerplate plotting code or LaTeX-ing the KKT conditions), be ready to explain in your own words what each piece does — don't let "the AI wrote it" be your answer to any question.

---

# 6. Limitations

These are the honest, specific limitations of the core model above — call these out proactively in the report rather than waiting to be asked.

1. **Simplified energy model.** `E(x) = c · Σ ||P_{i+1} - P_i||` ignores wind, payload, acceleration, altitude, temperature, battery aging, motor efficiency, propeller efficiency, and flight speed — real consumption depends on all of these, not just distance.
2. **Weather is not modeled.** Wind and other conditions materially change both energy use and safe/legal flight paths.
3. **No-fly-zones are treated as static**, but real restrictions can appear or change mid-flight (e.g., a temporary restriction). A static formulation can't react to that.
4. **The problem is non-convex** (Section 2.2), so gradient-based methods (GD, Heavy-Ball) can converge to a local optimum that is not the global optimum — this must not be overclaimed in the report.
5. **Discretization.** The path is a discrete sequence of `N` waypoints, not a continuous trajectory; `N` trades off accuracy against computational cost (Section 3.4).
6. **No safety margin by default.** `dist(P_i, F_k) ≥ 0` (touching the boundary) isn't good enough in practice — GPS error and control error mean the drone needs a real buffer, hence `d_safety` (e.g., 20 m) rather than 0.

---

# 7. Real-World Extensions — Overcoming the Limitations

This is where the project goes from "a homework optimization problem" to something closer to an actual applicable delivery-routing tool. Treat these as the "going deeper" layer on top of the core (Sections 1–5), attempted after the core version works and is validated.

## 7.1 A realistic energy model

Replace the linear-in-distance `E(x)` with a segment-wise model that accounts for the dominant real factors:

```
E_i ≈ (c_0 + c_1 · payload + c_2 · |v_i|² + c_3 · max(0, Δaltitude_i)) · ||P_{i+1} - P_i|| / v_i
```

- `c_2 · |v_i|²` captures that drag/power roughly scales with speed squared — this also makes speed a decision variable, not just a constant, which is a meaningful upgrade over the base model.
- `c_3 · Δaltitude` captures climbing cost (descending can even be ~free or regenerative, worth a footnote).
- Wind can be added as a vector `w`, with effective airspeed `||v_i - w||` replacing `|v_i|` in the drag term — this is the cleanest way to fold weather (Limitation 2) into the same model without a separate subsystem.
- Note in the report that even this is still an approximation of manufacturer power curves, but it's a large step up from the pure-distance model and is defensible as "realistic for a course project."

## 7.2 Safety margin

Already folded into the core formulation as `d_safety` (Section 1.3) rather than treated as an afterthought — e.g. `d_safety = 20 m`. Worth an explicit sentence in the report: mathematically `P_i ∉ F_k` is not sufficient because GPS/control error can violate an exact boundary; `dist(P_i, F_k) ≥ d_safety` maintains a real buffer.

## 7.3 Battery constraint as a real operating limit

Beyond the simple `E(x) ≤ E_battery` cap, a more realistic version reserves a safety margin (e.g., require `E(x) ≤ 0.8 · E_battery` to keep a reserve for wind gusts or a detour), and can be tested as a **sensitivity analysis**: sweep the allowed budget and plot how the optimal route/time changes — this is a cheap, high-value addition that directly demonstrates "battery-constrained" delivery planning.

## 7.4 Dynamic no-fly-zones → receding-horizon / MPC extension

If a no-fly-zone can appear mid-flight, a single offline optimization isn't enough. The natural extension:

```
At each control step:
    1. Re-observe current NFZ map
    2. Re-solve the trajectory optimization from the current position to the goal
       over a shorter look-ahead horizon
    3. Execute the first segment
    4. Repeat
```

This is a receding-horizon / Model Predictive Control (MPC) pattern. It's worth including as a described (not necessarily fully implemented) stretch extension — a short simulated demo (re-solving once when a new NFZ is injected mid-path) is a strong, achievable stretch goal that shows depth without requiring a full real-time control stack.

## 7.5 Handling non-convexity honestly

- Run GD/Heavy-Ball from **multiple different initializations** (e.g., straight-line, and 2–3 routes that go around obstacles from different sides) and report that different starts can converge to different local optima with different objective values — this is the honest, demonstrable way to handle non-convexity in a project like this, rather than claiming a single run found "the" optimum.
- Optionally mention convex relaxations or mixed-integer formulations (e.g., pre-assigning which side of each obstacle the path passes on, which turns the non-convex avoidance constraint into a convex one *per assignment*) as a known technique, without necessarily implementing it — this shows awareness of the broader literature for the viva.

## 7.6 Discretization trade-off, formalized

Already specified as an experiment in Section 3.4 (`N = 10, 25, 50, 100`) — this doubles as both a limitation and a result: the report can present it as "here is the limitation, and here is the quantified trade-off we measured," which is a stronger structure than describing it only as a caveat.

---

# 8. Recommended Project Structure

```
drone_delivery_optimization/
│
├── model/
│   ├── objective.py        # E(x), T(x), f(x), and the extended energy model
│   ├── constraints.py      # NFZ distance constraints, battery constraint
│   └── penalty.py          # penalized F(x), augmented Lagrangian (optional)
│
├── optimizers/
│   ├── gradient_descent.py
│   └── heavy_ball.py
│
├── analysis/
│   ├── convexity_check.py  # numeric Hessian + eigenvalue check
│   └── kkt_analysis.py     # active/inactive constraint identification
│
├── experiments/
│   ├── run_core_experiment.py     # GD vs Heavy-Ball, multi-seed
│   ├── run_discretization_study.py # N = 10, 25, 50, 100 sweep
│   └── run_mpc_demo.py            # optional dynamic-NFZ stretch demo
│
├── results/
│   ├── convergence_plots/
│   ├── discretization_tradeoff/
│   └── kkt_summary.csv
│
└── report/
    └── README.md
```

---

# 9. Suggested Build Order

1. Implement `E(x)`, `T(x)`, `f(x)` and the basic NFZ + battery constraints (Section 1).
2. Numerically verify the convexity claim (Hessian eigenvalues) and derive KKT conditions on paper (Section 2).
3. Implement the penalized objective, then GD and Heavy-Ball (Section 3.1–3.2).
4. Run the core GD vs Heavy-Ball comparison, estimate κ, generate convergence plots (Section 3.3–3.4).
5. Run the discretization trade-off sweep (`N = 10, 25, 50, 100`).
6. Add the realistic energy model (Section 7.1) and re-run to compare against the simplified version — this comparison is itself a great figure for the report.
7. Add the safety margin and battery-sensitivity sweep (Sections 7.2–7.3).
8. If time allows: multi-initialization non-convexity demo (7.5) and the MPC/dynamic-NFZ demo (7.4) as stretch goals.
9. Write Reflection (Section 4) and prep the Viva talking points (Section 5) last, once real results exist to reflect on.
