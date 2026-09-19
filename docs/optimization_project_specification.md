# 2D Drone Delivery Optimization Project

## Optimization-First Project Specification

## 1. Project Direction

This project will be developed as one continuous system with two
possible layers:

1.  **Primary layer --- Mathematical Optimization**
2.  **Future extension --- Reinforcement Learning (RL)**

For the current phase, only the **optimization layer** will be
implemented. The RL part is intentionally postponed until the
optimization project is complete and stable.

The project will use a **2D drone-delivery environment** rather than a
3D environment. The goal is to formulate a constrained
trajectory-optimization problem, analyze its mathematical properties,
implement the optimization techniques required by the course, verify
feasibility, visualize the trajectory, and compare the convergence
behavior of Gradient Descent and Heavy-Ball Momentum.

------------------------------------------------------------------------

# 2. Main Project Idea

## Proposed Title

**2D Autonomous Drone Delivery: Constrained Trajectory Optimization
Using Gradient Descent and Heavy-Ball Momentum**

## Main Research Question

> How effectively can Gradient Descent and Heavy-Ball Momentum optimize
> a 2D drone-delivery trajectory under distance, energy, time, battery,
> and no-fly-zone constraints?

## Core Objective

Given:

-   A fixed starting point
-   A fixed destination
-   A set of intermediate 2D waypoints
-   A 2D environment containing obstacles/no-fly zones
-   Energy, distance, time, and battery considerations

we will optimize the waypoint coordinates so that the drone obtains a
safe and efficient route.

The project must first solve and explain the mathematical optimization
problem. Only after that may RL be considered as a future extension.

------------------------------------------------------------------------

# 3. Important Course Requirements

The project must satisfy the instructor's Optimization Techniques
requirements.

The mandatory structure is:

1.  Problem formulation
2.  Decision variables
3.  Objective function
4.  Constraints
5.  Gradient derivation
6.  Hessian calculation
7.  Positive semidefinite/positive definite analysis
8.  Convexity discussion
9.  Lagrangian formulation
10. KKT conditions
11. Active and inactive constraints
12. Gradient Descent implementation
13. Heavy-Ball Momentum implementation
14. Convergence comparison
15. Condition number analysis
16. Trajectory visualization
17. Feasibility verification
18. Results and reflection
19. Viva preparation

These components must not be removed or replaced by RL.

------------------------------------------------------------------------

# 4. Scope of the Current Version

## Included

-   2D drone-delivery environment
-   Fixed start and destination
-   Optimizable intermediate waypoints
-   Distance-based objective
-   Energy-related objective terms
-   Time-related objective terms
-   Battery constraint
-   No-fly-zone constraints
-   Feasibility checking
-   Gradient Descent from scratch
-   Heavy-Ball Momentum from scratch
-   Hessian analysis
-   Convexity analysis
-   Lagrangian and KKT analysis
-   Active/inactive constraint analysis
-   Condition number calculation
-   Convergence plots
-   Trajectory plots
-   Comparison table
-   Reflection and limitations

## Not included in the first version

-   Reinforcement Learning
-   PPO
-   SAC
-   AirSim
-   Camera-based navigation
-   3D simulation
-   Dynamic obstacles
-   Deep neural networks
-   Policy learning
-   Real-world flight control

These may be considered later only if the optimization layer is
complete.

------------------------------------------------------------------------

# 5. System Architecture

``` text
2D Drone Delivery Environment
            |
            v
      Problem Definition
            |
            v
      Decision Variables
            |
            v
      Objective Function
            |
            v
         Constraints
            |
            v
     Mathematical Analysis
     - Gradient
     - Hessian
     - PSD/PD
     - Convexity
     - Lagrangian
     - KKT
            |
            v
   Optimization Algorithms
     - Gradient Descent
     - Heavy-Ball
            |
            v
     Feasibility Checking
            |
            v
    Trajectory Visualization
            |
            v
   Convergence + Condition Number
            |
            v
       Final Comparison
            |
            v
       Results + Reflection
```

------------------------------------------------------------------------

# 6. 2D Environment Definition

The environment will be represented in a 2D Cartesian plane.

Example:

-   Start: `S = (0, 0)`
-   Destination: `G = (10, 10)`
-   Intermediate waypoints: variable
-   No-fly zones: circular or rectangular obstacles

A simple initial environment may use:

``` text
Start:       (0, 0)
Goal:        (10, 10)

No-fly zone 1:
Center:      (4, 4)
Radius:      1.5

No-fly zone 2:
Center:      (7, 7)
Radius:      1.2
```

These values are illustrative and may be adjusted after checking whether
the route is feasible.

The first environment should be simple enough to debug and visualize.

------------------------------------------------------------------------

# 7. Decision Variables

The start and destination remain fixed.

The intermediate waypoints are optimized.

For `N` intermediate waypoints:

\[ P_i=(x_i,y_i), `\qquad `{=tex}i=1,`\ldots`{=tex},N \]

The complete decision vector is:

\[ z = \[x_1,y_1,x_2,y_2,`\ldots`{=tex},x_N,y_N\]\^T \]

The complete trajectory is:

\[ P_0=S,`\quad`{=tex} P_1,`\ldots`{=tex},P_N,`\quad`{=tex} P\_{N+1}=G
\]

where:

-   (P_0) is the fixed start
-   (P\_{N+1}) is the fixed destination
-   (P_1,`\ldots`{=tex},P_N) are optimized

Example with three intermediate waypoints:

\[ z=\[x_1,y_1,x_2,y_2,x_3,y_3\]\^T \]

The algorithm must optimize only the intermediate waypoint coordinates.

------------------------------------------------------------------------

# 8. Segment Distance

For two consecutive waypoints:

\[ P_i=(x_i,y_i),`\qquad `{=tex}P\_{i+1}=(x\_{i+1},y\_{i+1}) \]

the Euclidean distance is:

\[ d_i= `\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}`{=tex} \]

The total trajectory distance is:

\[ D(z)=`\sum`{=tex}\_{i=0}\^{N}d_i \]

The distance function is the basic component of the objective.

------------------------------------------------------------------------

# 9. Objective Function

The objective should represent the cost of flying the drone along the
trajectory.

A practical first objective is a weighted combination:

\[ F(z)= w_DD(z)+ w_EE(z)+ w_TT(z)+ w_S S(z) \]

where:

-   (D(z)): total distance
-   (E(z)): estimated energy
-   (T(z)): estimated flight time
-   (S(z)): optional smoothness penalty
-   (w_D,w_E,w_T,w_S): nonnegative weights

## 9.1 Distance Cost

\[ D(z)=`\sum`{=tex}\_{i=0}\^{N}d_i \]

## 9.2 Simplified Energy Cost

For the first version, energy may be modeled as proportional to
distance:

\[ E(z)=c_E D(z) \]

where (c_E\>0) is an energy-per-distance coefficient.

A more advanced version may include turning or altitude-related terms,
but the first implementation should remain understandable.

## 9.3 Time Cost

If the drone speed is assumed constant:

\[ T(z)=`\frac{D(z)}{v}`{=tex} \]

where (v\>0) is the assumed speed.

## 9.4 Optional Smoothness Penalty

To discourage sharp turns:

\[ S(z)= `\sum`{=tex}*{i=1}\^{N} `\left`{=tex}\| P*{i+1}-2P_i+P\_{i-1}
`\right`{=tex}\|\^2 \]

This term may be included only after the basic distance objective is
working.

## 9.5 Simplified Combined Objective

Because energy and time may both be proportional to distance, the
initial objective can be simplified to:

\[ F(z)=`\alpha `{=tex}D(z)+`\lambda `{=tex}S(z) \]

where:

-   (`\alpha`{=tex}\>0) controls travel cost
-   (`\lambda`{=tex}`\geq0`{=tex}) controls smoothness

This simplified objective is recommended for the first working
implementation because it is easier to derive, differentiate, debug, and
explain in the viva.

------------------------------------------------------------------------

# 10. Constraints

The optimization problem must include realistic constraints.

## 10.1 Battery Constraint

Let the maximum available battery energy be (B\_{`\max`{=tex}}).

The route must satisfy:

\[ E(z)`\leq `{=tex}B\_{`\max`{=tex}} \]

If energy is proportional to distance:

\[ c_E D(z)`\leq `{=tex}B\_{`\max`{=tex}} \]

Equivalently:

\[ g_B(z)=c_E D(z)-B\_{`\max`{=tex}}`\leq0`{=tex} \]

## 10.2 Maximum Time Constraint

If the maximum allowed delivery time is (T\_{`\max`{=tex}}):

\[ T(z)`\leq `{=tex}T\_{`\max`{=tex}} \]

Equivalently:

\[ g_T(z)=T(z)-T\_{`\max`{=tex}}`\leq0`{=tex} \]

## 10.3 No-Fly-Zone Constraint

For a circular no-fly zone with center:

\[ C=(c_x,c_y) \]

and radius (r), every waypoint must remain outside the zone:

\[ (x_i-c_x)^2+(y_i-c_y)^2`\geq `{=tex}r\^2 \]

In standard inequality form:

\[ g\_{NFZ,i}(z)= r^2-(x_i-c_x)^2-(y_i-c_y)\^2 `\leq0`{=tex} \]

A safety margin (m) may be added:

\[ g\_{NFZ,i}(z)= (r+m)^2-(x_i-c_x)^2-(y_i-c_y)\^2 `\leq0`{=tex} \]

Important: checking only the waypoints is not always enough. A line
segment between two safe waypoints may still pass through an obstacle.
Therefore, the final feasibility checker should sample each trajectory
segment and verify that the complete path remains outside every no-fly
zone.

## 10.4 Workspace Bounds

The drone should remain inside the allowed 2D map:

\[ x\_{`\min`{=tex}}`\leq `{=tex}x_i`\leq `{=tex}x\_{`\max`{=tex}} \]

\[ y\_{`\min`{=tex}}`\leq `{=tex}y_i`\leq `{=tex}y\_{`\max`{=tex}} \]

These can be represented as inequality constraints.

## 10.5 Optional Maximum Segment Length

To avoid unrealistic jumps between waypoints:

\[ d_i`\leq `{=tex}d\_{`\max`{=tex}} \]

This constraint may be added after the basic version works.

------------------------------------------------------------------------

# 11. Complete Mathematical Formulation

The optimization problem is:

\[ `\min`{=tex}\_z F(z) \]

subject to:

\[ g_j(z)`\leq0`{=tex},`\qquad `{=tex}j=1,`\ldots`{=tex},m \]

where:

\[ F(z)=w_DD(z)+w_EE(z)+w_TT(z)+w_SS(z) \]

and the constraints include:

-   Battery
-   Time
-   No-fly zones
-   Workspace boundaries
-   Optional segment-length limits

The complete formulation should be written clearly in the report before
any code is presented.

------------------------------------------------------------------------

# 12. Feasibility Before Optimization

Before running Gradient Descent or Heavy-Ball, the system must check
whether the problem is feasible.

## Required feasibility checks

1.  Start is inside the workspace.
2.  Goal is inside the workspace.
3.  Start is not inside a no-fly zone.
4.  Goal is not inside a no-fly zone.
5.  An initial trajectory exists.
6.  The initial trajectory does not violate hard safety constraints, or
    a repair procedure is applied.
7.  Battery capacity is sufficient for at least one candidate route.
8.  Maximum time is sufficient for at least one candidate route.
9.  Waypoints remain inside the workspace.
10. Every segment is checked, not only waypoint locations.

If the initial path is infeasible, the code should not silently treat it
as feasible. It should report the violated constraints.

------------------------------------------------------------------------

# 13. Constraint Handling Strategy

Gradient Descent and Heavy-Ball are easiest to implement on an
unconstrained or penalized objective.

The first implementation may use a penalty function:

\[ F\_{`\text{penalty}`{=tex}}(z) = F(z) +
`\rho`{=tex}`\sum`{=tex}\_{j=1}\^{m} `\max`{=tex}(0,g_j(z))\^2 \]

where:

-   (`\rho`{=tex}\>0) is the penalty coefficient
-   (g_j(z)) is a constraint function

For no-fly-zone constraints:

\[ `\max`{=tex}(0,g\_{NFZ,i}(z))\^2 \]

adds a cost when a waypoint enters an obstacle.

However, penalty optimization does not automatically guarantee exact
feasibility. Therefore, after optimization, the route must be checked
again using the original hard constraints.

## Recommended approach

1.  Build the objective.
2.  Build constraint functions.
3.  Build penalty objective.
4.  Run GD and Heavy-Ball.
5.  Check original constraints.
6.  Report all violations.
7.  If needed, increase penalty strength or repair the route.
8.  Compare only feasible solutions fairly.

------------------------------------------------------------------------

# 14. Gradient Derivation

The gradient must be derived mathematically and also implemented in
code.

For a distance segment:

\[ d_i= `\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}`{=tex} \]

the partial derivatives are:

\[ `\frac{\partial d_i}{\partial x_i}`{=tex} =
`\frac{x_i-x_{i+1}}{d_i}`{=tex} \]

\[ `\frac{\partial d_i}{\partial y_i}`{=tex} =
`\frac{y_i-y_{i+1}}{d_i}`{=tex} \]

Similarly:

\[ `\frac{\partial d_i}{\partial x_{i+1}}`{=tex} =
`\frac{x_{i+1}-x_i}{d_i}`{=tex} \]

\[ `\frac{\partial d_i}{\partial y_{i+1}}`{=tex} =
`\frac{y_{i+1}-y_i}{d_i}`{=tex} \]

The complete gradient is obtained by summing the contributions from the
two neighboring segments of each intermediate waypoint.

For numerical stability, use a small value such as:

\[ d_i+`\epsilon`{=tex} \]

or:

\[ `\sqrt{\Delta x^2+\Delta y^2+\epsilon}`{=tex} \]

to avoid division by zero.

The gradient of the penalty terms must also be included.

------------------------------------------------------------------------

# 15. Hessian Analysis

The Hessian is:

\[ H(z)=`\nabla`{=tex}\^2F(z) \]

It contains the second-order partial derivatives.

The project must discuss:

-   What the Hessian represents
-   Whether it is positive definite
-   Whether it is positive semidefinite
-   Whether it is indefinite
-   How the Hessian changes with the waypoint configuration
-   What the Hessian implies about convexity

## Hessian computation options

The first implementation may use:

1.  Analytical Hessian for a simplified objective, or
2.  Numerical finite-difference Hessian

If a numerical Hessian is used, the report must explain the
finite-difference approximation.

For a gradient function (`\nabla `{=tex}F(z)):

\[ H\_{ij}(z) `\approx`{=tex} `\frac{
[\nabla F(z+h e_j)]_i-
[\nabla F(z-h e_j)]_i
}{2h}`{=tex} \]

where:

-   (e_j) is the (j)-th unit vector
-   \(h\) is a small finite-difference step

The Hessian should be symmetrized numerically if necessary:

\[ H`\leftarrow`{=tex}`\frac{H+H^T}{2}`{=tex} \]

------------------------------------------------------------------------

# 16. Positive Definiteness and Convexity

The project must explicitly analyze convexity.

## Positive definite

A symmetric Hessian is positive definite if:

\[ v\^THv\>0 \]

for every nonzero vector (v).

Equivalent numerical condition:

\[ `\lambda`{=tex}\_{`\min`{=tex}}(H)\>0 \]

## Positive semidefinite

A symmetric Hessian is positive semidefinite if:

\[ v\^THv`\geq0`{=tex} \]

for every vector (v).

Equivalent numerical condition:

\[ `\lambda`{=tex}\_{`\min`{=tex}}(H)`\geq0`{=tex} \]

within numerical tolerance.

## Indefinite

If the Hessian has both positive and negative eigenvalues, it is
indefinite.

## Convexity discussion

A function is convex on a region if its Hessian is positive semidefinite
throughout that region.

Important project point:

> The objective may be convex or approximately convex in a simplified
> setting, while the complete constrained drone problem may be
> non-convex because no-fly zones create forbidden regions and
> disconnected feasible regions.

The report must distinguish:

1.  Convexity of the objective
2.  Convexity of the feasible set
3.  Convexity of the complete optimization problem

Do not claim that the complete problem is convex only because the
objective looks convex.

------------------------------------------------------------------------

# 17. Lagrangian Formulation

For:

\[ `\min`{=tex}\_z F(z) \]

subject to:

\[ g_j(z)`\leq0`{=tex} \]

the Lagrangian is:

\[ `\mathcal{L}`{=tex}(z,`\mu`{=tex}) = F(z)+
`\sum`{=tex}\_{j=1}\^{m}`\mu`{=tex}\_jg_j(z) \]

where:

\[ `\mu`{=tex}\_j`\geq0`{=tex} \]

are Lagrange multipliers.

For example:

\[
`\mathcal{L}`{=tex}(z,`\mu`{=tex}\_B,`\mu`{=tex}*T,`\mu`{=tex}*{NFZ},`\ldots`{=tex})
= F(z) +`\mu`{=tex}\_Bg_B(z) +`\mu`{=tex}*Tg_T(z)
+`\sum`{=tex}*i`\mu`{=tex}*{NFZ,i}g*{NFZ,i}(z) +`\cdots`{=tex} \]

The report must explain the role of each multiplier.

------------------------------------------------------------------------

# 18. KKT Conditions

The Karush-Kuhn-Tucker conditions must be included.

For a candidate solution (z\^\*):

## 18.1 Stationarity

\[ `\nabla`{=tex}\_z`\mathcal{L}`{=tex}(z\^*,`\mu`{=tex}\^*)=0 \]

## 18.2 Primal Feasibility

\[ g_j(z\^\*)`\leq0`{=tex} \]

## 18.3 Dual Feasibility

\[ `\mu`{=tex}\_j\^\*`\geq0`{=tex} \]

## 18.4 Complementary Slackness

\[ `\mu`{=tex}\_j\^*g_j(z\^*)=0 \]

The report must explain that KKT conditions are necessary under suitable
regularity assumptions, but they are not automatically sufficient for a
globally optimal solution in a non-convex problem.

------------------------------------------------------------------------

# 19. Active and Inactive Constraints

A constraint is active if:

\[ g_j(z\^\*)=0 \]

A constraint is inactive if:

\[ g_j(z\^\*)\<0 \]

Examples:

-   Battery exactly used: active battery constraint
-   Battery remaining: inactive battery constraint
-   Waypoint exactly on safety boundary: active no-fly-zone constraint
-   Waypoint safely outside obstacle: inactive no-fly-zone constraint
-   Route reaches the maximum allowed time: active time constraint

The final report must list the constraint status for the final solution.

Recommended output:

  Constraint        Value   Limit   Slack Status
  --------------- ------- ------- ------- -----------------
  Battery             ...     ...     ... Active/Inactive
  Time                ...     ...     ... Active/Inactive
  No-fly zone 1       ...     ...     ... Active/Inactive
  No-fly zone 2       ...     ...     ... Active/Inactive
  Workspace x         ...     ...     ... Active/Inactive
  Workspace y         ...     ...     ... Active/Inactive

Use a numerical tolerance when deciding activity.

------------------------------------------------------------------------

# 20. Gradient Descent

Gradient Descent must be implemented from scratch.

Update rule:

\[ z\_{k+1} =
z_k-`\eta`{=tex}`\nabla `{=tex}F\_{`\text{penalty}`{=tex}}(z_k) \]

where:

-   (z_k): current waypoint vector
-   (`\eta`{=tex}): learning rate
-   (`\nabla `{=tex}F\_{`\text{penalty}`{=tex}}): gradient of the
    penalized objective

## Required GD features

-   Initial point
-   Learning rate
-   Maximum iterations
-   Stopping tolerance
-   Objective history
-   Gradient-norm history
-   Feasibility history
-   Runtime measurement
-   Final solution
-   Final constraint report

## Stopping criteria

Stop if:

\[ \|`\nabla `{=tex}F(z_k)\|\<`\epsilon`{=tex} \]

or:

\[ \|F(z\_{k+1})-F(z_k)\|\<`\epsilon`{=tex} \]

or the maximum number of iterations is reached.

The implementation must clearly state which stopping criterion was used.

------------------------------------------------------------------------

# 21. Heavy-Ball Momentum

Heavy-Ball must also be implemented from scratch.

The update rule is:

\[ z\_{k+1} =
z_k-`\eta`{=tex}`\nabla `{=tex}F\_{`\text{penalty}`{=tex}}(z_k)
+`\beta`{=tex}(z_k-z\_{k-1}) \]

where:

-   (`\eta`{=tex}): learning rate
-   (`\beta`{=tex}): momentum coefficient
-   (z_k-z\_{k-1}): previous movement direction

Initial condition:

\[ z\_{-1}=z_0 \]

or equivalently initialize the previous velocity to zero.

## Required Heavy-Ball features

-   Learning rate
-   Momentum coefficient
-   Initial waypoint vector
-   Maximum iterations
-   Objective history
-   Gradient-norm history
-   Feasibility history
-   Runtime
-   Final route
-   Final constraint report

The same initial point and comparable stopping conditions should be used
for GD and Heavy-Ball.

------------------------------------------------------------------------

# 22. Fair Comparison Protocol

GD and Heavy-Ball must be compared fairly.

Use:

-   Same environment
-   Same start and goal
-   Same waypoint count
-   Same initial waypoint configuration
-   Same objective
-   Same constraints
-   Same penalty coefficient
-   Same stopping tolerance
-   Same maximum iterations
-   Same numerical precision
-   Same evaluation metrics

The learning rate and momentum coefficient may be tuned, but the tuning
procedure must be reported honestly.

Do not compare one method using a carefully tuned parameter and the
other using a random parameter without explanation.

------------------------------------------------------------------------

# 23. Required Evaluation Metrics

For both GD and Heavy-Ball, record:

## Optimization metrics

-   Final objective value
-   Initial objective value
-   Number of iterations
-   Final gradient norm
-   Runtime
-   Convergence speed
-   Stability of objective decrease

## Trajectory metrics

-   Total distance
-   Estimated energy
-   Estimated time
-   Number of waypoints
-   Smoothness score
-   Maximum turn angle, if implemented

## Constraint metrics

-   Battery feasibility
-   Time feasibility
-   No-fly-zone feasibility
-   Workspace feasibility
-   Number of violated constraints
-   Maximum violation
-   Minimum obstacle clearance

## Quality metrics

-   Success/failure
-   Final route feasibility
-   Robustness to initialization
-   Sensitivity to learning rate
-   Sensitivity to penalty coefficient

------------------------------------------------------------------------

# 24. Condition Number

The condition number must be calculated because it is explicitly
relevant to convergence analysis.

For a symmetric positive definite Hessian:

\[ `\kappa`{=tex}(H)= `\frac{\lambda_{\max}(H)}`{=tex}
{`\lambda`{=tex}\_{`\min`{=tex}}(H)} \]

where:

-   (`\lambda`{=tex}\_{`\max`{=tex}}): largest eigenvalue
-   (`\lambda`{=tex}\_{`\min`{=tex}}): smallest positive eigenvalue

Interpretation:

-   (`\kappa`{=tex}`\approx1`{=tex}): well-conditioned
-   Large (`\kappa`{=tex}): ill-conditioned
-   Larger condition number often means slower or more difficult
    convergence for first-order methods

If the Hessian is singular, indefinite, or not positive definite, the
report must not blindly use the standard positive-definite condition
number. It should explain the issue and use an appropriate numerical
interpretation.

The condition number should be evaluated:

1.  At the initial point
2.  Near the final point
3.  Optionally at multiple points along the optimization path

------------------------------------------------------------------------

# 25. Required Visualizations

The project must include clear plots.

## Plot 1: Initial trajectory

Show:

-   Start
-   Goal
-   Waypoints
-   No-fly zones
-   Initial path

## Plot 2: Final GD trajectory

Show the optimized route obtained by Gradient Descent.

## Plot 3: Final Heavy-Ball trajectory

Show the optimized route obtained by Heavy-Ball.

## Plot 4: GD convergence

Plot:

\[ F(z_k)`\quad`{=tex}`\text{against iteration}`{=tex} \]

## Plot 5: Heavy-Ball convergence

Plot:

\[ F(z_k)`\quad`{=tex}`\text{against iteration}`{=tex} \]

## Plot 6: Combined convergence comparison

Plot GD and Heavy-Ball on the same axes.

Use a logarithmic y-axis if appropriate.

## Plot 7: Gradient norm

Plot:

\[ \|`\nabla `{=tex}F(z_k)\| \]

against iteration.

## Plot 8: Constraint violation

Plot total or maximum constraint violation against iteration.

## Plot 9: Objective versus waypoint count

Run experiments for:

\[ N=5,10,20,40 \]

or another manageable set.

Discuss the trade-off between:

-   Route flexibility
-   Computational cost
-   Smoothness
-   Feasibility
-   Convergence

------------------------------------------------------------------------

# 26. Experimental Plan

## Experiment A: Basic optimization

-   Use a simple map
-   Use one no-fly zone
-   Use a small number of waypoints
-   Run GD
-   Run Heavy-Ball
-   Verify feasibility
-   Plot trajectories

## Experiment B: Multiple obstacles

-   Add more no-fly zones
-   Repeat GD and Heavy-Ball
-   Compare feasibility and convergence

## Experiment C: Learning-rate sensitivity

Try several learning rates:

\[ `\eta`{=tex}`\in`{=tex}{0.001,0.01,0.05,0.1} \]

The exact values depend on objective scaling.

Record:

-   Convergence
-   Divergence
-   Oscillation
-   Final objective
-   Feasibility

## Experiment D: Momentum sensitivity

Try several momentum values:

\[ `\beta`{=tex}`\in`{=tex}{0.5,0.7,0.9,0.95} \]

Record the same metrics.

## Experiment E: Waypoint-count study

Try:

\[ N`\in`{=tex}{5,10,20,40} \]

Discuss whether more waypoints improve route quality and how they affect
computational cost.

## Experiment F: Initialization sensitivity

Use multiple initial waypoint configurations.

Compare:

-   Straight-line initialization
-   Slightly perturbed initialization
-   Obstacle-avoiding initialization

This is important because the full problem may be non-convex.

------------------------------------------------------------------------

# 27. Recommended Code Structure

The code should be modular and readable.

``` text
project/
|
|-- README.md
|-- optimization_specification.md
|
|-- data/
|   |-- environment_config.json
|   |-- experiment_results.csv
|
|-- src/
|   |-- environment.py
|   |-- objective.py
|   |-- constraints.py
|   |-- derivatives.py
|   |-- feasibility.py
|   |-- gradient_descent.py
|   |-- heavy_ball.py
|   |-- metrics.py
|   |-- visualization.py
|   |-- experiments.py
|
|-- notebooks/
|   |-- optimization_experiments.ipynb
|
|-- results/
|   |-- trajectories/
|   |-- convergence/
|   |-- tables/
|   |-- logs/
|
|-- report/
|   |-- final_report.md
```

## Recommended classes/functions

### Environment

-   `Environment2D`
-   Store map bounds
-   Store start and goal
-   Store no-fly zones

### Objective

-   `total_distance()`
-   `energy_cost()`
-   `time_cost()`
-   `smoothness_cost()`
-   `objective()`

### Constraints

-   `battery_constraint()`
-   `time_constraint()`
-   `no_fly_zone_constraint()`
-   `workspace_constraint()`
-   `all_constraints()`

### Derivatives

-   `gradient()`
-   `numerical_hessian()`
-   `eigenvalue_analysis()`

### Optimization

-   `GradientDescentOptimizer`
-   `HeavyBallOptimizer`

Both should expose a consistent interface such as:

``` python
optimizer.fit(initial_point)
optimizer.predict()
```

or equivalent clearly documented methods.

### Feasibility

-   `check_waypoints()`
-   `check_segments()`
-   `check_battery()`
-   `check_time()`
-   `check_all_constraints()`

### Metrics

-   `calculate_distance()`
-   `calculate_energy()`
-   `calculate_time()`
-   `calculate_condition_number()`
-   `calculate_constraint_violation()`

------------------------------------------------------------------------

# 28. Implementation Rules

The project must prioritize mathematical understanding.

## Allowed tools

Use basic numerical and plotting tools such as:

-   `numpy`
-   `pandas`
-   `matplotlib`
-   Python standard library

If the instructor has restricted libraries, follow those restrictions
exactly.

## Avoid

-   Calling a pre-built optimizer such as `scipy.optimize.minimize`
-   Calling a ready-made Gradient Descent implementation
-   Calling a ready-made Heavy-Ball optimizer
-   Using an automatic optimization framework that hides the mathematics
-   Claiming an algorithm was implemented from scratch when it was only
    imported

The optimization update equations must be written manually.

------------------------------------------------------------------------

# 29. Important Non-Convexity Discussion

The report must explain that the problem can become non-convex due to:

-   Circular forbidden regions
-   Multiple obstacles
-   Disconnected feasible regions
-   Penalty terms
-   Nonlinear distance expressions
-   Initialization dependence

Therefore:

-   GD may converge to a local solution
-   Heavy-Ball may converge faster but can oscillate
-   Different initializations may produce different routes
-   A feasible solution is not automatically globally optimal
-   KKT satisfaction does not automatically prove global optimality in a
    non-convex problem

This should be discussed in the results and reflection sections.

------------------------------------------------------------------------

# 30. Expected Results Table

The final comparison should include a table like:

  Metric                         Gradient Descent   Heavy-Ball
  ---------------------------- ------------------ ------------
  Final objective                             ...          ...
  Total distance                              ...          ...
  Estimated energy                            ...          ...
  Estimated time                              ...          ...
  Iterations                                  ...          ...
  Runtime                                     ...          ...
  Final gradient norm                         ...          ...
  Minimum obstacle clearance                  ...          ...
  Constraint violations                       ...          ...
  Feasible route?                          Yes/No       Yes/No
  Condition number                            ...          ...
  Convergence behavior                        ...          ...

Do not invent results before running the experiments.

------------------------------------------------------------------------

# 31. Report Structure

## Chapter 1: Introduction

-   Drone delivery motivation
-   Why trajectory optimization matters
-   Why 2D is used first
-   Project objectives

## Chapter 2: Problem Formulation

-   Environment
-   Decision variables
-   Objective
-   Constraints
-   Mathematical formulation

## Chapter 3: Mathematical Analysis

-   Gradient
-   Hessian
-   Eigenvalues
-   Positive definiteness
-   Convexity
-   Lagrangian
-   KKT
-   Active/inactive constraints

## Chapter 4: Algorithms

-   Gradient Descent
-   Heavy-Ball Momentum
-   Constraint penalty
-   Stopping criteria

## Chapter 5: Implementation

-   Code structure
-   Environment setup
-   Initial trajectory
-   Feasibility checker
-   Optimization pipeline

## Chapter 6: Experiments

-   Basic case
-   Multiple obstacles
-   Learning-rate study
-   Momentum study
-   Waypoint-count study
-   Initialization study

## Chapter 7: Results

-   Tables
-   Trajectory plots
-   Convergence plots
-   Condition number
-   Feasibility analysis

## Chapter 8: Reflection

-   What worked
-   What failed
-   GD vs Heavy-Ball
-   Effect of condition number
-   Effect of non-convexity
-   Limitations
-   Future RL extension

## Chapter 9: Conclusion

-   Summary
-   Best method for the tested environment
-   Main mathematical findings
-   Future work

------------------------------------------------------------------------

# 32. Reflection Questions

The final reflection must answer:

1.  Which method converged faster?
2.  Which method obtained the lower objective?
3.  Did faster convergence also produce a better route?
4.  Was the final route feasible?
5.  Which constraints were active?
6.  Which constraints were inactive?
7.  What did the Hessian reveal?
8.  Was the objective convex?
9.  Was the complete constrained problem convex?
10. What was the condition number?
11. How did conditioning affect convergence?
12. Did Heavy-Ball oscillate?
13. Did the initial point affect the result?
14. Did more waypoints improve the trajectory?
15. What are the limitations of the simplified energy model?
16. What would be improved in a future 3D/RL version?

------------------------------------------------------------------------

# 33. Definition of Completion

The optimization phase is complete only when:

-   [ ] The 2D environment is defined.
-   [ ] Start and goal are fixed.
-   [ ] Intermediate waypoints are decision variables.
-   [ ] Objective function is implemented.
-   [ ] Constraints are implemented.
-   [ ] Feasibility checker is implemented.
-   [ ] Gradient is derived and implemented.
-   [ ] Hessian is calculated.
-   [ ] Eigenvalues are analyzed.
-   [ ] Convexity is discussed.
-   [ ] Lagrangian is written.
-   [ ] KKT conditions are explained.
-   [ ] Active/inactive constraints are identified.
-   [ ] Gradient Descent is implemented from scratch.
-   [ ] Heavy-Ball is implemented from scratch.
-   [ ] Both algorithms use the same initial conditions.
-   [ ] Both algorithms are compared fairly.
-   [ ] Condition number is calculated.
-   [ ] Initial and final trajectories are plotted.
-   [ ] Convergence plots are generated.
-   [ ] Constraint violations are reported.
-   [ ] Final metrics table is created.
-   [ ] Reflection is written.
-   [ ] Limitations are documented.
-   [ ] Code is organized and reproducible.

Only after these items are complete should the RL extension be
considered.

------------------------------------------------------------------------

# 34. Future Continuity: RL Extension

The RL component is not part of the current implementation.

After the optimization layer is stable, the same 2D environment may
later be reused for RL.

Possible future structure:

``` text
Completed Optimization Model
            |
            v
      2D Simulation
            |
            v
      RL Environment
            |
            v
      PPO or SAC
            |
            v
  Compare Learned Policy
  with Optimization Route
```

The future RL version may compare:

-   Energy
-   Distance
-   Time
-   Collision rate
-   Success rate
-   Generalization
-   Adaptation to changing obstacles

But the current project must remain complete even if RL is never
implemented.

------------------------------------------------------------------------

# 35. Final Instruction to the Project Agent

Build this project in the following order:

1.  Read and understand this specification.
2.  Create the 2D environment.
3.  Define the mathematical problem.
4.  Implement the objective and constraints.
5.  Implement feasibility checking.
6.  Derive and implement the gradient.
7.  Calculate and analyze the Hessian.
8.  Explain convexity.
9.  Formulate the Lagrangian.
10. Explain and evaluate KKT conditions.
11. Implement Gradient Descent from scratch.
12. Implement Heavy-Ball Momentum from scratch.
13. Run both methods under matched conditions.
14. Generate trajectory and convergence plots.
15. Calculate the condition number.
16. Analyze active and inactive constraints.
17. Create the final comparison table.
18. Write the reflection and limitations.
19. Confirm that all instructor requirements are satisfied.
20. Do not begin RL until the optimization checklist is complete.

The main principle is:

> **First build a mathematically correct, feasible, and well-analyzed 2D
> optimization system. RL is only a future continuity extension.**
