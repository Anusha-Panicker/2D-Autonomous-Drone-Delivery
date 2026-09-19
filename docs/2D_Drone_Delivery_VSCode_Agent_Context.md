2D Autonomous Drone Delivery — Project Context & VS Code Agent Handoff

Purpose of this file

This file contains the important context, decisions, requirements, current implementation status, rubric alignment, known gaps, and recommended next steps for the academic project:

Project: 2D Autonomous Drone Delivery: Constrained Trajectory Optimization

Use this file as the project context / handoff document for a VS Code coding agent.
The agent should read this before modifying the repository.

1. Project Goal

The project is a 2D autonomous drone trajectory optimization problem.

The main goal is to find a collision-free route from a fixed start point to a fixed goal point while optimizing a combination of:

flight time

energy

obstacle/NFZ avoidance

battery feasibility

workspace feasibility

The current project is based on direct constrained trajectory optimization, not Reinforcement Learning.

Important

Do not pivot the project to RL, deep learning, camera perception, 3D simulation, AirSim, or dynamic-obstacle planning unless explicitly requested later.

The current optimization project should be completed first because the grading rubric is primarily about:

Problem formulation

Convexity and KKT

GD / Heavy-Ball implementation and convergence

Reflection

Viva

2. Current Repository

GitHub repository:

https://github.com/Anusha-Panicker/2D-Autonomous-Drone-Delivery

Repository title:

2D Autonomous Drone Delivery: Constrained Trajectory Optimization

Current important structure:

.
├── data/
├── docs/
├── results/
├── scripts/
├── src/
├── .github/
├── README.md
├── requirements.txt
└── ...

Important source files:

src/environment.py
src/objective.py
src/constraints.py
src/derivatives.py
src/gradient_descent.py
src/heavy_ball.py
src/feasibility.py
src/visualization.py
src/experiments.py

Dataset:

data/drone_optimization_dataset.json

The dataset currently contains 50+ scenarios according to the project work completed so far.

3. Academic Rubric

The grading distribution is:

Component

Marks

Problem Formulation

20

Convexity / KKT

25

Implementation / Convergence

25

Reflection

15

Viva

15

Total

100

3.1 Problem Formulation — 20 marks

Need to clearly show:

decision variables

objective function

constraints

units

physical/context meaning

consistency between mathematical formulation and implementation

The written formulation and actual code must agree.

3.2 Convexity / KKT — 25 marks

Need:

Hessian

eigenvalue reasoning

convexity classification

Lagrangian

KKT conditions

numerical KKT checking

active constraints

inactive constraints

justification for active/inactive status

Important:

Simply labeling constraints as active/inactive is not enough to prove KKT.

We should eventually compute:

primal feasibility

stationarity residual

dual feasibility

complementary slackness

estimated Lagrange multipliers where possible

3.3 Implementation / Convergence — 25 marks

Need:

Gradient Descent

Heavy-Ball

both implemented from scratch

fair comparison

same initial conditions

convergence plots

gradient norm

constraint violations

final metrics

condition number κ

explanation of how conditioning affects convergence

Important:

Do not simply say:

Heavy-Ball is faster.

That conclusion must come from the actual experiment results.

3.4 Reflection — 15 marks

Reflection should be based on actual experimental observations.

Potential questions:

Which method converged faster?

Which reached lower objective?

What happened to route quality?

Were final routes feasible?

Which constraints were active?

Which constraints were inactive?

What did the Hessian/eigenvalues show?

Is the complete problem convex?

What was the condition number?

Did Heavy-Ball oscillate?

How did initialization affect the solution?

How did waypoint count affect the result?

What are the limitations of the simplified energy model?

What would be done in future work?

How could the system extend to 3D or RL later?

Do not invent answers before running experiments.

4. Mathematical Formulation

The decision vector consists of the interior 2D waypoints.

If there are N waypoints/segments as defined by the implementation, the optimization vector contains the interior points:

[
x = (P_1,\ldots,P_{N-1}) \in \mathbb{R}^{2(N-1)}
]

where each waypoint is:

[
P_i = (x_i,y_i)
]

The start and goal are fixed.

The trajectory is therefore represented by:

Start → P1 → P2 → ... → P(N-1) → Goal

5. Objective

The objective combines time and energy.

Conceptually:

[
F(x) = \alpha , T(x) + \beta , E(x)
]

with normalized/scaled components as implemented in the repository.

The repository currently supports delivery modes approximately as:

Battery Saving:
    alpha = 0.2
    beta  = 0.8

Express:
    alpha = 0.9
    beta  = 0.1

Balanced:
    alpha = 0.5
    beta  = 0.5

Important limitation:

The current simplified model uses fixed-speed / distance-proportional energy behavior. Therefore, changing the time/energy weights may produce very similar routes.

This is a known limitation, not something to hide.

A richer future model could make speed, acceleration, payload, battery consumption, etc. affect the trajectory more realistically.

6. Constraints

The project includes:

6.1 No-Fly Zone (NFZ)

Circular no-fly zones are represented with a safety buffer.

For a waypoint P_i and obstacle center C:

[
R_{\text{safe}}^2 - |P_i-C|^2 \le 0
]

This means the waypoint must remain outside the buffered obstacle.

The implementation also checks path segments, not only waypoint locations.

6.2 Battery

Battery constraint is represented approximately as:

[
E(x) - E_{\max,\text{usable}} \le 0
]

The actual implementation checks energy against the usable battery budget.

6.3 Time

Time constraint is approximately:

[
T(x) - T_{\max} \le 0
]

6.4 Workspace Bounds

The route must stay inside the allowed 2D workspace.

6.5 Segment-Level NFZ Collision

A critical implementation detail:

It is not sufficient for all waypoints to be outside an obstacle.

A straight segment between two safe waypoints could still pass through an obstacle.

The project therefore includes:

segment sampling in derivative/penalty calculations

exact segment-circle clearance checking in the feasibility checker using projection of the circle center onto the segment

This should remain intact.

7. Penalty Method

The optimizer currently works with a penalized objective.

Conceptually:

F_{\text{objective}}
+
\rho
\sum_j \max(0,g_j(x))^2
]

where:

g_j(x) <= 0 represents a constraint

rho is the penalty coefficient

Important distinction for the report:

Original constrained objective

The physical optimization objective:

[
F_{\text{objective}}
]

Penalized optimization objective

The function actually optimized by unconstrained GD/HB:

[
F_{\text{penalty}}
]

Do not confuse these two in the final report.

Also distinguish:

physical objective value

penalty contribution

final feasibility

constraint violation

8. Gradient Descent

src/gradient_descent.py

Gradient Descent is implemented from scratch.

Core update:

[
z_{k+1}=z_k-\eta\nabla F(z_k)
]

where:

z = waypoint decision vector

eta = learning rate

F = penalized objective

The implementation records:

objective history

gradient norm

feasibility

constraint violation

runtime

iteration count

It also uses practical safeguards such as:

maximum step clipping

workspace clipping

stopping based on gradient norm or objective change

The step clipping value currently used is approximately:

max_step = 0.25

This is a numerical safeguard and should be explained if asked in viva.

9. Heavy-Ball

src/heavy_ball.py

Heavy-Ball is also implemented from scratch.

Core update:

z_k

\eta\nabla F(z_k)
+
\beta(z_k-z_{k-1})
]

where:

eta = learning rate

beta = momentum parameter

It records the same type of diagnostics as GD.

It uses the same practical safeguards, including step clipping and workspace clipping.

10. Fair GD vs Heavy-Ball Comparison

This is the immediate priority.

The comparison must be scientifically fair.

For every scenario:

Keep identical

environment

start

goal

obstacles

battery

time limits

waypoint count

objective weights

penalty coefficient

initial waypoint vector z0

stopping tolerance

maximum iterations

step clipping rules

workspace handling

Change only

The optimization method:

GD
vs
Heavy-Ball

Do not change the problem between the two methods.

11. Important Current Issue in experiments.py

src/experiments.py currently has:

scenario = dataset[0]

inside the main experiment flow.

Therefore, the current main experiment only demonstrates one scenario.

This is not sufficient for the final 50+ scenario evaluation.

Required improvement

Run the GD vs Heavy-Ball experiment across all available scenarios in:

data/drone_optimization_dataset.json

Then aggregate the results.

12. Main Immediate Experiment

The required experiment should be:

50+ scenarios
      ↓
same scenario setup
      ↓
same initialization
      ↓
run GD
      ↓
run Heavy-Ball
      ↓
collect metrics
      ↓
compare
      ↓
aggregate statistics
      ↓
plots
      ↓
interpretation

For each scenario, collect at least:

scenario ID

optimizer

initial objective

final objective

final penalized objective

distance

time

energy

feasible/not feasible

maximum constraint violation

final gradient norm

iterations

runtime

minimum obstacle clearance

Hessian eigenvalue information where practical

condition number κ where mathematically meaningful

active/inactive constraints

13. Aggregate Statistics

After all scenarios are run, calculate at least:

Per optimizer

mean final objective

standard deviation of final objective

median final objective

mean iterations

median iterations

mean runtime

median runtime

mean final gradient norm

feasibility rate

mean/max constraint violation

mean minimum clearance

mean condition number where valid

Potential comparison table:

Metric

GD

Heavy-Ball

Mean objective

...

...

Median objective

...

...

Std objective

...

...

Mean iterations

...

...

Median iterations

...

...

Mean runtime

...

...

Feasibility rate

...

...

Mean final gradient norm

...

...

Mean violation

...

...

Do not automatically call one optimizer the "winner".

Report the measured differences and explain what they imply.

14. Convergence Plots

The final project should contain clear convergence plots.

At minimum:

Plot 1 — Objective vs Iteration

Compare:

GD
Heavy-Ball

on the same scenario with the same initial point.

Use the same objective definition for both.

Plot 2 — Gradient Norm vs Iteration

Plot:

[
|\nabla F(x_k)|
]

Prefer a log scale if useful.

Plot 3 — Constraint Violation vs Iteration

Show whether the optimization becomes feasible.

Plot 4 — Trajectory Comparison

Show:

Initial trajectory
GD trajectory
Heavy-Ball trajectory
NFZs
Start
Goal

The final report does not need a trajectory figure for every scenario.

Use a few representative scenarios in the main report and put the full set/statistics in results or appendix.

15. Condition Number κ

The rubric explicitly expects discussion of condition number.

For a positive-definite Hessian:

[
\kappa =
\frac{\lambda_{\max}}{\lambda_{\min}}
]

where:

lambda_max = largest eigenvalue

lambda_min = smallest positive eigenvalue

Interpretation:

κ close to 1 → better conditioned

large κ → ill-conditioned

ill-conditioning can cause gradient descent to progress slowly because curvature differs significantly between directions

Important:

Do not blindly calculate:

[
\lambda_{\max}/\lambda_{\min}
]

when the Hessian is:

indefinite

singular

negative definite

nearly singular

The repository already has Hessian/eigenvalue analysis.

Condition number should be reported only where the mathematical interpretation is valid.

If the Hessian is singular/indefinite, report that clearly rather than forcing a misleading κ.

16. Hessian / Convexity

src/analysis.py provides:

numerical Hessian

eigenvalues

symmetry check

Hessian classification

condition number

The analysis classifies cases such as:

positive definite
positive semidefinite
indefinite
negative semidefinite

Need to use this in the final report.

Important mathematical distinction:

Even if the objective Hessian is positive definite/PSD in a region, the complete constrained trajectory problem is not automatically convex.

The obstacle constraint:

[
|P_i-C|^2 \ge R^2
]

describes the outside of a disk.

The feasible set outside a disk is nonconvex.

Therefore:

Do not claim global convexity or global optimality merely because the objective Hessian looks convex.

Also:

Do not claim global optimality from GD/HB convergence.

The optimizers are local numerical methods and the obstacle geometry can produce multiple feasible route choices/local minima.

17. KKT Analysis — Required Improvement

Current code already reports:

constraint values

slack

active/inactive/violated status

However, this is not a full KKT verification.

A stronger final project should add numerical KKT analysis.

For constraints:

[
g_j(x)\le0
]

the KKT conditions are:

Primal feasibility

[
g_j(x^*)\le0
]

Dual feasibility

[
\lambda_j\ge0
]

Stationarity

0
]

Complementary slackness

[
\lambda_jg_j(x^*)=0
]

Recommended implementation:

Create something like:

src/kkt_analysis.py

The exact approach to estimating multipliers should be numerically justified.

At minimum, report:

primal violation

stationarity residual

multiplier values

negative multiplier violations

complementary slackness residual

active constraints

Do not pretend numerical KKT is exact.

Use tolerances and describe it as a numerical check.

18. Active / Inactive Constraints

A constraint is approximately active when:

[
|g_j(x)| \approx 0
]

A constraint is inactive when it has clear positive slack.

For g(x) <= 0:

[
\text{slack}=-g(x)
]

The report should show examples such as:

Battery:
    inactive
    slack = ...

Time:
    active
    slack = ...

NFZ #1:
    active
    clearance = ...

NFZ #2:
    inactive
    clearance = ...

Use actual experiment results.

Do not fabricate values.

19. Multiple Initializations

The project already has infrastructure for:

straight
offset
perturbed

initialization strategies.

This is useful because the problem can be nonconvex.

Recommended experiment:

Run the same scenario from several initial points.

Compare:

final route

final objective

feasibility

convergence

final solution differences

This can provide evidence of sensitivity to initialization and local minima.

This is useful, but it is secondary to completing the 50+ scenario GD/HB comparison.

20. Waypoint Count Study

Current experiments include waypoint counts approximately:

5
10
20
40

The project specification also suggested studying discretization such as:

N = 10, 25, 50, 100

Do not add this at the expense of the core GD/HB experiment.

For the final report, discuss the tradeoff:

More waypoints:

potentially more flexible trajectory

higher-dimensional optimization

more computation

potentially harder conditioning/convergence

Fewer waypoints:

cheaper optimization

less trajectory flexibility

potentially poorer obstacle avoidance/path representation

21. Feasibility Checking

src/feasibility.py performs important final checks.

It checks:

finite coordinates

workspace bounds

waypoint obstacle clearance

segment-obstacle collision

battery

time

The exact segment-circle projection check is important and should not be removed.

The final evaluation should use the feasibility checker rather than relying only on penalty values.

22. Derivatives

src/derivatives.py contains analytical gradient logic for:

distance

time/energy weighting

battery penalty

time penalty

waypoint NFZ penalty

sampled segment NFZ penalties

This should be preserved.

Any changes to the objective or constraints must also update:

derivative calculations

feasibility checking

metrics

experiments

documentation

The mathematical formulation and code must remain consistent.

23. README Limitation

The README already acknowledges an important limitation:

Because the current model uses fixed speed and energy proportional to distance, changing delivery-mode weights can lead to similar routes.

A richer model would be needed to create stronger time-vs-energy tradeoffs.

This should be explicitly mentioned in the reflection/limitations section.

24. Recommended Final Experiment Architecture

A clean architecture would be:

run_all_scenarios
    |
    +-- load dataset
    |
    +-- for each scenario
    |       |
    |       +-- create environment
    |       +-- create same z0
    |       +-- run GD
    |       +-- run Heavy-Ball
    |       +-- feasibility check
    |       +-- collect metrics
    |       +-- Hessian / kappa
    |       +-- KKT analysis
    |
    +-- save raw per-scenario results
    |
    +-- aggregate statistics
    |
    +-- generate plots
    |
    +-- save final summary CSV/JSON

Recommended files:

src/kkt_analysis.py

scripts/run_50_scenarios.py
scripts/run_kkt_analysis.py
scripts/run_kappa_analysis.py

Exact filenames can be adapted to the repository style.

25. Suggested Results Folder

Something like:

results/
├── metrics/
├── convergence/
├── trajectories/
├── kkt/
├── kappa/
├── aggregate/
└── summary/

Potential outputs:

gd_hb_all_scenarios.csv
gd_hb_summary.json
gd_hb_summary.csv
kkt_results.csv
kappa_results.csv

Avoid generating unnecessary files for every intermediate experiment if they make the repository messy.

26. Recommended Final Figures

The final report should ideally contain:

Figure 1

Initial route + environment

Figure 2

GD vs Heavy-Ball final trajectories

Figure 3

Objective convergence

Figure 4

Gradient norm convergence

Figure 5

Constraint violation convergence

Figure 6

Condition number / conditioning analysis

Figure 7

Waypoint-count tradeoff

Figure 8

Aggregate GD vs Heavy-Ball performance across 50+ scenarios

The main report can use representative scenarios.

The full scenario results can go into appendix/results.

27. What Is Already Mostly Implemented

The repository already has substantial core functionality:

2D environment

dataset with 50+ scenarios

waypoint representation

objective

constraints

analytical gradient

numerical Hessian

eigenvalue analysis

convexity classification

GD from scratch

Heavy-Ball from scratch

feasibility checker

trajectory visualization

constraint report

metrics

sensitivity experiments

multiple initialization experiments

waypoint count experiments

Therefore:

Do NOT rewrite everything.

The main task now is to turn the existing implementation into a rigorous experimental evaluation.

28. Biggest Current Gaps

Priority order:

Priority 1 — 50+ scenario GD vs Heavy-Ball comparison

Current experiments.py mainly runs:

scenario = dataset[0]

Change this so the final experiment evaluates all scenarios.

Priority 2 — Aggregate metrics

Create:

mean

median

standard deviation

feasibility rate

iterations

runtime

final objective

gradient norm

constraint violation

for GD and Heavy-Ball.

Priority 3 — Condition number analysis

Calculate/report κ where mathematically valid.

Then explain observed convergence differences using conditioning.

Priority 4 — Stronger KKT verification

Add numerical KKT residual checks.

Priority 5 — Final convergence figures

Produce clean:

objective

gradient norm

constraint violation

trajectory comparison

plots.

Priority 6 — Reflection

Write the reflection only after the actual experiments are complete.

Priority 7 — Viva preparation

Prepare concise explanations for:

decision variables

objective

constraints

penalty method

Hessian

eigenvalues

convexity

KKT

GD

Heavy-Ball

learning rate

momentum

conditioning

active constraints

initialization

limitations

29. Things NOT to Do Yet

Do not spend time on:

RL implementation

deep learning

computer vision

drone camera simulation

3D trajectory

AirSim

dynamic obstacles

complicated real-world flight physics

large UI/dashboard work

unless the core project is already complete and the user explicitly asks for these.

The current grading marks are more directly supported by rigorous optimization experiments.

30. Important Scientific Reporting Rules

The VS Code agent must follow these rules.

Rule 1 — Never fabricate experimental results

If a result has not been run, do not invent it.

Use:

TODO: run experiment

or implement the experiment.

Rule 2 — Never claim Heavy-Ball is faster without data

The correct process is:

run experiment
→ measure
→ compare
→ interpret

Possible outcomes include:

Heavy-Ball converges faster

GD converges faster under some settings

difference is small

Heavy-Ball oscillates

one method reaches feasibility earlier

final objectives are similar

Report what actually happens.

Rule 3 — Do not claim global optimality

Do not write:

The algorithm found the global optimum.

Unless there is a mathematically valid global-optimality proof, which is not currently expected.

Prefer:

The optimizer converged to a feasible local solution from the specified initialization.

Rule 4 — Distinguish objective and penalty

Always make clear whether a reported number is:

physical objective

or:

penalized objective

Rule 5 — Distinguish convex objective from complete problem

A locally/analytically convex-looking objective does not imply that the obstacle-constrained problem is convex.

Rule 6 — Explain numerical safeguards

If using:

clipping

tolerances

penalty coefficients

workspace projection

document why they are used.

31. Suggested Coding Workflow for VS Code Agent

When asked to modify the repository:

Step 1

Inspect:

src/experiments.py
src/metrics.py
src/derivatives.py
src/constraints.py
src/feasibility.py
src/gradient_descent.py
src/heavy_ball.py
src/analysis.py

before changing anything.

Step 2

Understand the existing APIs.

Do not duplicate functionality unnecessarily.

Step 3

Implement the smallest change needed.

Step 4

Run a small smoke test first.

For example:

1 scenario
GD + HB

Step 5

Then run:

all 50+ scenarios

Step 6

Check:

no crashes

finite metrics

feasibility

sensible objective values

no missing scenarios

both optimizers use same initial conditions

Step 7

Generate aggregate results.

Step 8

Only then add/refine plots and report outputs.

32. Reproducibility

The final project should be reproducible.

Document:

dataset path

random seeds

waypoint count

learning rate

Heavy-Ball momentum

penalty coefficient

stopping tolerance

maximum iterations

initialization method

environment configuration

Python dependencies

If random initialization is used, set and record a seed.

33. Final Report Structure Recommendation

A strong final report can follow:

1. Introduction
2. Problem Formulation
3. Mathematical Model
   3.1 Decision Variables
   3.2 Objective
   3.3 Constraints
4. Convexity Analysis
   4.1 Hessian
   4.2 Eigenvalues
   4.3 Convexity
5. KKT Conditions
   5.1 Lagrangian
   5.2 Numerical KKT Verification
   5.3 Active / Inactive Constraints
6. Optimization Methods
   6.1 Gradient Descent
   6.2 Heavy-Ball
7. Experimental Setup
8. Results
   8.1 Representative Trajectories
   8.2 Convergence
   8.3 Feasibility
   8.4 Condition Number
   8.5 50+ Scenario Aggregate Results
   8.6 Waypoint Sensitivity
9. Discussion
10. Limitations
11. Reflection
12. Conclusion
13. References
14. Appendix

34. Final Metrics Checklist

Before declaring the project complete, verify that the final results include:

[ ] 50+ scenarios evaluated
[ ] GD evaluated on every scenario
[ ] Heavy-Ball evaluated on every scenario
[ ] same initial condition for GD/HB
[ ] same objective/constraints
[ ] final objective
[ ] penalized objective
[ ] runtime
[ ] iterations
[ ] gradient norm
[ ] constraint violation
[ ] feasibility
[ ] minimum clearance
[ ] battery status
[ ] time status
[ ] NFZ status
[ ] active/inactive constraints
[ ] Hessian eigenvalues
[ ] convexity classification
[ ] condition number where valid
[ ] KKT residuals
[ ] convergence plots
[ ] trajectory plots
[ ] aggregate statistics
[ ] waypoint study
[ ] reproducibility information
[ ] limitations
[ ] reflection
[ ] viva preparation

35. Immediate Next Task

The immediate next task is:

Audit and improve the existing GD vs Heavy-Ball experiment so it can run fairly across all 50+ scenarios and produce clean aggregate results.

Start with:

src/experiments.py
src/metrics.py
src/gradient_descent.py
src/heavy_ball.py
src/derivatives.py
src/constraints.py
src/feasibility.py
src/analysis.py

Then:

preserve existing functionality

add an all-scenario experiment

ensure identical initial conditions

collect per-scenario metrics

save machine-readable results

calculate aggregate statistics

create convergence/trajectory plots

add κ analysis

add KKT analysis after the GD/HB pipeline is stable

36. High-Level Project Status

Current status:

Problem formulation              → largely implemented
2D environment                   → implemented
50+ scenario dataset             → implemented
Objective                        → implemented
Constraints                      → implemented
Feasibility checker              → implemented
Gradient                         → implemented
Hessian/eigenvalue analysis      → implemented
GD                               → implemented
Heavy-Ball                       → implemented
Visualization                    → implemented
Basic experiments                → implemented
All-scenario GD/HB comparison    → NEEDS TO BE COMPLETED
Aggregate statistics             → NEEDS TO BE COMPLETED
κ analysis/reporting             → NEEDS TO BE STRENGTHENED
Numerical KKT verification       → NEEDS TO BE STRENGTHENED
Final convergence figures        → NEEDS TO BE COMPLETED
Final reflection                 → AFTER EXPERIMENTS
Viva preparation                 → FINAL STAGE
RL / 3D / advanced simulation    → NOT REQUIRED NOW

37. Instruction to the VS Code Agent

Treat this document as project context.

Before coding:

inspect the current repository

inspect existing implementations

do not rewrite working components unnecessarily

preserve mathematical consistency

preserve reproducibility

avoid fabricated results

test changes incrementally

report exactly what was changed

report any assumptions

identify any mismatch between documentation and implementation

When making optimization-related changes, always check whether the change affects:

objective
→ gradient
→ constraints
→ feasibility
→ metrics
→ experiments
→ plots
→ documentation

The final goal is not merely "working code".

The goal is a defensible academic optimization project where:

the mathematics matches the implementation,

the GD/HB comparison is fair,

the 50+ scenarios provide meaningful evidence,

convexity/KKT claims are numerically supported,

conditioning is discussed correctly,

feasibility is explicitly verified,

limitations are honestly documented,

and no unsupported global-optimality claim is made.

38. Useful External Project Reference

GitHub repository:

https://github.com/Anusha-Panicker/2D-Autonomous-Drone-Delivery

Use the repository's current code as the source of truth for implementation details, and use the project specification/rubric documents for academic requirements.

