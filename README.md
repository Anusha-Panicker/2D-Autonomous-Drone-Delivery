# 2D Autonomous Drone Delivery: Constrained Trajectory Optimization

This project implements a constrained trajectory optimization system for a 2D drone delivery environment. The primary goal is to minimize both **flight time** and **energy consumption** while ensuring the drone avoids **No-Fly Zones (NFZ)** and adheres to **battery capacity** limits.

## 🚀 Core Objectives
- **Optimization**: Use Gradient Descent and Heavy-Ball Momentum to find an efficient, safe route.
- **Constraints**: 
  - Avoid circular obstacles (No-Fly Zones).
  - Maintain safety buffers around obstacles.
  - Stay within defined map boundaries.
  - Ensure the total energy cost does not exceed the available battery.
- **Comparison**: Analyze the convergence speed and final route quality between first-order optimization methods.

## 🛠️ Project Structure
- `src/`: Core implementation
  - `environment.py`: Environment and drone parameter management.
  - `objective.py`: Time and Energy cost functions.
  - `constraints.py`: Mathematical formulation of constraints.
  - `derivatives.py`: Analytical gradients and numerical Hessian computation.
  - `gradient_descent.py`: GD optimizer implemented from scratch.
  - `heavy_ball.py`: Heavy-Ball optimizer implemented from scratch.
  - `feasibility.py`: Rigorous trajectory safety verification.
  - `visualization.py`: Plotting of routes and convergence metrics.
  - `experiments.py`: Main entry point to run the optimization study.
- `2D Dataset/`: Scenario generation and dataset storage.
- `results/`: Generated plots and logs (ignored by git).
- `optimization_project_specification.md`: Detailed mathematical and academic requirements.

## 📦 Installation
```bash
pip install -r requirements.txt
```

## 🏃 How to Run
Run the main experiment script to optimize a trajectory and generate comparison plots:
```bash
python src/experiments.py
```

The GD vs Heavy-Ball comparison can be run using:
```bash
python scripts/run_gd_hb_comparison.py
```

## 📊 Mathematical Approach
The constrained problem is handled using a quadratic penalty formulation:

$$ F_{\text{penalty}}(z) = F_{\text{objective}}(z) + \rho \sum_j \max(0,g_j(z))^2 $$

where \(z\) represents the waypoint coordinates and \(g_j(z)\) represents the optimization constraints.

The project also includes **Hessian/eigenvalue analysis, KKT verification, constraint-violation analysis, feasibility checking, and condition-number (\(\kappa\)) analysis.**

## 🔬 GD vs Heavy-Ball Evaluation

Both methods are evaluated under the same experimental settings and starting conditions across **50 scenarios.**

The comparison includes:

-Final objective and penalized objective
-Penalized gradient norm
-Constraint violations
-Exact trajectory feasibility
-Minimum obstacle clearance
-Iterations and runtime
-Hessian classification
-Valid condition numbers (\(\kappa\))
-Convergence and trajectory plots

The current experiments show that difficult scenarios can remain infeasible or fail to satisfy the numerical stopping criteria within the iteration limit. These cases are retained as part of the analysis rather than being removed.

## Delivery priority modes

The optimizer supports three delivery modes. Time and energy are normalized by reference scales before weights are applied, so `alpha` and `beta` represent priorities rather than raw units:

| Mode | Weights | Priority |
|---|---|---|
| Normal / Battery Saving | `alpha = 0.2`, `beta = 0.8` | Energy |
| Express / Urgent Delivery | `alpha = 0.9`, `beta = 0.1` | Time |
| Balanced | `alpha = 0.5`, `beta = 0.5` | Equal priority |

Results are saved in `results/metrics/delivery_modes.json`.

Because the current model uses fixed speed and energy proportional to distance, all three modes can produce similar routes. A richer energy model or speed as a decision variable is required for visibly different speed-energy trade-offs.
