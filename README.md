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

## 📊 Mathematical Approach
The project uses a **Penalty Method** to transform a constrained optimization problem into an unconstrained one:
$$F_{penalty}(z) = F_{objective}(z) + \rho \sum \max(0, g_j(z))^2$$
Where $g_j(z)$ represents the constraints (NFZ, Battery, Workspace).
