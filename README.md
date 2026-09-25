# 🚁 3D Autonomous Drone Optimization

### Constrained 3D Trajectory Optimization using Gradient Descent & Heavy-Ball Momentum

## 📌 Overview

This project focuses on **3D drone trajectory optimization** to find safe and efficient flight paths while satisfying real-world constraints.

The system is evaluated across **50 different drone scenarios** and compares **Gradient Descent** and **Heavy-Ball Momentum** for optimization and convergence.

## 🎯 Key Features

- 🚁 **3D Trajectory Optimization**
- 🚫 **No-Fly Zone (NFZ) Avoidance**
- 🛡️ **Safety Margin & Constraint Handling**
- ⚡ **Energy & Flight-Time Optimization**
- 📐 **KKT & Convexity Analysis**
- 📉 **Gradient Descent vs Heavy-Ball**
- 📊 **50 Scenario Evaluation**
- 🌐 **3D Trajectory Visualization**

## 🧠 Optimization Methods

### Gradient Descent

$$
x_{k+1}=x_k-\alpha\nabla f(x_k)
$$

### Heavy-Ball Momentum

$$
x_{k+1}=x_k-\alpha\nabla f(x_k)+\beta(x_k-x_{k-1})
$$

The two methods are compared based on **convergence behavior, objective value, iterations, and trajectory feasibility**.

## 📐 Constraints

The optimization considers constraints such as:

- No-Fly Zones
- Safety clearance
- Start and goal positions
- Workspace boundaries
- Drone motion limitations
- Energy / battery requirements

## 📊 Evaluation

The optimization framework is tested on **50 generated scenarios** to analyze:

- Final objective value
- Convergence
- Optimization iterations
- Energy and flight time
- Constraint satisfaction
- Optimized 3D trajectory

## 🛠️ Tech Stack

- **Python**
- **NumPy**
- **Matplotlib**
- **Optimization Algorithms**
- **3D Visualization**

## ▶️ Run

```bash
pip install -r requirements.txt
python src/experiments.py
```

## 🗂️ Project Structure

```bash
3D Drone Optimization/
│
├── src/
│   ├── analysis.py
│   ├── base_optimizer.py
│   ├── experiments.py
│   ├── gradient_descent.py
│   ├── heavy_ball.py
│   └── kkt_analysis.py
│
├── scripts/
│   ├── run_gd_hd_comparison.py
│   └── run_kkt_analysis.py
│
├── data/
│   └── drone_optimization_dataset.json
│
├── 3D_Visualization.py
│
├── requirements.txt
│
└── README.md
```

## 🚀 Future Scope

Potential extensions include:

* Reinforcement Learning for adaptive trajectory planning
* Dynamic obstacles
* More realistic aerodynamic energy models
* Variable drone speed as an optimization variable
* Real-time replanning
* AirSim-based simulation
* Multi-drone trajectory coordination
