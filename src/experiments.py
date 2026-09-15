import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from analysis import hessian_analysis
from constraints import Constraints
from derivatives import Derivatives
from environment import Environment2D
from feasibility import FeasibilityChecker
from gradient_descent import GradientDescentOptimizer
from heavy_ball import HeavyBallOptimizer
from metrics import solution_metrics
from objective import Objective
from objective import DELIVERY_MODES
from visualization import Visualizer

ROOT = Path(__file__).parent.parent
DATASET_PATH = ROOT / "data" / "drone_optimization_dataset.json"
RESULTS = ROOT / "results"


def initial_waypoints(environment, count, mode="straight", seed=0):
    t = np.linspace(0, 1, count + 2)
    path = np.array([environment.get_start() + fraction * (environment.get_goal() - environment.get_start()) for fraction in t])
    waypoints = path[1:-1].copy()
    direction = environment.get_goal() - environment.get_start()
    perpendicular = np.array([-direction[1], direction[0]])
    perpendicular /= max(np.linalg.norm(perpendicular), 1e-12)
    if mode == "offset":
        waypoints += 0.8 * np.sin(np.pi * t[1:-1])[:, None] * perpendicular
    elif mode == "perturbed":
        rng = np.random.default_rng(seed)
        waypoints += rng.normal(0.0, 0.25, size=waypoints.shape)
    return waypoints.flatten()


def build_problem(scenario, count):
    environment = Environment2D(scenario)
    objective = Objective(environment)
    constraints = Constraints(environment)
    constraints.set_time_calculator(objective.time_cost)
    derivatives = Derivatives(environment, objective, constraints)
    feasibility = FeasibilityChecker(environment)
    return environment, objective, constraints, derivatives, feasibility, initial_waypoints(environment, count)


def run_method(environment, objective, constraints, derivatives, feasibility, z0, method, learning_rate=0.01, momentum=0.9, rho=100.0, max_iter=1000, mode="balanced"):
    alpha, beta = objective.mode_weights(mode)
    if method == "Gradient Descent":
        optimizer = GradientDescentOptimizer(environment, objective, constraints, derivatives, learning_rate=learning_rate, rho=rho, max_iter=max_iter)
    else:
        optimizer = HeavyBallOptimizer(environment, objective, constraints, derivatives, learning_rate=learning_rate, momentum=momentum, rho=rho, max_iter=max_iter)
    solution = optimizer.fit(z0, w_time=alpha, w_energy=beta)
    metrics = solution_metrics(environment, objective, constraints, feasibility, optimizer, solution)
    metrics["delivery_mode"] = mode
    metrics["time_weight_alpha"] = alpha
    metrics["energy_weight_beta"] = beta
    return solution, optimizer, metrics


def run_pair(scenario, count, z0=None, max_iter=1000, rho=100.0, mode="balanced"):
    environment, objective, constraints, derivatives, feasibility, default_z0 = build_problem(scenario, count)
    z0 = default_z0 if z0 is None else z0
    initial = {
        "feasible": feasibility.verify_all(z0, objective.energy_cost, objective.time_cost)[0],
        "message": feasibility.verify_all(z0, objective.energy_cost, objective.time_cost)[1],
        "objective": float(objective.combined_objective(z0)),
    }
    results = {}
    solutions = {}
    for method in ("Gradient Descent", "Heavy-Ball"):
        solution, optimizer, metrics = run_method(environment, objective, constraints, derivatives, feasibility, z0, method, rho=rho, max_iter=max_iter, mode=mode)
        results[method] = metrics
        solutions[method] = (solution, optimizer)
    return environment, objective, constraints, derivatives, feasibility, initial, results, solutions


def save_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


def main():
    dataset = json.loads(DATASET_PATH.read_text(encoding="utf-8"))
    scenario = dataset[0]
    RESULTS.mkdir(exist_ok=True)
    environment, objective, constraints, derivatives, feasibility, initial, results, solutions = run_pair(scenario, 5)
    visualizer = Visualizer(environment)
    initial_z = initial_waypoints(environment, 5)
    visualizer.plot_trajectory(initial_z, "Initial Straight-Line Route", RESULTS / "trajectories" / "initial_path.png")
    visualizer.plot_trajectory(solutions["Gradient Descent"][0], "Gradient Descent Optimized Path", RESULTS / "trajectories" / "gd_path.png")
    visualizer.plot_trajectory(solutions["Heavy-Ball"][0], "Heavy-Ball Optimized Path", RESULTS / "trajectories" / "hb_path.png")
    visualizer.plot_diagnostics(solutions["Gradient Descent"][1], solutions["Heavy-Ball"][1], RESULTS / "convergence")

    initial_hessian = hessian_analysis(derivatives, initial_waypoints(environment, 5))
    final_hessian = {
        method: hessian_analysis(derivatives, solutions[method][0], rho=solutions[method][1].rho)
        for method in solutions
    }
    basic = {
        "scenario_id": scenario["id"],
        "waypoints": 5,
        "safety_margin_m": scenario.get("safety_parameters", {}).get("safety_margin", 0.0),
        "extra_clearance_m": scenario.get("safety_parameters", {}).get("extra_clearance", 0.0),
        "usable_battery_Wh": environment.usable_battery_Wh(),
        "max_time_s": environment.max_time_s,
        "initial": initial,
        "initial_hessian": initial_hessian,
        "final_hessian": final_hessian,
        "methods": results,
    }
    simple_scenario = dict(scenario)
    simple_scenario["obstacles"] = scenario["obstacles"][:1]
    _, _, _, _, _, simple_initial, simple_results, _ = run_pair(simple_scenario, 5, max_iter=3000, rho=1000.0)
    basic["simple_obstacle_case"] = {"initial": simple_initial, "methods": simple_results}
    basic["multiple_obstacle_case"] = results
    save_json(RESULTS / "metrics" / "scenario_0.json", basic)

    mode_comparison = {}
    for mode in DELIVERY_MODES:
        _, _, _, _, _, _, mode_results, _ = run_pair(scenario, 5, max_iter=1000, mode=mode)
        mode_comparison[mode] = mode_results
    save_json(RESULTS / "metrics" / "delivery_modes.json", mode_comparison)

    experiment_rows = []
    for learning_rate in (0.005, 0.01, 0.02):
        for method in ("Gradient Descent", "Heavy-Ball"):
            env, obj, cons, deriv, feas, _, _, _ = run_pair(scenario, 5, max_iter=500)
            z0 = initial_waypoints(env, 5)
            solution, optimizer, metrics = run_method(env, obj, cons, deriv, feas, z0, method, learning_rate=learning_rate, max_iter=500)
            experiment_rows.append({"experiment": "learning_rate", "value": learning_rate, "method": method, **metrics})
    for momentum in (0.7, 0.9, 0.95):
        env, obj, cons, deriv, feas, _, _, _ = run_pair(scenario, 5, max_iter=500)
        z0 = initial_waypoints(env, 5)
        solution, optimizer, metrics = run_method(env, obj, cons, deriv, feas, z0, "Heavy-Ball", momentum=momentum, max_iter=500)
        experiment_rows.append({"experiment": "momentum", "value": momentum, "method": "Heavy-Ball", **metrics})
    waypoint_rows = []
    for count in (5, 10, 20, 40):
        env, obj, cons, deriv, feas, _, pair_results, _ = run_pair(scenario, count, max_iter=3000, rho=1000.0)
        for method, metrics in pair_results.items():
            waypoint_rows.append({"experiment": "waypoint_count", "waypoints": count, "method": method, **metrics})
    visualizer.plot_waypoint_study(waypoint_rows, RESULTS / "convergence" / "objective_vs_waypoints.png")
    for mode in ("straight", "offset", "perturbed"):
        env, obj, cons, deriv, feas, _, _, _ = run_pair(scenario, 5, z0=initial_waypoints(Environment2D(scenario), 5, mode), max_iter=500)
        z0 = initial_waypoints(env, 5, mode)
        for method in ("Gradient Descent", "Heavy-Ball"):
            solution, optimizer, metrics = run_method(env, obj, cons, deriv, feas, z0, method, max_iter=500)
            experiment_rows.append({"experiment": "initialization", "value": mode, "method": method, **metrics})

    save_json(RESULTS / "metrics" / "experiment_study.json", {"sensitivity": experiment_rows, "waypoint_count": waypoint_rows})
    print(json.dumps({"initial": initial, "methods": results}, indent=2))
    print("Full experiment study saved to results/metrics/experiment_study.json")


if __name__ == "__main__":
    main()
