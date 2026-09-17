import numpy as np
import time

class GradientDescentOptimizer:
    """
    Implements Gradient Descent from scratch.
    Update: z_{k+1} = z_k - eta * grad(F_penalty)
    """
    def __init__(self, env, objective, constraints, derivatives,
                 learning_rate=0.01, max_iter=1000, tol=1e-5, rho=100.0,
                 max_step=0.25):
        self.env = env
        self.obj = objective
        self.cons = constraints
        self.deriv = derivatives
        self.eta = learning_rate
        self.max_iter = max_iter
        self.tol = tol
        self.rho = rho
        self.max_step = max_step

        # State
        self.history = []
        self.grad_norm_history = []
        self.feasibility_history = []
        self.constraint_violation_history = []
        self.runtime = 0
        self.stopping_reason = None

    def fit(self, z0, w_time=1.0, w_energy=1.0, w_smooth=0.0):
        z = np.array(z0, dtype=float)
        self.w_time = w_time
        self.w_energy = w_energy
        self.w_smooth = w_smooth
        self.history = []
        self.grad_norm_history = []
        self.feasibility_history = []
        self.constraint_violation_history = []
        self.stopping_reason = "maximum_iterations"

        start_time = time.time()

        for k in range(self.max_iter):
            # 1. Calculate gradient
            g = self.deriv.gradient(z, w_time, w_energy, self.rho, w_smooth)
            g_norm = np.linalg.norm(g)

            # 2. Store stats
            self.history.append(self.deriv.penalized_objective(z, w_time, w_energy, self.rho, w_smooth))
            self.grad_norm_history.append(g_norm)
            constraints = self.cons.all_constraints(z, self.obj.energy_cost, self.obj.time_cost)
            self.constraint_violation_history.append(float(max(0.0, np.max(constraints))))
            self.feasibility_history.append(bool(self.constraint_violation_history[-1] <= 1e-6))

            # 3. Stopping criteria
            if g_norm < self.tol:
                self.stopping_reason = "gradient_norm_tolerance"
                break
            if k > 0 and abs(self.history[-1] - self.history[-2]) < self.tol:
                self.stopping_reason = "objective_change_tolerance"
                break

            # 4. Update
            step = self.eta * g
            step_norm = np.linalg.norm(step)
            if not np.isfinite(step_norm):
                self.stopping_reason = "nonfinite_step"
                break
            if step_norm > self.max_step:
                step *= self.max_step / step_norm
            z = z - step
            z[0::2] = np.clip(z[0::2], self.env.x_min, self.env.x_max)
            z[1::2] = np.clip(z[1::2], self.env.y_min, self.env.y_max)

        self.runtime = time.time() - start_time
        return z

    def predict(self):
        return self.history[-1] if self.history else None
