import numpy as np

class Objective:
    """
    Calculates the cost associated with a drone trajectory.
    Main objective: Minimize Time and Energy.
    """
    def __init__(self, environment):
        self.env = environment
        self.drone = environment.get_drone_params()

        # Constants for calculation
        self.v_max = self.drone['max_speed_ms']

        # Power model coefficients: P = c0 + c1*v + c2*v^2
        self.c0 = self.drone['power_model']['c0']
        self.c1 = self.drone['power_model']['c1']
        self.c2 = self.drone['power_model']['c2']

        self.alpha = 1.0
        self.smoothness_weight = 0.0

    def total_distance(self, z):
        """
        z: decision vector [x1, y1, x2, y2, ..., xN, yN]
        """
        # Reconstruct the full trajectory: Start -> Waypoints -> Goal
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])

        # Calculate Euclidean distance between consecutive points
        diffs = np.diff(full_path, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)

    def time_cost(self, z):
        """
        Simplified time cost: Total Distance / Max Speed
        """
        dist = self.total_distance(z)
        return dist / self.v_max

    def energy_cost(self, z):
        """
        Simplified energy cost: (Power at max speed) * (Time at max speed)
        Power P = c0 + c1*v + c2*v^2
        Energy E = P * T = (c0 + c1*v_max + c2*v_max^2) * (dist / v_max)
        """
        dist = self.total_distance(z)
        time = dist / self.v_max
        power = self.c0 + self.c1 * self.v_max + self.c2 * (self.v_max**2)
        return power * time

    def combined_objective(self, z, w_time=1.0, w_energy=1.0, w_smooth=0.0):
        """
        Weighted sum of costs.
        """
        t_cost = self.time_cost(z)
        e_cost = self.energy_cost(z)
        s_cost = self.smoothness_cost(z) if w_smooth > 0 else 0.0

        return w_time * t_cost + w_energy * e_cost + w_smooth * s_cost

    def smoothness_gradient(self, z):
        """Gradient of the squared second-difference smoothness term."""
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])
        gradient = np.zeros_like(full_path)
        for i in range(1, len(full_path) - 1):
            difference = full_path[i + 1] - 2 * full_path[i] + full_path[i - 1]
            gradient[i - 1] += 2 * difference
            gradient[i] -= 4 * difference
            gradient[i + 1] += 2 * difference
        return gradient[1:-1].flatten()

    def smoothness_cost(self, z):
        """
        Penalty for sharp turns.
        S(z) = sum ||P_{i+1} - 2P_i + P_{i-1}||^2
        """
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])

        smoothness = 0.0
        for i in range(1, len(full_path) - 1):
            # Second-order difference (curvature approximation)
            diff = full_path[i+1] - 2*full_path[i] + full_path[i-1]
            smoothness += np.sum(diff**2)

        return smoothness
