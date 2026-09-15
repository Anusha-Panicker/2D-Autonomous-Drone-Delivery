import numpy as np

class Constraints:
    """
    Defines the constraints for the drone trajectory.
    Returns g(z) <= 0.
    """
    def __init__(self, environment):
        self.env = environment
        self.drone = environment.get_drone_params()
        self.safety_params = environment.safety_params
        self.segment_samples = 5
        self._time_calculator = None

    def battery_constraint(self, z, energy_calculator):
        """
        g_B(z) = Energy(z) - Battery_Capacity <= 0
        """
        energy = energy_calculator(z)
        battery_cap = self.env.usable_battery_Wh()
        # Conversion might be needed if energy is in Joules vs Wh.
        # For now, we assume consistent units or handled in the energy_calculator.
        return energy - battery_cap

    def time_constraint(self, z, time_calculator):
        """g_T(z) = T(z) - T_max <= 0."""
        return time_calculator(z) - self.env.max_time_s

    def nfz_constraint(self, z):
        """
        Returns buffered no-fly constraints for every waypoint.
        g_NFZ(z) = R_safe^2 - ||P_i - Center||^2 <= 0
        """
        waypoints = z.reshape(-1, 2)
        obstacles = self.env.get_obstacles()
        constraints = []

        # We check each waypoint against each obstacle
        for p in waypoints:
            for obs in obstacles:
                center = np.array(obs['center'])
                radius = self.env.effective_obstacle_radius(obs)
                dist_sq = np.sum((p - center)**2)
                constraints.append(radius**2 - dist_sq)

        return np.array(constraints)

    def segment_nfz_constraint(self, z):
        """Check interior points of each segment against buffered no-fly zones."""
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])
        constraints = []
        for start, end in zip(full_path[:-1], full_path[1:]):
            for fraction in np.linspace(0.0, 1.0, self.segment_samples + 2)[1:-1]:
                point = start + fraction * (end - start)
                for obstacle in self.env.get_obstacles():
                    center = np.asarray(obstacle['center'])
                    radius = self.env.effective_obstacle_radius(obstacle)
                    constraints.append(radius**2 - np.sum((point - center) ** 2))
        return np.asarray(constraints)

    def workspace_constraint(self, z):
        """
        Ensures waypoints are within [0, 10].
        g_x_min = x_min - x_i <= 0
        g_x_max = x_i - x_max <= 0
        """
        waypoints = z.reshape(-1, 2)
        constraints = []
        for p in waypoints:
            constraints.append(self.env.x_min - p[0])
            constraints.append(p[0] - self.env.x_max)
            constraints.append(self.env.y_min - p[1])
            constraints.append(p[1] - self.env.y_max)

        return np.array(constraints)

    def all_constraints(self, z, energy_calculator, time_calculator=None):
        """
        Combines all constraints into a single array.
        """
        battery = np.array([self.battery_constraint(z, energy_calculator)])
        calculator = time_calculator or self._time_calculator
        if calculator is None:
            raise ValueError("A time calculator is required for the time constraint")
        time = np.array([self.time_constraint(z, calculator)])
        nfz = self.nfz_constraint(z)
        segment_nfz = self.segment_nfz_constraint(z)
        workspace = self.workspace_constraint(z)

        return np.concatenate([battery, time, nfz, segment_nfz, workspace])

    def set_time_calculator(self, time_calculator):
        self._time_calculator = time_calculator
