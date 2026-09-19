import numpy as np
class Environment2D:
    """
    Manages the 2D drone delivery environment, including map bounds,
    start/goal locations, and no-fly zones (obstacles).
    """
    def __init__(self, scenario_data):
        self.start = np.asarray(scenario_data['start'], dtype=float)
        self.goal = np.asarray(scenario_data['goal'], dtype=float)
        self.obstacles = scenario_data['obstacles']
        self.drone_params = scenario_data['drone_params']
        self.safety_params = scenario_data.get('safety_parameters', {})
        self.max_time_s = float(scenario_data.get('max_time_s', 3.0))

        # Map bounds (assumed 0-10 based on dataset generator)
        self.x_min, self.x_max = 0.0, 10.0
        self.y_min, self.y_max = 0.0, 10.0

    def effective_obstacle_radius(self, obstacle):
        """Return the no-fly radius including the configured safety buffer."""
        margin = self.safety_params.get('safety_margin', 0.0)
        extra_clearance = self.safety_params.get('extra_clearance', 0.0)
        return obstacle['radius'] + margin + extra_clearance

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def get_obstacles(self):
        return self.obstacles

    def get_drone_params(self):
        return self.drone_params

    def usable_battery_Wh(self):
        """Return energy available above the configured reserve state of charge."""
        battery = self.drone_params['battery_capacity_Wh']
        initial_soc = self.drone_params.get('initial_soc', 1.0)
        reserve_soc = self.drone_params.get('reserve_soc', 0.0)
        return battery * max(initial_soc - reserve_soc, 0.0)

    def validate_fixed_points(self):
        """Validate fixed endpoints before any waypoint optimization starts."""
        messages = []
        for name, point in (("start", self.start), ("goal", self.goal)):
            if not np.all(np.isfinite(point)):
                messages.append(f"{name} contains non-finite coordinates")
            if not (self.x_min <= point[0] <= self.x_max and self.y_min <= point[1] <= self.y_max):
                messages.append(f"{name} is outside the workspace")
            for obstacle in self.obstacles:
                distance = np.linalg.norm(point - np.asarray(obstacle['center']))
                if distance < self.effective_obstacle_radius(obstacle):
                    messages.append(f"{name} violates buffered obstacle clearance")
        return messages
