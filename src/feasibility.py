import numpy as np

class FeasibilityChecker:
    """
    Performs rigorous checks to verify if a trajectory is actually safe.
    Includes segment-based collision detection.
    """
    def __init__(self, environment):
        self.env = environment
        self.safety_margin = environment.safety_params.get('safety_margin', 0.0)
        self.extra_clearance = environment.safety_params.get('extra_clearance', 0.0)

    def check_waypoints(self, z):
        """Checks if all waypoints clear each buffered no-fly zone."""
        if not np.all(np.isfinite(z)):
            return False, "Trajectory contains non-finite coordinates"
        waypoints = z.reshape(-1, 2)
        if np.any(waypoints[:, 0] < self.env.x_min) or np.any(waypoints[:, 0] > self.env.x_max):
            return False, "Waypoint violates x workspace bounds"
        if np.any(waypoints[:, 1] < self.env.y_min) or np.any(waypoints[:, 1] > self.env.y_max):
            return False, "Waypoint violates y workspace bounds"
        obstacles = self.env.get_obstacles()
        for p in waypoints:
            for obs in obstacles:
                dist = np.linalg.norm(p - np.array(obs['center']))
                required = self.env.effective_obstacle_radius(obs)
                if dist < required:
                    return False, f"Waypoint {p} is within {required:.2f} m of obstacle {obs['center']}"
        return True, "All waypoints safe"

    def check_segments(self, z):
        """
        Checks if any line segment between waypoints intersects a buffered NFZ.
        Uses projection of circle center onto line segment.
        """
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])
        obstacles = self.env.get_obstacles()

        for i in range(len(full_path) - 1):
            p1 = full_path[i]
            p2 = full_path[i+1]
            line_vec = p2 - p1
            line_len_sq = np.dot(line_vec, line_vec)

            if line_len_sq == 0: continue

            for obs in obstacles:
                center = np.array(obs['center'])
                # Projection of center onto the segment
                t = np.dot(center - p1, line_vec) / line_len_sq
                t = np.clip(t, 0, 1)
                closest_point = p1 + t * line_vec
                dist = np.linalg.norm(center - closest_point)

                required = self.env.effective_obstacle_radius(obs)
                if dist < required:
                    return False, f"Segment {i} violates {required:.2f} m clearance from obstacle {obs['center']}"

        return True, "All segments safe"

    def check_battery(self, z, energy_calculator, battery_cap):
        """Checks if total energy is within battery capacity."""
        energy = energy_calculator(z)
        if energy > battery_cap:
            return False, f"Energy {energy} exceeds capacity {battery_cap}"
        return True, "Battery sufficient"

    def check_time(self, z, time_calculator):
        """Checks the maximum delivery-time constraint."""
        flight_time = time_calculator(z)
        if flight_time > self.env.max_time_s:
            return False, f"Time {flight_time:.3f} exceeds limit {self.env.max_time_s:.3f} s"
        return True, "Time limit satisfied"

    def minimum_clearance(self, z):
        """Return the smallest distance from the route to a buffered obstacle boundary."""
        waypoints = z.reshape(-1, 2)
        full_path = np.vstack([self.env.get_start(), waypoints, self.env.get_goal()])
        minimum = np.inf
        for p1, p2 in zip(full_path[:-1], full_path[1:]):
            line_vec = p2 - p1
            line_len_sq = np.dot(line_vec, line_vec)
            for obstacle in self.env.get_obstacles():
                center = np.asarray(obstacle['center'])
                if line_len_sq > 0:
                    fraction = np.dot(center - p1, line_vec) / line_len_sq
                    fraction = np.clip(fraction, 0.0, 1.0)
                    closest = p1 + fraction * line_vec
                else:
                    closest = p1
                clearance = np.linalg.norm(center - closest) - self.env.effective_obstacle_radius(obstacle)
                minimum = min(minimum, clearance)
        return float(minimum)

    def verify_all(self, z, energy_calculator, time_calculator=None):
        """
        Runs all checks and returns a report.
        """
        battery_cap = self.env.usable_battery_Wh()
        endpoint_messages = self.env.validate_fixed_points()
        if endpoint_messages:
            return False, "; ".join(endpoint_messages)

        w_safe, w_msg = self.check_waypoints(z)
        if not w_safe: return False, w_msg

        s_safe, s_msg = self.check_segments(z)
        if not s_safe: return False, s_msg

        b_safe, b_msg = self.check_battery(z, energy_calculator, battery_cap)
        if not b_safe: return False, b_msg

        if time_calculator is not None:
            t_safe, t_msg = self.check_time(z, time_calculator)
            if not t_safe: return False, t_msg

        return True, "Trajectory is fully feasible"
