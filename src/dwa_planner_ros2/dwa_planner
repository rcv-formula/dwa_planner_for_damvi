import math
import numpy as np

class DWAPlanner:
    def __init__(self, params):
        for k, v in params.items():
            setattr(self, k, v)
        self.target_velocity = min(self.target_velocity, self.max_velocity)

    def calc_dynamic_window(self, current_twist):
        min_v = max(current_twist.linear.x - self.max_deceleration * self.sim_period, self.min_velocity)
        max_v = min(current_twist.linear.x + self.max_acceleration * self.sim_period, self.target_velocity)
        min_y = max(current_twist.angular.z - self.max_d_yawrate * self.sim_period, -self.max_yawrate)
        max_y = min(current_twist.angular.z + self.max_d_yawrate * self.sim_period, self.max_yawrate)
        return min_v, max_v, min_y, max_y

    def generate_trajectory(self, v, y):
        traj = []
        state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'velocity': v, 'yawrate': y}
        dt = self.predict_time / self.sim_time_samples
        for _ in range(self.sim_time_samples):
            state['yaw'] += state['yawrate'] * dt
            state['x'] += state['velocity'] * math.cos(state['yaw']) * dt
            state['y'] += state['velocity'] * math.sin(state['yaw']) * dt
            traj.append(state.copy())
        return traj

    def calc_to_goal_cost(self, traj, goal):
        last = traj[-1]
        dx = last['x'] - goal.x
        dy = last['y'] - goal.y
        return math.hypot(dx, dy)

    def calc_obs_cost(self, traj, obs_list):
        min_dist = self.obs_range
        for s in traj:
            for p in obs_list:
                d = math.hypot(s['x'] - p.x, s['y'] - p.y) - self.robot_radius - self.footprint_padding
                if d <= 0:
                    return float('inf')
                min_dist = min(min_dist, d)
        return self.obs_range - min_dist

    def calc_speed_cost(self, traj):
        return self.target_velocity - traj[0]['velocity']

    def evaluate_trajectory(self, traj, goal, obs_list):
        cost = 0.0
        cost += self.to_goal_cost_gain * self.calc_to_goal_cost(traj, goal)
        cost += self.obs_cost_gain * self.calc_obs_cost(traj, obs_list)
        cost += self.speed_cost_gain * self.calc_speed_cost(traj)
        return cost

    def dwa_planning(self, current_twist, goal, obs_list):
        dw = self.calc_dynamic_window(current_twist)
        best, min_cost = None, float('inf')
        for v in np.linspace(dw[0], dw[1], self.velocity_samples):
            for y in np.linspace(dw[2], dw[3], self.yawrate_samples):
                traj = self.generate_trajectory(v, y)
                cost = self.evaluate_trajectory(traj, goal, obs_list)
                if cost < min_cost:
                    best, min_cost = traj, cost
        return best
