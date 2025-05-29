import math
import numpy as np
from geometry_msgs.msg import Point

class DWAPlanner:
    class State:
        def __init__(self, x=0.0, y=0.0, yaw=0.0, velocity=0.0, yawrate=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.velocity = velocity
            self.yawrate = yawrate

    class Cost:
        def __init__(self, obs=0.0, to_goal=0.0, speed=0.0, path=0.0):
            self.obs = obs
            self.to_goal = to_goal
            self.speed = speed
            self.path = path
            self.total = 0.0

        def calc_total(self, gains):
            self.total = (
                self.obs * gains['obs'] +
                self.to_goal * gains['goal'] +
                self.speed * gains['speed'] +
                self.path * gains['path']
            )

    def __init__(self, params):
        self.p = params
        self.gains = {
            'obs': self.p.obs_cost_gain,
            'goal': self.p.to_goal_cost_gain,
            'speed': self.p.speed_cost_gain,
            'path': self.p.path_cost_gain
        }

    def calc_dynamic_window(self, current_twist):
        dw = [0.0] * 4
        dw[0] = max(current_twist.linear.x - self.p.max_deceleration * self.p.sim_period,
                    self.p.min_velocity)
        dw[1] = min(current_twist.linear.x + self.p.max_acceleration * self.p.sim_period,
                    self.p.target_velocity)
        dw[2] = max(current_twist.angular.z - self.p.max_d_yawrate * self.p.sim_period,
                    -self.p.max_yawrate)
        dw[3] = min(current_twist.angular.z + self.p.max_d_yawrate * self.p.sim_period,
                    self.p.max_yawrate)
        return dw

    def generate_trajectory(self, v, y):
        traj = []
        state = self.State()
        dt = self.p.predict_time / self.p.sim_time_samples
        for _ in range(self.p.sim_time_samples):
            state.yaw += y * dt
            state.x += v * math.cos(state.yaw) * dt
            state.y += v * math.sin(state.yaw) * dt
            state.velocity = v
            state.yawrate = y
            traj.append(self.State(state.x, state.y, state.yaw, v, y))
        return traj

    def normalize_costs(self, costs):
        mins = [float('inf')] * 4
        maxs = [0.0] * 4
        for c in costs:
            if c.obs < 1e6:
                mins[0] = min(mins[0], c.obs); maxs[0] = max(maxs[0], c.obs)
                mins[1] = min(mins[1], c.to_goal); maxs[1] = max(maxs[1], c.to_goal)
                mins[2] = min(mins[2], c.speed);  maxs[2] = max(maxs[2], c.speed)
                mins[3] = min(mins[3], c.path);   maxs[3] = max(maxs[3], c.path)
        for c in costs:
            if c.obs < 1e6:
                c.obs     = (c.obs - mins[0])    / (maxs[0] - mins[0] + 1e-6)
                c.to_goal = (c.to_goal - mins[1]) / (maxs[1] - mins[1] + 1e-6)
                c.speed   = (c.speed - mins[2])   / (maxs[2] - mins[2] + 1e-6)
                c.path    = (c.path - mins[3])    / (maxs[3] - mins[3] + 1e-6)

    def calc_to_goal_cost(self, traj, goal):
        last = traj[-1]
        dx = last.x - goal.x
        dy = last.y - goal.y
        return math.hypot(dx, dy)

    def calc_obs_cost(self, traj, obs_list):
        min_dist = self.p.obs_range
        for st in traj:
            for o in obs_list:
                if self.p.use_footprint:
                    # footprint-based collision check (omitted for brevity)
                    pass
                else:
                    dist = math.hypot(st.x - o.x, st.y - o.y)
                    d = dist - self.p.robot_radius - self.p.footprint_padding
                if d <= 0:
                    return 1e6
                min_dist = min(min_dist, d)
        return self.p.obs_range - min_dist

    def calc_path_cost(self, traj, path_poses):
        if not self.p.use_path_cost:
            return 0.0
        end = traj[-1]
        p1 = path_poses[0]; p2 = path_poses[-1]
        a = p2.y - p1.y; b = -(p2.x - p1.x)
        c = -a * p1.x - b * p1.y
        return abs(a * end.x + b * end.y + c) / math.hypot(a, b)

    def calc_speed_cost(self, traj):
        if not self.p.use_speed_cost:
            return 0.0
        dw = self.calc_dynamic_window(traj[0])
        return dw[1] - traj[0].velocity

    def evaluate_trajectory(self, traj, goal, obs_list, path_poses=None):
        cost = self.Cost()
        cost.obs     = self.calc_obs_cost(traj, obs_list)
        cost.to_goal = self.calc_to_goal_cost(traj, goal)
        cost.speed   = self.calc_speed_cost(traj)
        cost.path    = self.calc_path_cost(traj, path_poses)
        return cost

    def dwa_planning(self, current_twist, goal, obs_list, path_poses=None):
        dw = self.calc_dynamic_window(current_twist)
        costs, trajs = [], []
        for v in np.linspace(dw[0], dw[1], self.p.velocity_samples):
            for y in np.linspace(dw[2], dw[3], self.p.yawrate_samples):
                tr = self.generate_trajectory(v, y)
                c  = self.evaluate_trajectory(tr, goal, obs_list, path_poses)
                costs.append(c); trajs.append(tr)
        self.normalize_costs(costs)
        best, min_cost = trajs[0], float('inf')
        for tr, c in zip(trajs, costs):
            c.calc_total(self.gains)
            if c.total < min_cost:
                min_cost, best = c.total, tr
        return best
