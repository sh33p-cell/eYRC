"""Timing/clearance regressions on the captured Task 2C arena.

These exercise real planner/controller code with ideal or delayed first-order
motion. They complement, rather than replace, the competition simulator.
"""

from collections import deque
import math
from pathlib import Path
import unittest

import cv2
import numpy as np

from test_task_2 import NS, Time, load, pose_message


def measure_followers(planner, follower, plans, occupied, lag=0., delay_ticks=0):
    """Return timing and clearance metrics under a deterministic motion model."""
    starts = dict(zip(planner.ROBOTS, planner.START_CIRCLES))
    arrivals = {}
    nodes = {r: follower.RobotNode(r, lambda r, t: arrivals.update({r: t}))
             for r in planner.ROBOTS}
    poses = {r: np.array([*starts[r], -math.pi / 2]) for r in nodes}
    velocities = {r: np.zeros(3) for r in nodes}
    history = {r: deque([poses[r].copy()] * (delay_ticks + 1),
                         maxlen=delay_ticks + 1) for r in nodes}
    obstacle_distance = cv2.distanceTransform(
        (~occupied).astype('uint8'), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * planner.CELL_SIZE
    # Account conservatively for sampling clearance at a cell centre.
    cell_uncertainty = planner.CELL_SIZE / math.sqrt(2)
    dt = 1 / follower.CONTROL_HZ
    response = 1. if lag == 0 else 1 - math.exp(-dt / lag)
    inverse_ik = np.linalg.inv(follower.IK_MATRIX)
    for r, node in nodes.items():
        node.on_path(planner.PathPlanner.to_path_msg(None, plans[r], Time()))
    first_motion = None
    min_pair, min_obstacle = math.inf, math.inf
    peak_wheel = max_tracking_error = 0.
    for step in range(round(20 / dt)):
        now = step * dt
        for r, node in nodes.items():
            node.now = Time(seconds=now)
            node.on_odom(NS(pose=pose_message(*history[r][0])))
            node.tick()
            wheels = node.messages[-1].data
            peak_wheel = max(peak_wheel, *map(abs, wheels))
            command = inverse_ik @ wheels
            velocities[r] += response * (command - velocities[r])
            vx, vy, wz = velocities[r]
            c, s = math.cos(poses[r][2]), math.sin(poses[r][2])
            poses[r] += dt * np.array([c * vx - s * vy, s * vx + c * vy, wz])
            history[r].append(poses[r].copy())
            reference = follower.position_at(plans[r], now + dt)
            max_tracking_error = max(max_tracking_error, math.dist(poses[r][:2], reference))
            col, row = np.clip((poses[r][:2] / planner.CELL_SIZE).astype(int),
                               0, planner.GRID_CELLS - 1)
            min_obstacle = min(min_obstacle, float(obstacle_distance[row, col]) - cell_uncertainty)
        if first_motion is None and any(math.dist(poses[r][:2], starts[r]) > .003 for r in nodes):
            first_motion = now
        for i, a in enumerate(planner.ROBOTS):
            for b in planner.ROBOTS[i + 1:]:
                min_pair = min(min_pair, math.dist(poses[a][:2], poses[b][:2]))
        if len(arrivals) == len(nodes):
            break
    return dict(complete=len(arrivals) == len(nodes),
                elapsed=now - first_motion if first_motion is not None else math.inf,
                min_pair=min_pair, min_obstacle=min_obstacle, peak_wheel=peak_wheel,
                max_tracking_error=max_tracking_error,
                goal_errors={r: math.dist(poses[r][:2], planner.GOALS[r]) for r in nodes},
                held={r: n.held for r, n in nodes.items()})


class TaskTwoCPerformanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = load('task_2c', 'path_planner.py')
        cls.f = load('task_2c', 'path_follower.py')
        frame = cv2.imread(str(Path(__file__).parent / 'fixtures/task2c_start.jpg'))
        starts = dict(zip(cls.p.ROBOTS, cls.p.START_CIRCLES))
        cls.occupied = cls.p.remove_drivable(
            frame, cls.p.frame_to_occupancy(frame), list(starts.values()), list(cls.p.GOALS.values()))
        cls.blocked = cls.p.planning_grid(cls.occupied)
        cls.plans = cls.p.plan_all(cls.blocked, starts)

    def test_captured_arena_schedule_has_time_budget(self):
        self.assertIsNotNone(self.plans)
        # Leave at least one second for imperfect tracking and final settling.
        self.assertLessEqual(max(p[-1][0] for p in self.plans.values()), 9.)
        for path in self.plans.values():
            for a, b in zip(path, path[1:]):
                self.assertTrue(self.p.segment_clear(self.blocked, a[1:], b[1:]))
        # Check exact swept separation at every breakpoint of both plans.
        for index, a in enumerate(self.p.ROBOTS):
            for b in self.p.ROBOTS[index + 1:]:
                times = sorted({p[0] for p in self.plans[a] + self.plans[b]})
                for t0, t1 in zip(times, times[1:]):
                    self.assertTrue(self.p.motion_clear(
                        self.f.position_at(self.plans[a], t0), self.f.position_at(self.plans[a], t1),
                        self.f.position_at(self.plans[b], t0), self.f.position_at(self.plans[b], t1),
                        self.p.ROBOT_CLEARANCE - 1e-9))

    def test_finish_within_ten_seconds_with_tracking_margin(self):
        for lag, delay in [(0., 0), (.06, 0), (.12, 2)]:
            with self.subTest(motor_lag=lag, odometry_delay_ticks=delay):
                result = measure_followers(self.p, self.f, self.plans, self.occupied, lag, delay)
                self.assertTrue(result['complete'], result)
                self.assertLessEqual(result['elapsed'], 10., result)
                self.assertGreaterEqual(result['min_pair'], .168, result)
                self.assertGreaterEqual(result['min_obstacle'], .084, result)
                self.assertLessEqual(result['peak_wheel'], 30., result)
                self.assertLessEqual(result['max_tracking_error'], .07, result)
                self.assertLess(max(result['goal_errors'].values()), .03, result)
                self.assertEqual(set(result['held'].values()), {0.}, result)

    def test_arrival_requires_consecutive_settling_ticks(self):
        arrivals = []
        robot = self.p.ROBOTS[0]
        goal = self.p.GOALS[robot]
        node = self.f.RobotNode(robot, lambda *args: arrivals.append(args))
        node.on_path(self.p.PathPlanner.to_path_msg(
            None, [(0., *self.p.START_CIRCLES[0]), (1., *goal)], Time()))
        node.now = Time(seconds=2.)
        node.on_odom(NS(pose=pose_message(*goal)))
        for _ in range(self.f.SETTLE_TICKS - 1):
            node.tick()
        self.assertFalse(node.arrived)
        node.on_odom(NS(pose=pose_message(goal[0] + .04, goal[1])))
        node.tick()
        self.assertEqual(node.goal_settled, 0)
        node.on_odom(NS(pose=pose_message(*goal)))
        for _ in range(self.f.SETTLE_TICKS):
            node.tick()
        self.assertTrue(node.arrived)
        node.tick()
        self.assertEqual(len(arrivals), 1)
        self.assertEqual(node.messages[-1].data, [0., 0., 0.])


if __name__ == '__main__':
    unittest.main()
