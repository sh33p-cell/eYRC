"""Task 2C planning and ideal-motion checks; no ROS or MuJoCo required."""

import math
import unittest

import cv2
import numpy as np

from test_task_2 import NS, Time, load, pose_message


class TaskTwoCTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = load('task_2c', 'path_planner.py')
        cls.f = load('task_2c', 'path_follower.py')
        cls.starts = dict(zip(cls.p.ROBOTS, cls.p.START_CIRCLES))
        rows, cols = np.indices((cls.p.GRID_CELLS, cls.p.GRID_CELLS))
        x, y = (cols + .5) * cls.p.CELL_SIZE, (rows + .5) * cls.p.CELL_SIZE
        # A wall with one 50 cm opening, before robot-radius inflation.
        cls.occupied = (abs(y - 1.3) < .06) & (abs(x - 1.2192) > .25)
        cls.blocked = cls.p.planning_grid(cls.occupied)
        cls.plans = cls.p.plan_all(cls.blocked, cls.starts)

    def test_gap_routes_exact_endpoints_and_valid_speeds(self):
        self.assertIsNotNone(self.plans)
        waits = 0
        for robot, plan in self.plans.items():
            self.assertEqual(plan[0], (0., *self.starts[robot]))
            self.assertEqual(plan[-1][1:], self.p.GOALS[robot])
            self.assertEqual(len(self.f.stops_in(plan)), 1)
            for a, b in zip(plan, plan[1:]):
                self.assertGreater(b[0], a[0])
                self.assertTrue(self.p.segment_clear(self.blocked, a[1:], b[1:]))
                speed = math.dist(a[1:], b[1:]) / (b[0] - a[0])
                self.assertLessEqual(speed, self.p.PLAN_SPEED + 1e-10)
                if a[1:] == b[1:]:
                    waits += 1
                    self.assertLess(b[0] - a[0], self.f.MIN_STOP)
        self.assertGreater(waits, 0, 'the single gap should require traffic waits')

    def test_continuous_separation_including_parked_robots(self):
        self.assertIsNotNone(self.plans)
        for index, r1 in enumerate(self.p.ROBOTS):
            for r2 in self.p.ROBOTS[index + 1:]:
                a, b = self.plans[r1], self.plans[r2]
                times = sorted({p[0] for p in a + b})
                for t0, t1 in zip(times, times[1:]):
                    rel0 = np.subtract(self.f.position_at(a, t0), self.f.position_at(b, t0))
                    rel1 = np.subtract(self.f.position_at(a, t1), self.f.position_at(b, t1))
                    delta = rel1 - rel0
                    denominator = float(delta @ delta)
                    u = np.clip(-float(rel0 @ delta) / denominator, 0, 1) if denominator else 0
                    distance = np.linalg.norm(rel0 + u * delta)
                    self.assertGreaterEqual(distance + 1e-10, self.p.ROBOT_CLEARANCE)

    def test_edge_swaps_and_crossings_are_rejected(self):
        self.assertFalse(self.p.motion_clear((0., 0.), (1., 0.), (1., 0.), (0., 0.)))
        self.assertFalse(self.p.motion_clear((0., .5), (1., .5), (.5, 0.), (.5, 1.)))
        self.assertTrue(self.p.motion_clear((0., 0.), (1., 0.), (0., .3), (1., .3)))

    def test_parked_goal_remains_reserved(self):
        blocked = np.zeros((61, 61), dtype=bool)
        prior = [(.5, .5), (.54, .5)]
        self.assertIsNone(self.p.plan_one(blocked, (1., 1.), (.6, .5), [prior], []))

    def test_arrival_cannot_park_in_a_future_crossing(self):
        blocked = np.zeros((61, 61), dtype=bool)
        goal = (.8, .8)
        # The earlier robot passes through this goal after a long initial wait.
        prior = [(1.2, .8)] * 20 + [(1.2 - i * .04, .8) for i in range(1, 21)]
        route = self.p.plan_one(blocked, (.8, 1.2), goal, [prior], [])
        self.assertIsNotNone(route)
        self.assertGreater(len(route) - 1, 30)
        for step in range(max(len(route), len(prior)) - 1):
            self.assertTrue(self.p.motion_clear(
                route[min(step, len(route) - 1)], route[min(step + 1, len(route) - 1)],
                prior[min(step, len(prior) - 1)], prior[min(step + 1, len(prior) - 1)]))

    def test_camera_cleanup_leaves_gap_starts_and_goals_free(self):
        p = self.p
        hsv = np.full((720, 1280, 3), (20, 80, 220), dtype=np.uint8)
        def pixel(xy):
            return (round(p.ARENA_LEFT + xy[0] * p.ARENA_PIXELS / p.ARENA_SIZE),
                    round(p.ARENA_TOP + xy[1] * p.ARENA_PIXELS / p.ARENA_SIZE))
        for lo, hi in [((0., 1.24), (.9692, 1.36)),
                       ((1.4692, 1.24), (p.ARENA_SIZE, 1.36))]:
            cv2.rectangle(hsv, pixel(lo), pixel(hi), (0, 0, 130), -1)
        for point in self.starts.values():
            cv2.circle(hsv, pixel(point), 26, (90, 220, 220), -1)
            cv2.circle(hsv, pixel(point), 18, (0, 0, 130), -1)
        for point in p.GOALS.values():
            cv2.circle(hsv, pixel(point), 9, (60, 220, 220), -1)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        grid = p.frame_to_occupancy(frame)
        cleaned = p.remove_drivable(frame, grid, list(self.starts.values()), list(p.GOALS.values()))
        blocked = p.planning_grid(cleaned)
        for point in [*self.starts.values(), *p.GOALS.values(), (1.2192, 1.3)]:
            self.assertFalse(blocked[p.to_cell(point)], str(point))
        self.assertTrue(blocked[p.to_cell((.6, 1.3))])
        self.assertTrue(blocked[p.to_cell((1.8, 1.3))])

    def test_unplanned_robot_starts_remain_protected(self):
        blocked = np.zeros((61, 61), dtype=bool)
        waiting = (1., 1.)
        route = self.p.plan_one(blocked, (.5, 1.), (1.5, 1.), [], [waiting])
        self.assertIsNotNone(route)
        for a, b in zip(route, route[1:]):
            # Independent stationary point-to-segment distance.
            delta = np.subtract(b, a)
            denom = float(delta @ delta)
            u = np.clip(np.dot(np.subtract(waiting, a), delta) / denom, 0, 1) if denom else 0
            self.assertGreaterEqual(np.linalg.norm(np.subtract(a, waiting) + u * delta), .26 - 1e-10)

    def test_no_route_and_invalid_endpoints(self):
        blocked = self.blocked.copy()
        blocked[:, 30:35] = True
        self.assertIsNone(self.p.plan_all(blocked, self.starts))
        starts = dict(self.starts, r2=self.starts['r1'])
        self.assertIsNone(self.p.plan_all(self.blocked, starts))
        blocked = self.blocked.copy()
        blocked[self.p.to_cell(self.p.GOALS['r3'])] = True
        self.assertIsNone(self.p.plan_all(blocked, self.starts))

    def test_shared_stamp_and_roundtrip(self):
        start = Time(seconds=123.8)
        for robot, plan in self.plans.items():
            msg = self.p.PathPlanner.to_path_msg(None, plan, start)
            self.assertEqual(msg.header.stamp, start.to_msg())
            np.testing.assert_allclose(self.f.path_to_traj(msg), plan, atol=1e-9)

    def test_all_followers_through_gap_with_ideal_dynamics(self):
        arrived = []
        nodes = {r: self.f.RobotNode(r, lambda robot, t: arrived.append(robot))
                 for r in self.p.ROBOTS}
        poses = {r: np.array([*self.starts[r], -math.pi / 2]) for r in nodes}
        for robot, node in nodes.items():
            node.on_path(self.p.PathPlanner.to_path_msg(None, self.plans[robot], Time(seconds=2)))
        min_separation, min_clearance = math.inf, math.inf
        for step in range(3000):
            for robot, node in nodes.items():
                node.now = Time(seconds=step / self.f.CONTROL_HZ)
                node.on_odom(NS(pose=pose_message(*poses[robot])))
                node.tick()
                vx, vy, wz = np.linalg.solve(self.f.IK_MATRIX, node.messages[-1].data)
                pose = poses[robot]
                c, s = math.cos(pose[2]), math.sin(pose[2])
                pose += np.array([c * vx - s * vy, s * vx + c * vy, wz]) / self.f.CONTROL_HZ
                # Distance to each rectangle making up the physical wall.
                for lo, hi in [((0., 1.24), (.9692, 1.36)),
                               ((1.4692, 1.24), (self.p.ARENA_SIZE, 1.36))]:
                    d = np.maximum(np.maximum(np.subtract(lo, pose[:2]), np.subtract(pose[:2], hi)), 0)
                    min_clearance = min(min_clearance, np.linalg.norm(d))
            for i, a in enumerate(self.p.ROBOTS):
                for b in self.p.ROBOTS[i + 1:]:
                    min_separation = min(min_separation, math.dist(poses[a][:2], poses[b][:2]))
            if all(n.arrived for n in nodes.values()):
                break
        self.assertCountEqual(arrived, self.p.ROBOTS)
        self.assertGreaterEqual(min_separation, .168)
        self.assertGreaterEqual(min_clearance, .084)
        for robot, node in nodes.items():
            self.assertLess(math.dist(poses[robot][:2], self.p.GOALS[robot]), .020)
            self.assertEqual(node.held, 0., 'traffic waits must not shift a follower clock')
            self.assertEqual(node.reached, 1)
            self.assertTrue(any('reached its goal' in log for log in node.logs))


if __name__ == '__main__':
    unittest.main()
