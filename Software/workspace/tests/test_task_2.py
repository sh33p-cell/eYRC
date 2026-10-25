"""Offline algorithm checks; ROS messaging and dynamics need simulator tests.

Run with: python -m unittest discover -s Software/workspace/tests -v
Requires NumPy 1.x and OpenCV. Only ROS interfaces are replaced by test doubles.
"""

import ast
import heapq
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from collections import deque

import cv2
import numpy as np


class Time:
    def __init__(self, seconds=0.0, nanoseconds=0):
        self.nanoseconds = round(seconds * 1e9) + nanoseconds

    @classmethod
    def from_msg(cls, msg):
        return cls(nanoseconds=msg.sec * 10**9 + msg.nanosec)

    def to_msg(self):
        sec, nanosec = divmod(self.nanoseconds, 10**9)
        return NS(sec=sec, nanosec=nanosec)

    def __add__(self, other):
        return Time(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other):
        return Time(nanoseconds=self.nanoseconds - other.nanoseconds)


def pose_message(x=0.0, y=0.0, yaw=-math.pi / 2):
    return NS(header=NS(stamp=Time().to_msg(), frame_id=""),
              pose=NS(position=NS(x=x, y=y, z=0.0),
                      orientation=NS(w=math.cos(yaw / 2), x=0.0,
                                     y=0.0, z=math.sin(yaw / 2))))


class Node:
    def __init__(self, name):
        self.name = name
        self.now = Time()
        self.messages = []
        self.logs = []

    def create_subscription(self, *args):
        return args

    def create_publisher(self, *args):
        return NS(publish=self.messages.append)

    def create_timer(self, *args):
        return args

    def get_clock(self):
        return NS(now=lambda: self.now)

    def get_logger(self):
        return NS(info=self.logs.append, warning=self.logs.append,
                  error=self.logs.append)


def load(task, filename):
    """Execute the actual source, replacing only unavailable ROS interfaces."""
    path = Path(__file__).resolve().parents[1] / "src" / task / task / filename
    tree = ast.parse(path.read_text())
    tree.body = [stmt for stmt in tree.body
                 if not isinstance(stmt, (ast.Import, ast.ImportFrom))]
    ns = dict(__name__="offline_test", math=math, heapq=heapq, deque=deque, np=np, cv2=cv2,
              Node=Node, Time=Time, Duration=Time, Odometry=object,
              Float64MultiArray=NS, PoseStamped=pose_message,
              Path=lambda: NS(header=NS(stamp=None, frame_id=""), poses=[]),
              QoSProfile=lambda **kw: NS(**kw),
              DurabilityPolicy=NS(TRANSIENT_LOCAL="transient_local"))
    exec(compile(tree, str(path), "exec"), ns)
    return NS(**ns)


class TaskTwoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a = load("task_2a", "path_follower.py")
        cls.b = load("task_2b", "path_follower.py")
        cls.p = load("task_2b", "path_planner.py")

    def test_kinematics_and_uniform_saturation(self):
        for f in (self.a, self.b):
            np.testing.assert_allclose(f.body_to_wheels(.15, 0, 0),
                                       [5.094267, -5.094267, 0], atol=1e-6)
            np.testing.assert_allclose(f.body_to_wheels(0, .15, 0),
                                       [2.941176, 2.941176, -5.882353], atol=1e-6)
            np.testing.assert_allclose(f.body_to_wheels(0, 0, -.5),
                                       [-1.257255] * 3, atol=1e-6)
            raw = f.body_velocity_to_wheel_speeds(4, 3, 2)
            np.testing.assert_allclose(f.body_to_wheels(4, 3, 2),
                                       raw * 30 / max(abs(raw)))

    def test_frames_and_wrap(self):
        for f in (self.a, self.b):
            np.testing.assert_allclose(f.to_body(0, -.15, -math.pi / 2),
                                       [.15, 0], atol=1e-12)
            self.assertAlmostEqual(f.wrap(math.radians(350)), math.radians(-10))
            self.assertAlmostEqual(f.wrap(-math.pi), math.pi)
            q = pose_message().pose.orientation
            self.assertAlmostEqual(f.yaw_from_quat(q.w, q.x, q.y, q.z), -math.pi / 2)

    def test_controller_limits_and_yaw_correction(self):
        for f in (self.a, self.b):
            ctl = f.GoToPoint()
            for _ in range(1000):
                wheels, _ = ctl.step((0, 0, -math.pi / 2), (10, 10), 0)
                vx, vy, wz = np.linalg.solve(f.IK_MATRIX, wheels)
                self.assertLessEqual(math.hypot(vx, vy), ctl.v_max + 1e-12)
                self.assertLessEqual(abs(wz), ctl.w_max + 1e-12)
                self.assertGreater(wz, 0)
            self.assertLessEqual(np.linalg.norm(ctl.integral_xy) * ctl.ki,
                                 .2 * ctl.v_max + 1e-12)

    def test_task_2a_ready_barrier_and_all_goals(self):
        ready, arrivals = set(), []
        nodes = [self.a.RobotNode(r, ready, lambda r, t: arrivals.append(r))
                 for r in self.a.ORDER]
        nodes[0].odom_cb(NS(pose=pose_message(*self.a.START['r1'])))
        nodes[0].tick()
        self.assertEqual(nodes[0].messages[-1].data, [0., 0., 0.])
        poses = {r: np.array([*self.a.START[r], -math.pi / 2])
                 for r in self.a.ORDER}
        for step in range(2000):
            for node in nodes:
                pose = poses[node.robot]
                node.odom_cb(NS(pose=pose_message(*pose)))
            for node in nodes:
                node.now = Time(seconds=step / self.a.CONTROL_HZ)
                node.tick()
                vx, vy, wz = np.linalg.solve(self.a.IK_MATRIX, node.messages[-1].data)
                pose = poses[node.robot]
                c, s = math.cos(pose[2]), math.sin(pose[2])
                pose += np.array([c * vx - s * vy, s * vx + c * vy, wz]) / self.a.CONTROL_HZ
            if all(node.arrived for node in nodes):
                break
        self.assertCountEqual(arrivals, self.a.ORDER)
        for node in nodes:
            self.assertLess(math.dist(poses[node.robot][:2], self.a.GOALS[node.robot]), .020)
            self.assertAlmostEqual(poses[node.robot][2], -math.pi / 2)
            node.tick()
            self.assertEqual(node.messages[-1].data, [0., 0., 0.])
        self.assertEqual(len(arrivals), 3)

    def test_task_2a_settling_must_be_consecutive(self):
        arrived = []
        node = self.a.RobotNode('r2', set(self.a.ORDER), lambda *args: arrived.append(args))
        goal = self.a.GOALS['r2']
        node.odom_cb(NS(pose=pose_message(*goal)))
        for _ in range(self.a.SETTLE_TICKS - 1):
            node.tick()
        self.assertFalse(node.arrived)
        node.odom_cb(NS(pose=pose_message(goal[0] + .1, goal[1])))
        node.tick()
        self.assertEqual(node.settled, 0)
        node.odom_cb(NS(pose=pose_message(-1., goal[1])))
        self.assertNotIn('r2', node.ready)
        self.assertEqual(node.messages[-1].data, [0., 0., 0.])
        node.odom_cb(NS(pose=pose_message(*goal)))
        for _ in range(self.a.SETTLE_TICKS):
            node.tick()
        self.assertEqual(len(arrived), 1)

    def test_timed_path_roundtrip_and_stops(self):
        plan = [(0., .4, .4), (1.3, .6, .4), (3.3, .6, .4),
                (5., .6, .6), (7., .6, .6), (9., .8, .6), (11., .8, .6)]
        msg = self.p.PathPlanner.to_path_msg(None, plan, Time(seconds=100.8))
        np.testing.assert_allclose(self.b.path_to_traj(msg), plan, atol=1e-9)
        self.assertEqual(msg.header.frame_id, 'odom')
        stops = self.b.stops_in(self.b.path_to_traj(msg))
        self.assertEqual(len(stops), 3)
        self.assertEqual(stops[-1][1], math.inf)
        self.assertEqual(self.b.position_at(plan, 2.), (.6, .4))

    def test_follower_waits_and_parks(self):
        arrived = []
        node = self.b.RobotNode('r2', lambda *args: arrived.append(args))
        node.on_odom(NS(pose=pose_message(.4, .4)))
        msg = self.p.PathPlanner.to_path_msg(None, [(0., .4, .4), (1., .6, .4)], Time(seconds=10))
        node.on_path(msg)
        node.now = Time(seconds=9)
        node.tick()
        self.assertEqual(node.messages[-1].data, [0., 0., 0.])
        node.on_odom(NS(pose=pose_message(.6, .4)))
        node.now = Time(seconds=12)
        node.tick()
        node.tick()
        self.assertTrue(node.arrived)
        self.assertEqual(len(arrived), 1)
        self.assertEqual(node.messages[-1].data, [0., 0., 0.])

    def test_astar_no_corner_cutting_and_no_route(self):
        blocked = np.zeros((5, 5), dtype=bool)
        blocked[1, 0] = blocked[0, 1] = True
        self.assertIsNone(self.p.astar(blocked, (0, 0), (4, 4)))
        blocked[0, 1] = False
        path = self.p.astar(blocked, (0, 0), (4, 4))
        self.assertEqual(path[1], (0, 1))

    def test_segments_cannot_cross_or_touch_obstacles(self):
        blocked = np.zeros((61, 61), dtype=bool)
        blocked[25, 25] = True
        self.assertFalse(self.p.segment_clear(blocked, (.5, 1.02), (1.5, 1.02)))
        self.assertFalse(self.p.segment_clear(blocked, (.5, 1.), (1.5, 1.)))
        self.assertTrue(self.p.segment_clear(blocked, (.5, .95), (1.5, .95)))

    def test_route_visits_exact_goals_and_waits(self):
        blocked = np.zeros((61, 61), dtype=bool)
        blocked[25:35, 25:35] = True
        plan = self.p.plan_route(blocked, (1.2192, 2.2616))
        self.assertIsNotNone(plan)
        stops = self.b.stops_in(plan)
        self.assertEqual([s[2:] for s in stops], self.p.GOALS)
        for a, b in zip(plan, plan[1:]):
            self.assertGreater(b[0], a[0])
            self.assertTrue(self.p.segment_clear(blocked, a[1:], b[1:]))
            if a[1:] != b[1:]:
                self.assertAlmostEqual(math.dist(a[1:], b[1:]) / (b[0] - a[0]), self.p.PLAN_SPEED)
        blocked[self.p.to_cell(self.p.GOALS[0])] = True
        self.assertIsNone(self.p.plan_route(blocked, (1.2192, 2.2616)))

    def test_cleanup_preserves_pillar_near_goal(self):
        p = self.p
        hsv = np.full((720, 1280, 3), (20, 80, 220), dtype=np.uint8)
        # A goal and a grey pillar 12 cm away: both lie in the old clearing disc.
        goal = (1., 1.)
        pixel = lambda xy: (round(p.ARENA_LEFT + xy[0] * p.ARENA_PIXELS / p.ARENA_SIZE),
                            round(p.ARENA_TOP + xy[1] * p.ARENA_PIXELS / p.ARENA_SIZE))
        cv2.circle(hsv, pixel(goal), 9, (90, 220, 220), -1)
        pillar = (1.12, 1.)
        cv2.circle(hsv, pixel(pillar), 12, (0, 0, 130), -1)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        grid = p.frame_to_occupancy(frame)
        cleaned = p.remove_drivable(frame, grid, [], [goal])
        cell = lambda xy: (int(xy[1] / p.CELL_SIZE), int(xy[0] / p.CELL_SIZE))
        self.assertFalse(cleaned[cell(goal)])
        self.assertTrue(cleaned[cell(pillar)])
        self.assertTrue(grid[cell(goal)], 'input grid must not be mutated')

    def test_cleanup_removes_robot_but_keeps_payload(self):
        p = self.p
        hsv = np.full((720, 1280, 3), (20, 80, 220), dtype=np.uint8)
        pixel = lambda xy: (round(p.ARENA_LEFT + xy[0] * p.ARENA_PIXELS / p.ARENA_SIZE),
                            round(p.ARENA_TOP + xy[1] * p.ARENA_PIXELS / p.ARENA_SIZE))
        robot, pad = (1.2192, 2.2616), (.6, .6)
        cv2.circle(hsv, pixel(robot), 18, (0, 0, 130), -1)
        cv2.circle(hsv, pixel(pad), 45, (90, 220, 220), -1)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cleaned = p.remove_drivable(frame, p.frame_to_occupancy(frame), [robot], [])
        cell = lambda xy: (int(xy[1] / p.CELL_SIZE), int(xy[0] / p.CELL_SIZE))
        self.assertFalse(cleaned[cell(robot)])
        self.assertTrue(cleaned[cell(pad)])
        self.assertFalse(cleaned[cell((.73, .6))])

    def test_planner_follower_integration_with_ideal_dynamics(self):
        p, f = self.p, self.b
        rows, cols = np.indices((p.GRID_CELLS, p.GRID_CELLS))
        x, y = (cols + .5) * p.CELL_SIZE, (rows + .5) * p.CELL_SIZE
        for center in [(1.2192, 1.3), (.8, 1.), (1.5, 1.2)]:
            with self.subTest(pillar=center):
                occupied = (x - center[0])**2 + (y - center[1])**2 <= .08**2
                plan = p.plan_route(p.planning_grid(occupied), (1.2192, 2.2616))
                self.assertIsNotNone(plan)
                arrivals = []
                node = f.RobotNode('r2', lambda *args: arrivals.append(args))
                node.on_path(p.PathPlanner.to_path_msg(None, plan, Time(seconds=2)))
                pose = np.array([1.2192, 2.2616, -math.pi / 2])
                clearance = math.inf
                for step in range(5000):
                    node.now = Time(seconds=step / f.CONTROL_HZ)
                    node.on_odom(NS(pose=pose_message(*pose)))
                    node.tick()
                    vx, vy, wz = np.linalg.solve(f.IK_MATRIX, node.messages[-1].data)
                    c, s = math.cos(pose[2]), math.sin(pose[2])
                    pose += np.array([c * vx - s * vy, s * vx + c * vy, wz]) / f.CONTROL_HZ
                    clearance = min(clearance, math.dist(pose[:2], center) - .08)
                    if node.arrived:
                        break
                self.assertTrue(node.arrived)
                self.assertEqual(node.reached, 3)
                self.assertEqual(len(arrivals), 1)
                self.assertGreaterEqual(clearance, .084)
                self.assertLess(math.dist(pose[:2], p.GOALS[-1]), .020)


if __name__ == '__main__':
    unittest.main()
