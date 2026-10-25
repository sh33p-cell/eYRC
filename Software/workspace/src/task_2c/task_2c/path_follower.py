#!/usr/bin/env python3
# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2026-27)
*        =============================================
*
*  This script is to implement Task 2C of Hola The Explorer (HE) Theme (eYRC 2026-27).
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         path_follower.py
# Functions:        No extra helper functions.
# Global variables: None added.


############################ WHAT YOU HAVE TO DO ##############################
#
#  Drive r1, r2 and r3 at once, each along its timed path on /rN/plan.
#
#   1. Paste in your Task 2A code      (the FROM TASK 2A section)
#   2. Copy in path_to_traj()          (from Task 2B)
#
#  RobotNode is given, one per robot. All paths share one start time, so by
#  keeping to the path's clock each robot waits for the gap exactly when the
#  planner said so -- the nodes never talk to each other.
#
#  Run:  ros2 launch hb_description task2c.launch.py
#        ros2 run task_2c path_planner
#        ros2 run task_2c path_follower
#
###############################################################################

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

###############################################################


# ----------------------------------------------------- follower (given)
ROBOTS = ("r1", "r2", "r3")   # one node per robot
CONTROL_HZ = 50.0
GOAL_TOLERANCE = 0.020        # m, "on the goal"
LEAD = 0.12                   # s, aim this far ahead on the path
MIN_STOP = 0.5                # s, a pause this long in a path is a goal
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ----------------------------------------------------- robot (same as Task 1B)
WHEEL_RADIUS_M = 0.0255       # m
CHASSIS_RADIUS_M = 0.06412    # m, chassis centre to each wheel's axle
WHEEL_ANGLES_RAD = np.radians([30.0, 150.0, 270.0])
IK_MATRIX = np.array([
    [math.cos(angle), math.sin(angle), CHASSIS_RADIUS_M]
    for angle in WHEEL_ANGLES_RAD
]) / WHEEL_RADIUS_M
_CTRL_LIMIT = 30.0            # rad/s, the wheels' ctrlrange in the robot's MJCF:
                              # faster commands are clamped by the simulation


######################### FROM TASK 2A ########################
## Paste your Task 2A functions and GoToPoint here, unchanged.##
###############################################################

##############  ADD YOUR CODE HERE  ##############

def body_velocity_to_wheel_speeds(vx, vy, w):
    """Body twist (vx, vy, w) -> wheel speeds [left, right, back], rad/s."""
    ##############  ADD YOUR CODE HERE  ##############
    return IK_MATRIX @ np.array([vx, vy, w], dtype=float)


def body_to_wheels(vx, vy, wz):
    """Like body_velocity_to_wheel_speeds(), but kept inside +/-_CTRL_LIMIT.
    Returns a list."""
    ##############  ADD YOUR CODE HERE  ##############
    wheel_speeds = body_velocity_to_wheel_speeds(vx, vy, wz)
    peak = float(np.max(np.abs(wheel_speeds)))
    if peak > _CTRL_LIMIT:
        wheel_speeds *= _CTRL_LIMIT / peak
    return wheel_speeds.tolist()


def yaw_from_quat(w, x, y, z):
    """Quaternion -> yaw, radians."""
    ##############  ADD YOUR CODE HERE  ##############
    return math.atan2(2.0 * (w * z + x * y),
                      w * w + x * x - y * y - z * z)


def wrap(a):
    """Angle -> the same angle in (-pi, pi]. 350 degrees becomes -10."""
    ##############  ADD YOUR CODE HERE  ##############
    angle = (a + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if angle == -math.pi else angle


def to_body(vx_a, vy_a, yaw):
    """Arena-frame velocity -> body-frame velocity (vx, vy).
    Check: at yaw = -pi/2 (facing up), arena (0, -v) must give body (v, 0)."""
    ##############  ADD YOUR CODE HERE  ##############
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx_a + s * vy_a, -s * vx_a + c * vy_a


class GoToPoint:
    """PID controller: drive one robot straight at a point, holding a fixed yaw.
    One instance per robot -- the PID terms keep state between steps."""

    def __init__(self, v_max=0.25, w_max=0.6, kp=2.0, ki=0.02, kd=0.05,
                 kyaw=2.0, kiyaw=0.02, kdyaw=0.05,
                 tol=GOAL_TOLERANCE, dt=1.0 / CONTROL_HZ):
        self.v_max, self.w_max = v_max, w_max            # m/s, rad/s caps
        self.kp, self.ki, self.kd = kp, ki, kd           # position PID
        self.kyaw, self.kiyaw, self.kdyaw = kyaw, kiyaw, kdyaw   # yaw PID
        self.tol = tol                                   # m, stop inside this
        self.dt = dt                                     # s, 1 / CONTROL_HZ
        ##############  ADD YOUR CODE HERE  ##############
        if dt <= 0.0 or v_max <= 0.0 or w_max <= 0.0 or tol <= 0.0:
            raise ValueError("dt, speed caps and tolerance must be positive")
        self.integral_xy = np.zeros(2, dtype=float)
        self.previous_xy = None
        self.integral_yaw = 0.0
        self.previous_yaw = None
        ##################################################

    def step(self, pose, target, hold_yaw):
        """pose (x, y, yaw), target (x, y), hold_yaw -> (wheels, distance)."""
        ##############  ADD YOUR CODE HERE  ##############
        x, y, yaw = pose
        error = np.array([target[0] - x, target[1] - y], dtype=float)
        distance = float(np.linalg.norm(error))
        derivative = (np.zeros(2) if self.previous_xy is None else
                      (error - self.previous_xy) / self.dt)
        self.previous_xy = error.copy()

        if distance <= self.tol:
            velocity = np.zeros(2)
            self.integral_xy[:] = 0.0
        else:
            candidate = self.integral_xy + error * self.dt
            # Limit the integral contribution as a vector, preserving direction.
            if self.ki > 0.0:
                integral_cap = 0.2 * self.v_max / self.ki
                magnitude = float(np.linalg.norm(candidate))
                if magnitude > integral_cap:
                    candidate *= integral_cap / magnitude
            else:
                candidate[:] = 0.0
            velocity = (self.kp * error + self.ki * candidate +
                        self.kd * derivative)
            speed = float(np.linalg.norm(velocity))
            if speed > self.v_max:
                velocity *= self.v_max / speed
                # Freeze integration while the translation output saturates.
            else:
                self.integral_xy = candidate

        yaw_error = wrap(hold_yaw - yaw)
        yaw_derivative = (0.0 if self.previous_yaw is None else
                          wrap(yaw_error - self.previous_yaw) / self.dt)
        self.previous_yaw = yaw_error
        yaw_candidate = self.integral_yaw + yaw_error * self.dt
        if self.kiyaw > 0.0:
            integral_cap = 0.2 * self.w_max / self.kiyaw
            yaw_candidate = max(-integral_cap, min(integral_cap, yaw_candidate))
        else:
            yaw_candidate = 0.0
        wz_raw = (self.kyaw * yaw_error + self.kiyaw * yaw_candidate +
                  self.kdyaw * yaw_derivative)
        wz = max(-self.w_max, min(self.w_max, wz_raw))
        if abs(wz_raw) <= self.w_max:
            self.integral_yaw = yaw_candidate

        vx, vy = to_body(float(velocity[0]), float(velocity[1]), yaw)
        return body_to_wheels(vx, vy, wz), distance

##################################################


######################### PATH ################################

def path_to_traj(msg):
    """Path -> [(t, x, y), ...], t relative to the path's header stamp."""
    start = Time.from_msg(msg.header.stamp)
    traj = []
    for pose in msg.poses:
        t = (Time.from_msg(pose.header.stamp) - start).nanoseconds * 1e-9
        p = pose.pose.position
        traj.append((t, float(p.x), float(p.y)))
    return traj
    ##################################################


def stops_in(traj):
    """Given. Goals on a timed path: [(arrive, leave, x, y), ...], in s.
    A goal is a pause of at least MIN_STOP, or the end; the last is never left."""
    stops = [(t0, t1, x1, y1) for (t0, x0, y0), (t1, x1, y1) in zip(traj, traj[1:])
             if (x0, y0) == (x1, y1) and t1 - t0 >= MIN_STOP]
    if not stops or stops[-1][2:] != traj[-1][1:]:
        stops.append((traj[-1][0], traj[-1][0], *traj[-1][1:]))
    arrive, _, x, y = stops[-1]
    stops[-1] = (arrive, math.inf, x, y)
    return stops


def position_at(traj, t):
    """Given. Where the path says the robot should be at time t."""
    if t <= traj[0][0]:
        return traj[0][1:]
    for (t0, x0, y0), (t1, x1, y1) in zip(traj, traj[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return x0 + (x1 - x0) * u, y0 + (y1 - y0) * u
    return traj[-1][1:]


######################### NODE (given) ########################

class RobotNode(Node):
    """Drives one robot along its /rN/plan. Nodes: task_2c_r1, _r2, _r3."""

    def __init__(self, robot, on_arrival):
        super().__init__(f"task_2c_{robot}")
        self.robot, self.on_arrival = robot, on_arrival
        self.pose = self.hold_yaw = self.traj = self.start = None
        self.stops, self.reached, self.arrived = [], 0, False
        self.held = 0.0                         # s the path clock has been held on a goal
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 10)
        self.create_subscription(Path, f"/{robot}/plan", self.on_path, LATCHED)
        self.pub = self.create_publisher(Float64MultiArray, f"/{robot}/wheel_commands", 10)
        self.ctl = GoToPoint(v_max=0.30, tol=0.012, dt=1.0 / CONTROL_HZ)
        self.create_timer(1.0 / CONTROL_HZ, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.w, q.x, q.y, q.z))
        if self.hold_yaw is None:
            self.hold_yaw = self.pose[2]        # keep the starting heading throughout

    def on_path(self, msg):
        if self.traj is None and msg.poses:
            self.traj = path_to_traj(msg)
            self.stops = stops_in(self.traj)
            self.start = Time.from_msg(msg.header.stamp)
            self.get_logger().info("got my path: %d poses, arrive at t=%.1f s"
                                   % (len(msg.poses), self.traj[-1][0]))

    def send(self, wheels):
        self.pub.publish(Float64MultiArray(data=[float(w) for w in wheels]))

    def tick(self):
        if self.traj is None or self.pose is None or self.arrived:
            self.send((0.0, 0.0, 0.0))
            return
        now = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if now < 0:                             # not the start time yet
            self.send((0.0, 0.0, 0.0))
            return
        t = now - self.held                     # where on the path the robot should be
        arrive, leave, *goal = self.stops[self.reached]
        to_go = math.dist(self.pose[:2], goal)
        which = "its goal" if len(self.stops) == 1 else "goal %d of %d" % (self.reached + 1, len(self.stops))
        if t >= arrive:                         # the path is waiting on this goal
            if to_go < GOAL_TOLERANCE:
                self.get_logger().info("%s reached %s (%.0f mm out) at t=%.1fs"
                                       % (self.robot, which, to_go * 1000, now))
                self.reached += 1
                if self.reached == len(self.stops):
                    self.arrived = True
                    self.send((0.0, 0.0, 0.0))
                    self.on_arrival(self.robot, now)
                    return
            elif t > leave:                     # the stop is over but the robot is not on
                self.held += t - leave          # the goal yet: hold the path clock until it is
                t = leave
        # on a stop, aim at the goal itself: LEAD would aim down the next leg
        target = goal if t >= arrive else position_at(self.traj, t + LEAD)
        wheels, _ = self.ctl.step(self.pose, target, self.hold_yaw)
        self.send(wheels)


def main():
    rclpy.init()
    arrived = {}

    def on_arrival(robot, t):
        arrived[robot] = t
        if len(arrived) == len(ROBOTS):
            nodes[0].get_logger().info("task 2C complete in %.1fs" % max(arrived.values()))

    nodes = [RobotNode(r, on_arrival) for r in ROBOTS]
    executor = MultiThreadedExecutor(num_threads=len(ROBOTS))
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            try:
                n.send((0.0, 0.0, 0.0))          # leave nothing driving
            except Exception:
                pass
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
