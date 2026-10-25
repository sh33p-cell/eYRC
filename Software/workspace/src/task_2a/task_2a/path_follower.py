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
*  This script is to implement Task 2A of Hola The Explorer (HE) Theme (eYRC 2026-27).
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
#  Drive r1, r2 and r3 AT THE SAME TIME, each in a straight line to its goal
#  in GOALS, holding its starting heading, and park it there.
#
#   1. Paste in your Task 1B / 1C code   (body_velocity_to_wheel_speeds, ...)
#   2. Write wrap(), to_body() and the GoToPoint PID controller
#   3. Complete RobotNode                (main() runs one per robot)
#
#  Frame (/rN/odom): origin at the arena's TOP-LEFT corner, x right, y DOWN,
#  metres. Yaw 0 along +x, growing clockwise; the robots start at -pi/2.
#
#  Run:  ros2 launch hb_description task2a.launch.py
#        ros2 run task_2a path_follower
#
###############################################################################

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

###############################################################


# ----------------------------------------------------- arena (given, do not edit)
ARENA_SIZE = 2.4384           # m, the floor is square
ROBOT_RADIUS = 0.084          # m

START = {"r1": (0.8969, 2.2616),
         "r2": (1.2192, 2.2616),
         "r3": (1.5415, 2.2616)}
GOALS = {"r1": (0.6692, 0.8692),
         "r2": (1.2192, 0.6692),
         "r3": (1.7692, 0.8692)}
ORDER = ("r1", "r2", "r3")    # one node per robot

# ----------------------------------------------------- yours to set
CONTROL_HZ = 50.0              # how often tick() runs
GOAL_TOLERANCE = 0.020          # m, how close counts as "at the goal"
SETTLE_TICKS = 10              # ticks in a row inside GOAL_TOLERANCE = arrived

# ----------------------------------------------------- robot (same as Task 1B)
WHEEL_RADIUS_M = 0.0255       # m
CHASSIS_RADIUS_M = 0.06412    # m, chassis centre to each wheel's axle
WHEEL_ANGLES_RAD = np.radians([30.0, 150.0, 270.0])  # [left, right, back]
IK_MATRIX = np.array([
    [math.cos(angle), math.sin(angle), CHASSIS_RADIUS_M]
    for angle in WHEEL_ANGLES_RAD
]) / WHEEL_RADIUS_M
_CTRL_LIMIT = 30.0            # rad/s, the wheels' ctrlrange in the robot's MJCF:
                              # faster commands are clamped by the simulation


##################### FROM TASK 1B / 1C #######################

def body_velocity_to_wheel_speeds(vx, vy, w):
    """Body twist (vx, vy, w) -> wheel speeds [left, right, back], rad/s."""
    ##############  ADD YOUR CODE HERE  ##############
    return IK_MATRIX @ np.array([vx, vy, w], dtype=float)
    ##################################################


def body_to_wheels(vx, vy, wz):
    """Like body_velocity_to_wheel_speeds(), but kept inside +/-_CTRL_LIMIT.
    Returns a list."""
    ##############  ADD YOUR CODE HERE  ##############
    wheel_speeds = body_velocity_to_wheel_speeds(vx, vy, wz)
    peak = float(np.max(np.abs(wheel_speeds)))
    if peak > _CTRL_LIMIT:
        wheel_speeds *= _CTRL_LIMIT / peak
    return wheel_speeds.tolist()
    ##################################################


def yaw_from_quat(w, x, y, z):
    """Quaternion -> yaw, radians."""
    ##############  ADD YOUR CODE HERE  ##############
    return math.atan2(2.0 * (w * z + x * y),
                      w * w + x * x - y * y - z * z)
    ##################################################


######################### NEW IN 2A ###########################

def wrap(a):
    """Angle -> the same angle in (-pi, pi]. 350 degrees becomes -10."""
    ##############  ADD YOUR CODE HERE  ##############
    angle = (a + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if angle == -math.pi else angle
    ##################################################


def to_body(vx_a, vy_a, yaw):
    """Arena-frame velocity -> body-frame velocity (vx, vy).
    Check: at yaw = -pi/2 (facing up), arena (0, -v) must give body (v, 0)."""
    ##############  ADD YOUR CODE HERE  ##############
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx_a + s * vy_a, -s * vx_a + c * vy_a
    ##################################################


def frame_check(x, y, size=ARENA_SIZE):
    """Given. A complaint if (x, y) is not in the /odom frame, else None."""
    if not (0.0 <= x <= size and 0.0 <= y <= size):
        return ("odom pose (%.3f, %.3f) is outside 0..%.4f: this code works "
                "in the top-left-origin arena frame, not MuJoCo's "
                "centre-origin one" % (x, y, size))
    return None


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


######################### ONE NODE PER ROBOT ##################

class RobotNode(Node):
    """Drives one robot to GOALS[robot]. Nodes: task_2a_r1, _r2, _r3.
    No robot moves until all three are in `ready`, so they start together."""

    def __init__(self, robot, ready, on_arrival):
        super().__init__(f"task_2a_{robot}")
        self.robot = robot
        self.ready = ready            # shared by the three nodes
        self.on_arrival = on_arrival  # call once: on_arrival(robot, seconds)

        self.pose = None              # (x, y, yaw) from /<robot>/odom
        self.hold_yaw = None          # yaw on the first odom message
        self.settled = 0
        self.arrived = False
        self.t_start = None
        ##############  ADD YOUR CODE HERE  ##############
        self.create_subscription(Odometry, f"/{robot}/odom", self.odom_cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, f"/{robot}/wheel_commands", 10)
        self.ctl = GoToPoint(tol=0.012, dt=1.0 / CONTROL_HZ)
        self.create_timer(1.0 / CONTROL_HZ, self.tick)
        ##################################################

    def odom_cb(self, msg):
        ##############  ADD YOUR CODE HERE  ##############
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        complaint = frame_check(p.x, p.y)
        if complaint is not None:
            self.get_logger().error(complaint)
            self.ready.discard(self.robot)
            self.settled = 0
            self.send((0.0, 0.0, 0.0))
            return
        self.pose = (p.x, p.y, yaw_from_quat(q.w, q.x, q.y, q.z))
        if self.hold_yaw is None:
            self.hold_yaw = self.pose[2]
        self.ready.add(self.robot)
        ##################################################

    def send(self, wheels):
        self.pub.publish(Float64MultiArray(data=[float(w) for w in wheels]))

    def tick(self):
        if len(self.ready) < len(ORDER):
            self.send((0.0, 0.0, 0.0))
            return                                   # wait for every robot's odom
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        ##############  ADD YOUR CODE HERE  ##############
        if self.arrived:
            self.send((0.0, 0.0, 0.0))
            return
        wheels, distance = self.ctl.step(
            self.pose, GOALS[self.robot], self.hold_yaw)
        self.settled = self.settled + 1 if distance < GOAL_TOLERANCE else 0
        if self.settled >= SETTLE_TICKS:
            self.arrived = True
            self.send((0.0, 0.0, 0.0))
            elapsed = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
            self.get_logger().info("%s reached its goal (%.0f mm out)"
                                   % (self.robot, distance * 1000))
            self.on_arrival(self.robot, elapsed)
        else:
            self.send(wheels)
        ##################################################


def main():
    """Given. One RobotNode per robot; logs "task 2A complete" at the end."""
    rclpy.init()
    ready = set()
    arrived = {}

    def on_arrival(robot, t):
        arrived[robot] = t
        if len(arrived) == len(ORDER):
            nodes[0].get_logger().info("task 2A complete in %.1fs" % max(arrived.values()))

    nodes = [RobotNode(r, ready, on_arrival) for r in ORDER]
    executor = MultiThreadedExecutor(num_threads=len(ORDER))
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
            except Exception:                    # no publisher yet, or Ctrl+C took the context down
                pass
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
