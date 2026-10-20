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
# Functions:        [ Add every extra helper function you write to this list ]
# Global variables: [ Add every extra global variable you declare to this list ]


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
GOAL_TOLERANCE = 0.003          # m, how close counts as "at the goal"
SETTLE_TICKS = 3              # ticks in a row inside GOAL_TOLERANCE = arrived

# ----------------------------------------------------- robot (same as Task 1B)
WHEEL_RADIUS_M = 0.0255       # m
CHASSIS_RADIUS_M = 0.06412    # m, chassis centre to each wheel's axle
WHEEL_ANGLES_RAD = np.radians([30.0, 150.0, 270.0])   # TODO: from Task 1B, [left, right, back]
IK_MATRIX = np.zeros((3, 3))                     # TODO: from Task 1B
_CTRL_LIMIT = 30.0            # rad/s, the wheels' ctrlrange in the robot's MJCF:
                              # faster commands are clamped by the simulation


##################### FROM TASK 1B / 1C #######################

def body_velocity_to_wheel_speeds(vx, vy, w):
    """Body twist (vx, vy, w) -> wheel speeds [left, right, back], rad/s."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1B
    angles = WHEEL_ANGLES_RAD
    wheel_speeds = (vx * np.cos(angles) + vy * np.sin(angles) + CHASSIS_RADIUS_M * w) / WHEEL_RADIUS_M
    return wheel_speeds
    ##################################################


def body_to_wheels(vx, vy, wz):
    """Like body_velocity_to_wheel_speeds(), but kept inside +/-_CTRL_LIMIT.
    Returns a list."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1C. Scale all three wheels down together, do not clip each
    # one: clipping changes the direction the robot drives in.
    wheel_speeds = np.array(body_velocity_to_wheel_speeds(vx, vy, wz))
    max_speed = np.max(np.abs(wheel_speeds))

    #scaling
    if max_speed > _CTRL_LIMIT:
        wheel_speeds *= _CTRL_LIMIT / max_speed
    
    return list(wheel_speeds)
    
    ##################################################


def yaw_from_quat(w, x, y, z):
    """Quaternion -> yaw, radians."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1C
    return math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    
    ##################################################


######################### NEW IN 2A ###########################

def wrap(a):
    """Angle -> the same angle in (-pi, pi]. 350 degrees becomes -10."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    
    if a <= -math.pi:
       return math.pi
    
    return a
    ##################################################


def to_body(vx_a, vy_a, yaw):
    """Arena-frame velocity -> body-frame velocity (vx, vy).
    Check: at yaw = -pi/2 (facing up), arena (0, -v) must give body (v, 0)."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO
    body_x = math.cos(yaw) * vx_a + math.sin(yaw) * vy_a
    body_y = -math.sin(yaw) * vx_a + math.cos(yaw) * vy_a
    
    return body_x, body_y
    ##################################################


def frame_check(x, y, size=ARENA_SIZE):
    """Given. A complaint if (x, y) is not in the /odom frame, else None."""
    if not (0.0 <= x <= size and 0.0 <= y <= size):
        return ("odom pose (%.3f, %.3f) is outside 0..%.4f: this code works "
                "in the top-left-origin arena frame, not MuJoCo's "
                "centre-origin one" % (x, y, size))
    return None


class GoToPoint:
    """PID controller: drive one robot straight at a point, holding a fixed yaw."""

    def __init__(self, v_max=0.0, w_max=0.0, kp=0.0, ki=0.0, kd=0.0,
                 kyaw=0.0, kiyaw=0.0, kdyaw=0.0, tol=0.0, dt=0.0):
        self.v_max, self.w_max = v_max, w_max
        self.kp, self.ki, self.kd = kp, ki, kd
        self.kyaw, self.kiyaw, self.kdyaw = kyaw, kiyaw, kdyaw
        self.tol = tol
        self.dt = dt

        self.integral_ex = 0.0
        self.integral_ey = 0.0
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        
    
        self.integral_yaw = 0.0
        self.prev_yaw_error = 0.0

    def step(self, pose, target, hold_yaw):
        """Return wheel speeds and distance to the target."""   

        x, y, yaw = pose

        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance <= self.tol:
            vx_world = 0.0
            vy_world = 0.0
            self.integral_ex *= 0.95
            self.integral_ey *= 0.95
        else:
            self.integral_ex += dx * self.dt
            self.integral_ey += dy * self.dt

            self.integral_ex = max(-1.0, min(1.0, self.integral_ex))
            self.integral_ey = max(-1.0, min(1.0, self.integral_ey))

            dex = (dx - self.prev_ex) / self.dt
            dey = (dy - self.prev_ey) / self.dt

            vx_world = self.kp * dx + self.ki * self.integral_ex + self.kd * dex
            vy_world = self.kp * dy + self.ki * self.integral_ey + self.kd * dey

            self.prev_ex = dx
            self.prev_ey = dy

            speed = math.hypot(vx_world, vy_world)
            if speed > self.v_max and speed > 0.0:
                scale = self.v_max / speed
                vx_world *= scale
                vy_world *= scale

        # Yaw control calculation
        yaw_error = wrap(hold_yaw - yaw)
        self.integral_yaw += yaw_error * self.dt
        dyaw = wrap(yaw_error - self.prev_yaw_error) / self.dt

        self.integral_yaw = max(-1.0, min(1.0, self.integral_yaw))
        wz = self.kyaw * yaw_error + self.kiyaw * self.integral_yaw + self.kdyaw * dyaw

        self.prev_yaw_error = yaw_error
        wz = max(-self.w_max, min(self.w_max, wz))

        vx_body, vy_body = to_body(vx_world, vy_world, yaw)
        wheels = body_to_wheels(vx_body, vy_body, wz)

        return wheels,distance
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
        self.ctl = GoToPoint(
                v_max=0.15,
                w_max=0.15,
                kp=15.0,
                ki=0.25,
                kd=1.0,
                kyaw=2.0,
                kiyaw=0.0,
                kdyaw=0.0,
                tol=GOAL_TOLERANCE,
                dt=1.0 / CONTROL_HZ
        )
        ##############  ADD YOUR CODE HERE  ##############
        # TODO:
        #   - subscribe to /<robot>/odom (self.odom_cb); publisher for
        #     /<robot>/wheel_commands in self.pub
        #   - self.ctl = a GoToPoint for this robot
        #   - a timer calling self.tick at CONTROL_HZ
        self.sub = self.create_subscription(
            Odometry,
            f"/{robot}/odom",
            self.odom_cb,
            10
        )
        self.pub = self.create_publisher(
            Float64MultiArray,
            f"/{robot}/wheel_commands",
            10
        )
        
        self.timer = self.create_timer(
            1.0 / CONTROL_HZ,
            self.tick
        )
        ##################################################

    def odom_cb(self, msg):
        ##############  ADD YOUR CODE HERE  ##############
        # TODO: self.pose = (x, y, yaw). First message only: set
        # self.hold_yaw, log frame_check(x, y) if it complains, and add
        # self.robot to self.ready.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.w, q.x, q.y, q.z)
    
        warning = frame_check(x, y)
        if warning is not None:
            self.get_logger().warning(warning)
            return
        
        self.pose = (x, y, yaw)

        if self.hold_yaw is None:
            self.hold_yaw = yaw

            self.get_logger().info(
            f"{self.robot}: initial pose "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )

        self.ready.add(self.robot)

        ##################################################

    def send(self, wheels):
        self.pub.publish(Float64MultiArray(data=[float(w) for w in wheels]))

    def tick(self):
        if len(self.ready) < len(ORDER):
            return                                   # wait for every robot's odom
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        ##############  ADD YOUR CODE HERE  ##############
        # TODO:
        #   - arrived: send zeros and return
        #   - else: wheels from self.ctl.step(self.pose, GOALS[self.robot],
        #     self.hold_yaw), and send them
        #   - after SETTLE_TICKS in a row within GOAL_TOLERANCE: set
        #     self.arrived, log "<robot> reached its goal (... mm out)", and
        #     call self.on_arrival(self.robot, seconds since self.t_start)
        # Record the start time when control begins
        if self.arrived:
            self.send([0.0, 0.0, 0.0])
            return
        
                # Get the destination assigned to this robot.
        goal = GOALS[self.robot]
        
                # Calculate the current distance to the destination.
        distance = math.hypot(
            goal[0] - self.pose[0],
            goal[1] - self.pose[1]
        )
        
                # Require consecutive measurements inside tolerance.
        if distance <= GOAL_TOLERANCE:
            self.settled += 1
        else:
            self.settled = 0
        
                # Declare arrival after enough consecutive in-tolerance measurements.
        if self.settled >= SETTLE_TICKS:
            self.arrived = True
        
                    # Stop all three wheels.
            self.send([0.0, 0.0, 0.0])
        
                    # Calculate elapsed time since control began.
            elapsed = (
                self.get_clock().now() - self.t_start
            ).nanoseconds / 1e9
        
            self.get_logger().info(
                f"{self.robot} reached its goal "
                f"({distance * 1000:.1f} mm out)"
            )
        
                    # Notify main() that this robot has arrived.
            self.on_arrival(self.robot, elapsed)
            return
        
        wheels, distance = self.ctl.step(self.pose,goal,self.hold_yaw)
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