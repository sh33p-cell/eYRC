#!/usr/bin/env python3
"""
Boilerplate controller for the HE bot.

Fetches a shape from the get_shape service and builds a list of waypoints
to trace it. Fill in the control loop to drive the robot through them.


"""

import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from shape_interface.srv import GetShape

# Wheel <-> body-velocity mapping, columns are [left, right, back] wheel
# speed (rad/s); rows are body frame [vx, vy, wz] per unit wheel speed.
_WHEEL_TO_BODY = np.array([
        [np.cos(np.radians(30)),
         np.cos(np.radians(150)),
         np.cos(np.radians(270))],

        [np.sin(np.radians(30)),
         np.sin(np.radians(150)),
         np.sin(np.radians(270))],

        [1,
         1,
         1]
    ])
_BODY_TO_WHEEL = np.linalg.inv(_WHEEL_TO_BODY)
_CTRL_LIMIT = 3.14     # rad/s, matches lekiwi.xml actuator ctrlrange

WAYPOINT_TOLERANCE = 0.04   # metres(given by eytr team in discuss form)
CIRCLE_SEGMENTS     = 36    
POSITION_KP         = 25.0  # the propotional constant (P)
POSITION_KD         = 5.0  # the derivative constant (D)
POSITION_KI         = 5.0   # the integral constant (I)
YAW_HOLD_KP         = 3.0   # propstional constant for yaw(P of yaw)
CONTROL_PERIOD      = 0.02    # how frequently the control step is being called

def body_to_wheels(vx, vy, wz):
    """Body-frame (vx, vy, wz) -> wheel angular velocities [left, right, back]."""
    # TODO: convert body velocity to wheel speeds using _BODY_TO_WHEEL,
    # clip each wheel to [-_CTRL_LIMIT, _CTRL_LIMIT], return as a list.

    X = np.array([vx, vy, wz])
    M_inv = np.array([
        [np.cos(np.radians(30)), np.sin(np.radians(30)), 1],
        [np.cos(np.radians(150)), np.sin(np.radians(150)), 1],
        [np.cos(np.radians(270)), np.sin(np.radians(270)), 1]
    ])
    wheel_speeds = (M_inv @ X)
    wheel_speeds = np.clip(wheel_speeds,-(_CTRL_LIMIT),_CTRL_LIMIT)
    return list(wheel_speeds)


def yaw_from_quat(w, x, y, z):
    # TODO: convert quaternion to yaw (radians).
    return math.atan2(2 * (w * z + x * y), w * w + x * x - y * y - z * z)


def _regular_polygon(cx, cy, n_sides, side_length, start_angle=math.pi / 2):
    """Vertices of a regular polygon centred on (cx, cy), closed back to the
    first vertex so the last waypoint returns the robot to where it started
    drawing."""
    r = side_length / (2 * math.sin(math.pi / n_sides))
    pts = [
        (cx + r * math.cos(start_angle + 2 * math.pi * i / n_sides),
         cy + r * math.sin(start_angle + 2 * math.pi * i / n_sides))
        for i in range(n_sides)
    ]
    return pts + [pts[0]]


def build_waypoints(shape_name, data):
    """World-frame waypoints for `shape_name`, as returned by the get_shape
    service: data[0:2] is the shape's centre (x, y); the remaining entries
    are its size parameters (see shape_service.cpp's shape_map)."""
    cx, cy = data[0], data[1]

    if shape_name == "Circle":
        radius = data[2]
        return [
            (cx + radius * math.cos(2 * math.pi * i / CIRCLE_SEGMENTS),
             cy + radius * math.sin(2 * math.pi * i / CIRCLE_SEGMENTS))
            for i in range(1, CIRCLE_SEGMENTS + 1)
        ]

    if shape_name == "Square":
        return _regular_polygon(cx, cy, 4, data[2], start_angle=math.pi / 4)

    if shape_name == "Triangle":
        return _regular_polygon(cx, cy, 3, data[2])

    if shape_name == "Pentagon":
        return _regular_polygon(cx, cy, 5, data[2])

    if shape_name == "Rectangle":
        w, h = data[2], data[3]
        corners = [
            (cx - w / 2, cy - h / 2),
            (cx + w / 2, cy - h / 2),
            (cx + w / 2, cy + h / 2),
            (cx - w / 2, cy + h / 2),
        ]
        return corners + [corners[0]]

    raise ValueError(f"unknown shape '{shape_name}'")


class ShapeController(Node):
    def __init__(self, speed):
        super().__init__("shape_controller")
        self.speed = speed

        self.pose = None        # (x, y, yaw), latest ground truth
        self.start_pose = None  # (x, y, yaw), recorded on first odom message
        self.wp_index = 0            #this is where next waypoint is
        self.done = False
        self.waypoints = self._request_shape()[1]                #_request_shape give two things that is first shape name second the list of waypoints 
        self.integral_ex = 0.0
        self.integral_ey = 0.0
        self.prev_ex = 0.0
        self.prev_ey = 0.0
#Add the publsiher and subscriber scripts
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_commands",        # this is publisher that give commands 
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,                           # this is subscriber which reads command
            "/odom",
            self._odom_cb,
            10
        )
        self.timer = self.create_timer(          #to call the control step function every Control period (uses ros2)
            CONTROL_PERIOD,
            self._control_step
        )

    def _request_shape(self):
        client = self.create_client(GetShape, "get_shape")
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for get_shape service...")

        future = client.call_async(GetShape.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                f"get_shape service call failed: {response and response.message}"
            )

        return response.shape_name, build_waypoints(response.shape_name, list(response.data))

    def _odom_cb(self, msg):
        # TODO: extract (x, y, yaw) from msg.pose.pose into self.pose,
        # and record self.start_pose on the first callback.
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        w=msg.pose.pose.orientation.w

        yaw=yaw_from_quat(w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        self.pose=(x, y, yaw)

        if(self.start_pose is None):           #for second condition
            self.start_pose=self.pose

    def _publish(self, wheels):
        self.cmd_pub.publish(Float64MultiArray(data=wheels))

    def _control_step(self):
        if self.done or self.pose is None:
            return

        x, y, yaw = self.pose

        target_x, target_y = self.waypoints[self.wp_index]

        dx = target_x - x
        dy = target_y - y

        distance = math.hypot(dx, dy)

        if distance < WAYPOINT_TOLERANCE:
            self.wp_index += 1
            self.integral_ex = 0.0
            self.integral_ey = 0.0
            self.prev_ex = 0.0                #after reaching the waypoints the errors will become zero
            self.prev_ey = 0.0

            if self.wp_index >= len(self.waypoints):
                self.done = True
                self._publish([0.0, 0.0, 0.0])   #to stop things totally
                return

            return

        self.integral_ex += dx * CONTROL_PERIOD            #to find integral that is  summation of dx with every control period of time
        self.integral_ey += dy * CONTROL_PERIOD

        derivative_ex = (dx - self.prev_ex) / CONTROL_PERIOD
        derivative_ey = (dy - self.prev_ey) / CONTROL_PERIOD      # same as intergral (the derivative)

        output_x = (POSITION_KP * dx + POSITION_KI * self.integral_ex + POSITION_KD * derivative_ex)   
        output_y = (POSITION_KP * dy + POSITION_KI * self.integral_ey + POSITION_KD * derivative_ey)    # this is w.r.t world corrdinates

        self.prev_ex = dx 
        self.prev_ey = dy

        body_x = (math.cos(yaw) * output_x + math.sin(yaw) * output_y)         #corrected according to body
        body_y = (-math.sin(yaw) * output_x + math.cos(yaw) * output_y)


        #yaw hold kp
        target_yaw = self.start_pose[2]
        dyaw = target_yaw - yaw
        output_yaw = YAW_HOLD_KP * dyaw

        self._publish(body_to_wheels(body_x,body_y,output_yaw))               #publishing the velocities (it is working without wz as there is no target wz)



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.25,
                         help="max approach speed, m/s")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ShapeController(args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish([0.0, 0.0, 0.0])
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()