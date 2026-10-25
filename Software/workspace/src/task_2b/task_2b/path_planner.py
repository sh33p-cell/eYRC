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
*  This script is to implement Task 2B of Hola The Explorer (HE) Theme (eYRC 2026-27).
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         path_planner.py
# Functions:        segment_clear(), astar(), simplify_path()
# Global variables: None added.


############################ WHAT YOU HAVE TO DO ##############################
#
#  r2 must visit the three GOALS in order without touching an obstacle. This
#  node plans its path ONCE and publishes it on /r2/plan; path_follower.py
#  drives it.
#
#    camera frame --frame_to_occupancy()--> grid       (given)
#                 --remove_drivable()-----> grid       (YOURS)
#                 --planning_grid()-------> blocked    (given)
#                 --plan_route()----------> plan       (YOURS)
#                 --to_path_msg()---------> /r2/plan   (YOURS)
#
#  A plan is [(t, x, y), ...]: be at (x, y) t seconds after setting off.
#  To stop on a goal, repeat its point with a later t.
#
#  Frame (/r2/odom): origin at the arena's TOP-LEFT corner, x right, y DOWN.
#
#  Run:  ros2 launch hb_description task2b.launch.py
#        ros2 run task_2b path_planner
#        ros2 run task_2b path_follower
#
###############################################################################

import math
import heapq

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

###############################################################


# ----------------------------------------------------- arena (given, do not edit)
ROBOTS = ("r2",)
START_CIRCLES = [(0.8969, 2.2616), (1.2192, 2.2616), (1.5415, 2.2616)]
GOALS = [(1.2192, 0.4992),           # visited in this order
         (0.3592, 1.5592),
         (2.0792, 0.6992)]
ARENA_SIZE = 2.4384                  # m
ROBOT_RADIUS = 0.084                 # m
SAFETY_MARGIN = 0.030                # m, kept clear on top of the robot radius

# ----------------------------------------------------- camera (given)
CAMERA_URL = "http://127.0.0.1:8080/stream"
ARENA_LEFT, ARENA_TOP, ARENA_PIXELS = 304, 24, 671       # floor square in the image
GRID_CELLS = 244                     # occupancy grid is GRID_CELLS x GRID_CELLS
CELL_SIZE = ARENA_SIZE / GRID_CELLS  # m, about 1 cm
SAND_LOW, SAND_HIGH = (10, 40, 170), (30, 120, 255)      # sand colour, HSV

# ----------------------------------------------------- planning (tunable)
PLAN_CELL = 0.04                     # m, planning grid cell
PLAN_SPEED = 0.24                    # m/s, the speed the path is timed at
GOAL_STOP = 2.0                      # s, stop this long on each goal
START_DELAY = 2.0                    # s, from publishing to setting off

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


######################### OCCUPANCY GRID ######################

def read_frame(url=CAMERA_URL):
    """Given. One BGR frame from the overhead camera."""
    cap = cv2.VideoCapture(url)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("no frame from %s -- is task2b.launch.py running?" % url)
    return frame


def crop_arena(frame):
    """Given. The arena floor only, ARENA_PIXELS square."""
    return frame[ARENA_TOP:ARENA_TOP + ARENA_PIXELS, ARENA_LEFT:ARENA_LEFT + ARENA_PIXELS]


def frame_to_occupancy(frame):
    """Given. Frame -> grid[row, col] = grid[y, x], True = obstacle.
    Only sand is free. To view: cv2.imshow("g", np.where(grid, 0, 255).astype(np.uint8))"""
    hsv = cv2.cvtColor(cv2.medianBlur(crop_arena(frame), 5), cv2.COLOR_BGR2HSV)
    obstacles = cv2.bitwise_not(cv2.inRange(hsv, SAND_LOW, SAND_HIGH))
    # drop specks and thin painted lines, then fill the holes inside each obstacle
    obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(obstacles, contours, -1, 255, cv2.FILLED)
    # one pixel per cell; a cell is an obstacle if any part of it is
    small = cv2.resize(obstacles, (GRID_CELLS, GRID_CELLS), interpolation=cv2.INTER_AREA)
    return small > 0


def clear_circle(occupied, xy, radius):
    """Given. Free every cell within `radius` m of point xy (in place)."""
    rows, cols = np.indices(occupied.shape)
    x, y = (cols + 0.5) * CELL_SIZE, (rows + 0.5) * CELL_SIZE
    occupied[(x - xy[0]) ** 2 + (y - xy[1]) ** 2 <= radius ** 2] = False


def remove_drivable(frame, occupied, robots, goals):
    """Free what a robot may drive over; return the grid.

    frame_to_occupancy() marks everything that is not sand as an obstacle --
    including the painted floor, the start circles, the robot and the goal
    dots, which leaves NO path. Keep the pillars (and each pad's payload)."""
    ##############  ADD YOUR CODE HERE  ##############
    arena = crop_arena(frame)
    if arena.shape != (ARENA_PIXELS, ARENA_PIXELS, 3):
        raise ValueError("Camera frame does not contain the expected 671 x 671 arena")
    if occupied.shape != (GRID_CELLS, GRID_CELLS):
        raise ValueError("Unexpected occupancy grid shape")

    # Work in the same cropped image/HSV coordinates as frame_to_occupancy.
    # Sand is yellow/brown; floor artwork is saturated colour. Grey pillars
    # are deliberately excluded. These thresholds assume the supplied camera.
    hsv = cv2.cvtColor(cv2.medianBlur(arena, 5), cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    paint = ((s >= 70) & (v >= 45) &
             ((h < 10) | (h > 30) | (s >= 135))).astype(np.uint8) * 255
    paint = cv2.morphologyEx(paint, cv2.MORPH_CLOSE,
                             np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(paint, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    drivable = np.zeros_like(paint)
    payloads = np.zeros_like(paint)
    pixels_per_metre = ARENA_PIXELS / ARENA_SIZE
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 12.0:
            continue
        # Fill artwork, including its thin outlines and internal markings.
        cv2.drawContours(drivable, [contour], -1, 255, cv2.FILLED)
        perimeter = cv2.arcLength(contour, True)
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        fill_ratio = area / max(math.pi * radius * radius, 1.0)
        # The circular coloured pads are drivable, but their central payload
        # is solid. Goal dots/start circles are smaller than these pads.
        if (0.12 <= radius / pixels_per_metre <= 0.23 and
                circularity >= 0.65 and fill_ratio >= 0.70):
            cv2.circle(payloads, (round(cx), round(cy)),
                       math.ceil(0.095 * pixels_per_metre), 255, -1)

    # Include the narrow antialiased borders around the coloured artwork.
    # Solid masks below are restored after this small expansion.
    drivable = cv2.dilate(drivable, np.ones((13, 13), np.uint8))

    # Restore substantial grey features even when surrounded by paint.
    # Opening removes thin pale artwork lines, not full pillar footprints.
    grey = ((s <= 65) & (v <= 205)).astype(np.uint8) * 255
    grey = cv2.morphologyEx(grey, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    grey_contours, _ = cv2.findContours(grey, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
    solid = payloads.copy()
    # The start rings and robot image contain grey pixels too. Exclude only
    # whole components inside their known image footprints; clipping a disc
    # out of every solid would also cut into a neighbouring pillar. The extra
    # 25 mm covers the painted ring and camera/resampling fringe.
    removable_centres = list(START_CIRCLES) + list(robots)
    image_radius = ROBOT_RADIUS + 0.025
    for contour in grey_contours:
        x, y, width, height = cv2.boundingRect(contour)
        # Small grey highlights in coloured floor artwork are not pillars.
        # Only discard a speck when its entire bounding box is in the paint
        # mask; isolated grey objects and larger solid features stay protected.
        if (max(width, height) / pixels_per_metre <= 0.035 and
                np.all(drivable[y:y + height, x:x + width] > 0)):
            continue
        points = contour[:, 0, :] / pixels_per_metre
        if any(np.all(np.sum((points - xy) ** 2, axis=1) <= image_radius ** 2)
               for xy in removable_centres):
            continue
        cv2.drawContours(solid, [contour], -1, 255, cv2.FILLED)
    drivable[solid > 0] = 0

    # Require most of a fine-grid cell to be paint before freeing it.
    free_small = cv2.resize(drivable, (GRID_CELLS, GRID_CELLS),
                            interpolation=cv2.INTER_AREA) >= 128
    solid_small = cv2.resize(solid, (GRID_CELLS, GRID_CELLS),
                             interpolation=cv2.INTER_AREA) > 0
    cleaned = occupied.copy()
    cleaned[free_small] = False
    # Remove one-cell artwork fringes left by resizing. Restore all detected
    # solid features afterwards, so this does not shrink grey pillars/payloads.
    cleaned = cv2.morphologyEx(cleaned.astype(np.uint8), cv2.MORPH_OPEN,
                               np.ones((3, 3), np.uint8)) > 0
    cleaned[solid_small] = True

    for xy in START_CIRCLES:
        clear_circle(cleaned, xy, ROBOT_RADIUS + 0.025)
    for xy in robots:
        clear_circle(cleaned, xy, ROBOT_RADIUS + 0.025)
    for xy in goals:
        clear_circle(cleaned, xy, ROBOT_RADIUS + SAFETY_MARGIN + PLAN_CELL)
    # Clearing a marker must not erase a nearby pillar or a payload. The
    # robot itself is the only known solid object that may be removed: it
    # occupies its measured footprint in this camera frame.
    protected = solid_small.copy()
    for xy in robots:
        clear_circle(protected, xy, ROBOT_RADIUS)
    cleaned[protected] = True
    return cleaned
    ##################################################


######################### PLANNING GRID #######################

def planning_grid(occupied):
    """Given. Grid -> blocked[i, j] = blocked[x, y] (NOTE: x first) of PLAN_CELL
    cells. Obstacles grow by ROBOT_RADIUS + SAFETY_MARGIN, so plan the robot's
    centre as a point. The arena's edge is a wall."""
    pad = ROBOT_RADIUS + SAFETY_MARGIN
    k = int(math.ceil(pad / CELL_SIZE))
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    inflated = cv2.dilate(occupied.astype(np.uint8), disc) > 0

    # a planning cell is blocked if the robot's centre, standing on the cell's
    # centre, would be too close to an obstacle
    n = int(round(ARENA_SIZE / PLAN_CELL))
    centre = (np.arange(n) + 0.5) * PLAN_CELL
    fine = np.minimum((centre / CELL_SIZE).astype(int), GRID_CELLS - 1)
    blocked = inflated[np.ix_(fine, fine)].T        # [row=y, col=x] -> [i=x, j=y]
    near_wall = (centre < pad) | (centre > ARENA_SIZE - pad)
    blocked[near_wall, :] = True
    blocked[:, near_wall] = True
    return blocked


def to_cell(p):
    """Given. Point (x, y) in m -> planning cell (i, j)."""
    n = int(round(ARENA_SIZE / PLAN_CELL))
    return (min(max(int(p[0] / PLAN_CELL), 0), n - 1), min(max(int(p[1] / PLAN_CELL), 0), n - 1))


def to_world(c):
    """Given. Planning cell (i, j) -> its centre (x, y) in m."""
    return ((c[0] + 0.5) * PLAN_CELL, (c[1] + 0.5) * PLAN_CELL)


def nearest_free(blocked, p, max_r=0.30):
    """Given. The free cell closest to point p, or None within max_r m."""
    c = to_cell(p)
    if not blocked[c]:
        return c
    free = [(i, j) for i in range(blocked.shape[0]) for j in range(blocked.shape[1])
            if not blocked[i, j] and abs(i - c[0]) * PLAN_CELL <= max_r
            and abs(j - c[1]) * PLAN_CELL <= max_r]
    return min(free, key=lambda q: math.dist(to_world(q), p)) if free else None


def segment_clear(blocked, a, b):
    """Check the entire segment against closed blocked-cell squares.

    Unlike sparse point sampling, this also catches a line grazing a blocked
    corner. The input grid has already been inflated by the robot radius.
    """
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    pad = ROBOT_RADIUS + SAFETY_MARGIN
    if (a.shape != (2,) or b.shape != (2,) or
            not np.all(np.isfinite(a)) or not np.all(np.isfinite(b))):
        return False
    if np.any(np.minimum(a, b) < pad) or np.any(np.maximum(a, b) > ARENA_SIZE - pad):
        return False
    cells = np.argwhere(blocked)
    if not len(cells):
        return True
    low = cells * PLAN_CELL
    high = low + PLAN_CELL
    candidates = np.all((high >= np.minimum(a, b) - 1e-10) &
                        (low <= np.maximum(a, b) + 1e-10), axis=1)
    low, high = low[candidates], high[candidates]
    if not len(low):
        return True
    enter = np.zeros(len(low))
    leave = np.ones(len(low))
    possible = np.ones(len(low), dtype=bool)
    delta = b - a
    for axis in (0, 1):
        if abs(delta[axis]) < 1e-12:
            possible &= ((a[axis] >= low[:, axis] - 1e-10) &
                         (a[axis] <= high[:, axis] + 1e-10))
        else:
            t0 = (low[:, axis] - a[axis]) / delta[axis]
            t1 = (high[:, axis] - a[axis]) / delta[axis]
            enter = np.maximum(enter, np.minimum(t0, t1))
            leave = np.minimum(leave, np.maximum(t0, t1))
    return not bool(np.any(possible & (enter <= leave + 1e-10)))


def astar(blocked, start, goal):
    """Eight-connected A*, with no diagonal corner cutting; cells are (x, y)."""
    if blocked[start] or blocked[goal]:
        return None

    def heuristic(cell):
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    frontier = [(heuristic(start), 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    while frontier:
        _, g, current = heapq.heappop(frontier)
        if g > cost[current] + 1e-12:
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = parent[current]
                path.append(current)
            return path[::-1]
        x, y = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (x + dx, y + dy)
            if not (0 <= nxt[0] < blocked.shape[0] and
                    0 <= nxt[1] < blocked.shape[1]):
                continue
            if blocked[nxt]:
                continue
            if dx and dy and (blocked[x + dx, y] or blocked[x, y + dy]):
                continue
            new_cost = g + (math.sqrt(2.0) if dx and dy else 1.0)
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt] = new_cost
                parent[nxt] = current
                heapq.heappush(frontier, (new_cost + heuristic(nxt), new_cost, nxt))
    return None


def simplify_path(blocked, points):
    """Remove zigzags only when the whole replacement segment is clear."""
    if not points:
        return None
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i and not segment_clear(blocked, points[i], points[j]):
            j -= 1
        if j == i:
            return None
        result.append(points[j])
        i = j
    return result


def plan_route(blocked, start):
    """start -> GOALS[0] -> GOALS[1] -> GOALS[2] on `blocked`.
    Returns [(t, x, y), ...] starting with (0.0, *start), or None."""
    ##############  ADD YOUR CODE HERE  ##############
    if PLAN_SPEED <= 0.0 or GOAL_STOP <= 0.0:
        raise ValueError("PLAN_SPEED and GOAL_STOP must be positive")
    expected = int(round(ARENA_SIZE / PLAN_CELL))
    if blocked.shape != (expected, expected):
        raise ValueError("Unexpected planning grid shape")
    current = tuple(map(float, start))
    if not segment_clear(blocked, current, current):
        return None
    plan = [(0.0, *current)]
    t = 0.0
    for goal in GOALS:
        goal = tuple(map(float, goal))
        # Never silently substitute a nearby point for a required goal.
        if not segment_clear(blocked, goal, goal):
            return None
        first, last = nearest_free(blocked, current), nearest_free(blocked, goal)
        if first is None or last is None:
            return None
        cells = astar(blocked, first, last)
        if cells is None:
            return None
        points = [current] + [to_world(cell) for cell in cells] + [goal]
        points = simplify_path(blocked, points)
        if points is None:
            return None
        for point in points[1:]:
            distance = math.dist(current, point)
            if distance > 1e-12:
                t += distance / PLAN_SPEED
                plan.append((t, *point))
            current = point
        # Identical consecutive positions with increasing times specify a stop.
        t += GOAL_STOP
        plan.append((t, *goal))
        current = goal
    return plan
    ##################################################


######################### NODE ################################

class PathPlanner(Node):
    """Given, except to_path_msg(). Waits for /r2/odom, plans once, and
    publishes the path on /r2/plan (latched, so a late follower still gets it)."""

    def __init__(self):
        super().__init__("task_2b_path_planner")
        self.pose, self.pubs = {}, {}
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom", self._odom_cb(r), 10)
            self.pubs[r] = self.create_publisher(Path, f"/{r}/plan", LATCHED)
        self.planned = False
        self.create_timer(0.2, self.tick)
        self.get_logger().info("task 2B planner: %s visits %s" % (ROBOTS[0], GOALS))

    def _odom_cb(self, robot):
        def cb(msg):
            p = msg.pose.pose.position
            self.pose[robot] = (p.x, p.y)
        return cb

    def to_path_msg(self, plan, start):
        """[(t, x, y), ...] and start (rclpy Time) -> nav_msgs/Path."""
        ##############  ADD YOUR CODE HERE  ##############
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = start.to_msg()
        for t, x, y in plan:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = (start + Duration(seconds=float(t))).to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # valid identity quaternion
            msg.poses.append(pose)
        return msg
        ##################################################

    def tick(self):
        if self.planned or len(self.pose) < len(ROBOTS):
            return                              # wait for the robot's odom
        self.planned = True
        robot = ROBOTS[0]

        frame = read_frame()
        occupied = remove_drivable(frame, frame_to_occupancy(frame),
                                   robots=list(self.pose.values()), goals=GOALS)
        blocked = planning_grid(occupied)
        self.get_logger().info("occupancy grid: %.1f%% obstacle; planning grid %d x %d"
                               % (100.0 * occupied.mean(), *blocked.shape))

        route = plan_route(blocked, self.pose[robot])
        if route is None:
            self.get_logger().error("no route. Start and goals, as the planner saw them:")
            for name, p in [("start", self.pose[robot])] + [("goal %d" % (k + 1), g) for k, g in enumerate(GOALS)]:
                self.get_logger().error("  %s %s %s" % (name, p, "BLOCKED" if blocked[to_cell(p)] else "free"))
            return

        start = self.get_clock().now() + Duration(seconds=START_DELAY)
        self.pubs[robot].publish(self.to_path_msg(route, start))
        length = sum(math.dist(a[1:], b[1:]) for a, b in zip(route, route[1:]))
        self.get_logger().info("%s: published /%s/plan, %d poses, %.2f m, finishes at t=%.1f s"
                               % (robot, robot, len(route), length, route[-1][0]))


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)                        # keeps the latched path available
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
