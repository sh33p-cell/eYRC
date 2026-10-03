#!/usr/bin/env python3
# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2025-26)
*        =============================================
*
*  This script is to implement Task 1A of Hola The Explorer (HE) Theme (eYRC 2025-26).
*
*****************************************************************************************
'''

# Team ID:          eYRC#2879
# Author List:      Aayush Jadhav, Vishesh Satarkar
# Filename:         camera_detection.py
# Functions:        centre_of_quad(), find_trapezoids(), main(), angle_difference()
# Global variables: STREAM_URL, WINDOW, BINARY_WINDOW, FPS_WINDOW, ARENA_*, SAND_DISTANCE,
#                   HOUGH_*, MIN_TRAPEZOID_AREA, PARALLEL_TOLERANCE_DEG, REPORT_PERIOD_SEC
# Service Clients:  pixel_to_world  ->  shape_interface/srv/PixelToWorld


import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from shape_interface.srv import PixelToWorld

STREAM_URL = "http://127.0.0.1:8080/stream"
WINDOW = "camera_feed"
BINARY_WINDOW = "trapezoid_borders"
FPS_WINDOW = 30                 # number of frames averaged for the FPS readout

# Arena floor in image pixels, at the default 1280x720 stream. Everything
# outside this rectangle is terrain -- the SAME sandy rock as the floor, so if
# you do not crop it away it floods your mask. Re-measure these if you change
# the camera or the stream resolution (open one frame in an image viewer and
# read off the corners of the floor).
ARENA_X0, ARENA_Y0, ARENA_X1, ARENA_Y1 = 304, 24, 975, 695

# How far a pixel's colour must be from the floor's OWN colour before you call
# it "a drawn feature". The floor texture is very uniform, so a modest
# threshold separates cleanly. Raise it if noise leaks in, lower it if the pale
# cyan funnel disappears.
SAND_DISTANCE = 18

# Line-detection parameters. The funnel borders are thin outlines only a few
# pixels wide: a large enough minimum length keeps small icon detail out, and a
# generous maximum gap bridges the break where a safe overlaps a border.
HOUGH_THRESHOLD = 40
HOUGH_MIN_LENGTH = 35
HOUGH_MAX_GAP = 25

MIN_TRAPEZOID_AREA = 1200       # px^2, throws away small enclosed blobs
PARALLEL_TOLERANCE_DEG = 7      # two sides count as parallel within this angle

# Converting and printing at the full frame rate would be unreadable and would
# put ~90 service calls a second on the wire for no benefit.
REPORT_PERIOD_SEC = 0.5


def centre_of_quad(corners):
    """
    Find the centre of a quadrilateral given its four corners.

    Parameters
    ----------
    corners :  [ numpy array of shape (4, 2), dtype float ]
        the four corners of the quadrilateral, in pixel coordinates

    Returns
    -------
    cx :  float 
        x coordinate of the centre, in pixels
    cy :  float 
        y coordinate of the centre, in pixels

    Example call
    ------------
    cx, cy = centre_of_quad(corners)
    """

    cx, cy = 0.0, 0.0

    contour = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)

    moments = cv2.moments(contour)

    if abs(moments["m00"]) < 1e-9:
        return cx, cy

    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])

    return cx, cy

def angle_difference(a: float | int, b: float | int) -> float:
    """
    Return the difference between two line angles (modulo 180)

    Parameters
    ----------
    a : float | int
        First line angle
    b : float | int
        Second line angle

    Returns
    -------
    float
        Angle difference between the two line angles (``a`` and ``b``)
    """

    diff = abs(a - b)
    return min(diff, 180.0 - diff)

def find_trapezoids(frame):
    """
    Locate the three station funnels (trapezoids) in one camera frame.

    Parameters
    ----------
    frame :  np.ndarray
        one BGR frame straight from the camera stream

    Returns
    -------
    binary :  [ numpy array, uint8, same height/width as `frame` ]
        trapezoid borders in white (255) on black (0), nothing else
    trapezoids :  [ list of tuples ]
        one (centre_x, centre_y, corners) per detected trapezoid, where
        `corners` is a (4, 2) array in FULL-FRAME pixel coordinates

    Example call
    ------------
    binary, trapezoids = find_trapezoids(frame)
    """

    binary = np.zeros(frame.shape[:2], np.uint8)
    trapezoids = []

    arena = frame[ARENA_Y0:ARENA_Y1, ARENA_X0:ARENA_X1].copy()

    # making sure size of arena isn't 
    if arena.size == 0 or arena.shape != (671, 671, 3):
        return binary, trapezoids
    
    lab_conv = cv2.cvtColor(arena, cv2.COLOR_BGR2Lab)

    floor_color = []

    for channel in cv2.split(lab_conv):
        histogram = np.bincount(channel.ravel(), minlength=256)
        floor_color.append(int(np.argmax(histogram)))

    floor_color = np.asarray(floor_color, dtype=np.int32)

    diff = lab_conv.astype(np.int32) - floor_color

    distance = np.sqrt(np.sum(diff ** 2, axis=2))

    mask = (distance > SAND_DISTANCE).astype(np.uint8) * 255

    kernel = np.ones((3, 3), dtype=np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    edges = cv2.Canny(mask, 100, 200)

    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=HOUGH_THRESHOLD, maxLineGap=HOUGH_MAX_GAP, minLineLength=HOUGH_MIN_LENGTH)

    # check if HoughLinesP did not return None object
    if lines is None:
        return binary, trapezoids

    scratch_mask = np.zeros_like(mask)

    for line in lines:
        x1, y1, x2, y2 = map(int, line[0])

        cv2.line(scratch_mask, (x1,y1), (x2,y2), (255, 255, 255), 3)

    scratch_kernel = np.ones((3, 3), dtype=np.uint8)

    scratch_mask = cv2.morphologyEx(scratch_mask, cv2.MORPH_CLOSE, scratch_kernel)

    contours, hierarchy = cv2.findContours(image=scratch_mask, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)

    if hierarchy is None:
        return binary, trapezoids

    hierarchy = hierarchy[0]

    candidates = []

    for i, contour in enumerate(contours):
        parent = int(hierarchy[i][3])

        if parent < 0:
            continue

        area = abs(cv2.contourArea(contour))

        if area < MIN_TRAPEZOID_AREA:
            continue

        perimeter = cv2.arcLength(contour, closed=True)

        if perimeter <= 0:
            continue

        approx = cv2.approxPolyDP(contour, epsilon=0.03 * perimeter, closed=True)

        if len(approx) != 4:
            continue

        if not cv2.isContourConvex(approx):
            continue

        points = approx.reshape(4, 2).astype(np.float32)

        angles = []

        for j in range(0, 4):
            p1 = points[j]
            p2 = points[(j + 1) % 4]

            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])

            angle = math.degrees(math.atan2(dy, dx))
            angle %= 180.0

            angles.append(angle)

        # find angle between opposite lines of the polygon, if the difference <= 7 deg
        pair_02_parallel = (angle_difference(angles[0], angles[2]) <= PARALLEL_TOLERANCE_DEG)
        pair_13_parallel = (angle_difference(angles[1], angles[3]) <= PARALLEL_TOLERANCE_DEG)

        # exactly one pair should be parallel, if sum of these variables != 1, then 0 or 2 pairs are parallel
        if int(pair_02_parallel) + int(pair_13_parallel) != 1:
            continue

        corners = points.copy()

        # converting coordinates from cropped image, to coordinates of the entire frame
        corners[:, 0] += ARENA_X0
        corners[:, 1] += ARENA_Y0

        cx, cy = centre_of_quad(corners)

        candidates.append((cx, cy, corners, area))

    candidates.sort(key=lambda item : item[3], reverse=True)

    for cx, cy, corners, area in candidates:
        duplicate = False

        for old_cx, old_cy, _ in trapezoids:
            if math.hypot(cx - old_cx, cy - old_cy) < 25.0:
                duplicate = True
                break

        if duplicate:
            continue

        trapezoids.append((cx, cy, corners))

    for cx, cy, corners in trapezoids:
        polygon = np.round(corners).astype(np.int32)

        cv2.polylines(binary, [polygon], isClosed=True, color=255, thickness=3)

    return binary, trapezoids


##############################################################
def main():
    """
    Open the camera stream, run find_trapezoids() on every frame, display the
    result, and periodically convert each trapezoid centre to world
    coordinates through the `pixel_to_world` service.

    Parameters
    ----------
    None

    Returns
    -------
    None

    Example call
    ------------
    Called automatically by the Python interpreter.
    """

    rclpy.init()
    node = Node("camera_feed")
    client = node.create_client(PixelToWorld, "pixel_to_world")

    node.get_logger().info("waiting for the pixel_to_world service ...")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            "pixel_to_world is not up. Start it first: ros2 run task_1a pixel_to_world_service")
        rclpy.shutdown()
        return

    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        node.get_logger().error(f"could not open {STREAM_URL}")
        node.get_logger().error(
            "start the simulation first: ros2 launch hb_description task1a.launch.py")
        rclpy.shutdown()
        return

    stamps = deque(maxlen=FPS_WINDOW)
    fps = 0.0
    last_report = 0.0

    try:
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                break
    
            # rolling frame rate over the last FPS_WINDOW frames
            stamps.append(time.monotonic())
            if len(stamps) >= 2:
                span = stamps[-1] - stamps[0]
                fps = (len(stamps) - 1) / span if span > 0 else 0.0
    
            binary, trapezoids = find_trapezoids(frame)
    
            # overlay: red outline + yellow centre dot for every detection
            for cx, cy, corners in trapezoids:
                cv2.polylines(frame, [np.round(corners).astype(np.int32)], True, (0, 0, 255), 2)
                cv2.circle(frame, (int(round(cx)), int(round(cy))), 6, (0, 255, 255), -1)
    
            cv2.putText(frame, f"{fps:5.1f} FPS", (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"{len(trapezoids)} trapezoids", (12, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, frame)
            cv2.imshow(BINARY_WINDOW, binary)
    
            now = time.monotonic()
            if trapezoids and now - last_report >= REPORT_PERIOD_SEC:
                last_report = now
                print(f"\n{len(trapezoids)} trapezoid(s):")
    
                # sorted top-to-bottom, then left-to-right, so the printed order is
                # stable from frame to frame
                for cx, cy, _ in sorted(trapezoids, key=lambda t: (t[1], t[0])):
    
                    request = PixelToWorld.Request()
    
                    request.pixel_x = np.float64(cx)
                    request.pixel_y = np.float64(cy)
    
                    future = client.call_async(request)
    
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
    
                    result = future.result()
    
                    if result is None:
                        print(f"    pixel ({cx:7.2f}, {cy:7.2f}) -> "
                                f"timeout")
                    elif not result.success:
                        print(f"    pixel ({cx:7.2f}, {cy:7.2f}) -> "
                                f"ERROR: {result.message}")
                    else:
                        wx = float(result.world_x)
                        wy = float(result.world_y)
    
                        print(f"    pixel ({cx:7.2f}, {cy:7.2f}) -> "
                                f"world ({wx:6.3f}, {wy:6.3f}) m")
                    ##################################################
    
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


##############################################################
if __name__ == "__main__":
    main()