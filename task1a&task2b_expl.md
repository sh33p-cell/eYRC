# Explain Task 1A and 2B files

> I have a viva on implementation of task 1a & 2b. Give me a detailed explanation of each file involved in task 1a & 2b.

<details><summary>5 previous messages</summary>

> I’ll inspect both task packages and their launch/configuration dependencies, then explain each file and how the nodes work together for your viva.
>
> <details><summary>Explored 3 files, 2 lists, ran 6 commands</summary>
>
> - Ran `pwd && rg --files -g 'AGENTS.md' -g '*task_1a*' -g '*task_2b*' -g 'README*' -g 'package.xml' -g 'CMakeLists.txt' -g 'setup.py' -g 'setup.cfg'`
>
> - Ran `rg --files -g AGENTS.md -g '!Software/workspace/build/**' -g '!Software/workspace/install/**' -g '!Software/workspace/log/**' /home/aayush 2>/dev/null`
>
> <details><summary>Explored a list</summary>
>
> - Listed files in `task_1a`
>
> </details>
>
> - Ran `wc -l Software/workspace/src/task_1a/task_1a/camera_detection.py Software/workspace/src/task_2b/task_2b/*.py && cat Software/workspace/src/task_1a/ReadMe.md Software/workspace/src/task_2b/ReadMe.md`
>
> <details><summary>Explored 6 files, a list</summary>
>
> - Read `./Software/workspace/src/task_1a/task_1a/camera_detection.py`
> - Read `./Software/workspace/src/task_2b/task_2b/path_planner.py`
> - Read `./Software/workspace/src/task_2b/task_2b/path_follower.py`
> - Read `./Software/workspace/src/task_2b/task_2b/path_planner.py`
> - Read `./Software/workspace/src/task_1a/task_1a/camera_detection.py`
> - Listed files
> - Read `./Software/workspace/src/task_2b/task_2b/path_planner.py`
>
> </details>
>
> - Ran `for f in Software/workspace/src/hb_description/launch/task1a.launch.py Software/workspace/src/hb_description/launch/task2b.launch.py Software/workspace/src/hb_description/CMakeLists.txt Software/workspace/src/hb_description/package.xml ReadMe.md; do echo "$f"; nl -ba "$f"; done`
>
> - Ran `for f in Software/workspace/src/task_1a/CMakeLists.txt Software/workspace/src/task_1a/package.xml Software/workspace/src/task_2b/CMakeLists.txt Software/workspace/src/task_2b/package.xml Software/workspace/src/shape_interface/CMakeLists.txt Software/workspace/src/shape_interface/package.xml Software/workspace/src/shape_interface/srv/PixelToWorld.srv Software/workspace/src/shape_interface/srv/GetShape.srv Software/workspace/src/shape_interface/msg/Shape.msg; do echo "$f"; nl -ba "$f"; done`
>
> - Ran `file Software/workspace/src/task_1a/bin/pixel_to_world_service Software/workspace/src/task_2b/bin/task_2b_node Software/workspace/src/hb_description/bin/task1a_camera Software/workspace/src/hb_description/bin/task1a_bridge Software/workspace/src/hb_description/bin/task2b_camera Software/workspace/src/hb_description/bin/task2b_bridge Software/workspace/src/hb_description/bin/camera_stream; nl -ba Software/workspace/tests/test_task_2.py | sed -n '1,160p'; nl -ba Software/workspace/tests/test_task_2_camera.py | sed -n '1,140p'`
>
> </details>
>
> One important finding: the Task 1A file in this checkout still has empty detection, centroid, and service-call blocks. Task 2B has the planning and control code filled in. I’ll distinguish what Task 1A is intended to do from what currently runs, and explain Task 2B’s actual implementation.
>
> <details><summary>Explored 7 files, a search, ran 3 commands, searched the web</summary>
>
> <details><summary>Explored 5 files, a search</summary>
>
> - Read `./Software/workspace/src/hb_description/launch/task1a.launch.py`
> - Read `./Software/workspace/src/hb_description/launch/task2b.launch.py`
> - Read `./Software/workspace/src/hb_description/CMakeLists.txt`
> - Read `./Software/workspace/src/hb_description/package.xml`
> - Read `./Software/workspace/src/shape_interface/msg/Shape.msg`
> - Searched for `^    def test_|task_2b_node|task_2b|task 2B` in `test_task_2.py`
>
> </details>
>
> - Ran `strings Software/workspace/src/task_2b/bin/task_2b_node | rg 'wheel_commands|/odom|/r2|goal|task_2b|planner|follower|Usage|deprecated|legacy|usage|plan' | head -60; strings Software/workspace/src/task_1a/bin/pixel_to_world_service | rg 'camera_|image_|world_|pixel_|fov|height|outside|arena|invalid' | head -65; rg --files Software/workspace/tests/fixtures`
>
> Searched the web for `https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html`
>
> - Ran `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s Software/workspace/tests -p 'test_task_2.py' -v`
>
> <details><summary>Explored 2 files</summary>
>
> - Read `./Software/workspace/tests/test_task_2.py`
> - Read `./Software/workspace/src/task_1a/task_1a/__init__.py`
>
> </details>
>
> - Ran `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s Software/workspace/tests -p 'test_task_2_camera.py' -v`
>
> Searched the web for `site:docs.ros.org/en/humble "Transient local" "late-joining"`
>
> </details>
>
> I’ve traced the full Task 2B flow: camera image → obstacle cleanup → obstacle inflation → A* → timed path → PID control → wheel commands. The existing offline checks also pass, including route generation and tracking with ideal dynamics. I’ll call out viva details that are easy to miss, especially the grid indexing, coordinate frames, and how timestamps encode goal pauses.
</details>

I reviewed the files in your workspace. **Task 1A is still a partially completed template:** its centroid calculation, trapezoid detection, and pixel-to-world service calls are empty. **Task 2B contains implemented planning and control code.** Below, I distinguish the intended Task 1A algorithm from the actual Task 2B implementation.

The main distinction to understand for your viva is:

- **Task 1A:** extract checkpoint geometry from an image.
- **Task 2B:** identify obstacles, plan a safe route, and control a robot along that route.

---

**1. [Task 1A: `camera_detection.py`](./Software/workspace/src/task_1a/task_1a/camera_detection.py)**

This is the main perception program for Task 1A. It reads the overhead camera, should detect three trapezoidal checkpoints, and displays their borders in a binary image.

Its intended flow is:

```text
Camera image
  → Crop arena
  → Separate floor from markings
  → Clean mask
  → Detect edges and straight lines
  → Find closed quadrilaterals
  → Check trapezoid geometry
  → Draw binary outlines
  → Calculate centroids
  → Request world coordinates
```

**Imports and their purposes**

| Import | Purpose |
|---|---|
| `cv2` | Camera access, colour conversion, image processing, geometry, and display |
| `numpy` | Image arrays, masks, and numerical calculations |
| `math` | Geometric calculations such as side angles |
| `time` | FPS measurement and periodic reporting |
| `deque` | Stores a fixed number of recent frame timestamps |
| `rclpy` | ROS 2 Python communication and node lifecycle |
| `Node` | Creates the ROS node |
| `PixelToWorld` | Custom service type for converting pixels into metres |

An important detail: **the image is received through an HTTP MJPEG stream, not a ROS image topic.**

### How the constants affect detection

The stream URL is:

```python
http://127.0.0.1:8080/stream
```

`127.0.0.1` means the stream server runs on the same machine.

The arena bounds are:

```python
ARENA_X0, ARENA_Y0, ARENA_X1, ARENA_Y1 = 304, 24, 975, 695
```

This defines a `671 × 671` pixel crop inside the full `1280 × 720` image.

Cropping is necessary because the terrain outside the arena resembles the sandy floor. Processing the entire image would introduce irrelevant features.

Other important constants are:

| Constant | Value | Meaning |
|---|---:|---|
| `SAND_DISTANCE` | 18 | Colour-distance threshold for separating features from sand |
| `HOUGH_THRESHOLD` | 40 | Minimum Hough votes required to accept a line |
| `HOUGH_MIN_LENGTH` | 35 px | Rejects short line fragments |
| `HOUGH_MAX_GAP` | 25 px | Allows nearby collinear fragments to form a line |
| `MIN_TRAPEZOID_AREA` | 1200 px² | Rejects small enclosed regions |
| `PARALLEL_TOLERANCE_DEG` | 7° | Permitted angular difference between parallel sides |
| `REPORT_PERIOD_SEC` | 0.5 s | Limits coordinate reporting frequency |

Increasing a threshold can reject noise, but can also reject valid features. These values balance detection sensitivity and false detections.

**`centre_of_quad(corners)` — area centroid**

See [the function](./Software/workspace/src/task_1a/task_1a/camera_detection.py:118).

Its input is an array containing the four polygon vertices:

```text
[[x0, y0],
 [x1, y1],
 [x2, y2],
 [x3, y3]]
```

Its intended output is the polygon’s area centroid:

\[
c_x=\frac{M_{10}}{M_{00}},\qquad
c_y=\frac{M_{01}}{M_{00}}
\]

Here:

- \(M_{00}\) represents area.
- \(M_{10}\) and \(M_{01}\) represent first moments.
- `cv2.moments()` computes these values.

This is the centroid calculation documented by [OpenCV](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html).

**Why not average the four corners?** The average weights every vertex equally. The area centroid weights the enclosed region. A trapezoid generally has more area near its longer parallel side, so the two calculations generally differ.

A robust implementation must handle \(M_{00}\approx0\), because a degenerate polygon has no meaningful area centroid.

**Current state:** the function always returns `(0.0, 0.0)`.

**`find_trapezoids(frame)` — the intended detection algorithm**

See [the function](./Software/workspace/src/task_1a/task_1a/camera_detection.py:156).

It should return:

```python
binary, trapezoids
```

`binary` should be a single-channel image containing only white trapezoid outlines. Each entry in `trapezoids` should contain:

```python
(centre_x, centre_y, corners)
```

The comments prescribe the following algorithm.

**Step 1: Crop the arena**

All initial processing happens in the arena crop.

A point detected at crop coordinates \((u,v)\) must eventually be converted back to full-image coordinates:

\[
u_{\text{full}}=u+304,\qquad
v_{\text{full}}=v+24
\]

Forgetting this offset would give incorrect overlays and incorrect service requests.

**Step 2: Estimate the floor colour in LAB**

The checkpoints have different colours: cyan, green, and orange. A single narrow hue range cannot detect all three.

The suggested approach is:

1. Convert the crop from BGR to LAB.
2. Find the most common value in each LAB channel.
3. Treat that triplet as the estimated floor colour.
4. Calculate each pixel’s colour distance from it.

Conceptually:

\[
D=\sqrt{(L-L_f)^2+(a-a_f)^2+(b-b_f)^2}
\]

Pixels sufficiently far from the floor colour become foreground features.

LAB separates lightness from colour components and provides a useful space for this distance-based comparison.

In an implementation, convert `uint8` values to a signed or floating-point type before subtraction to avoid unsigned arithmetic overflow.

**Step 3: Apply morphological closing**

Closing means:

```text
Dilation → Erosion
```

It helps join small breaks in thin outlines. Opening performs the reverse sequence and is useful for removing small foreground noise. These operations are described in [OpenCV’s morphology documentation](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html).

**Step 4: Detect edges and line segments**

Canny identifies boundaries.

The probabilistic Hough transform then identifies straight line segments, represented by endpoints:

```python
(x1, y1, x2, y2)
```

Using straight lines helps distinguish the desired polygonal borders from texture and curved markers.

The code must handle a `None` result when no lines are found.

**Step 5: Construct temporary closed shapes**

Detected lines should be drawn on a blank scratch image with enough thickness to connect the sides. Another closing operation helps form closed loops.

This scratch image is used to recover geometry. It is separate from the final binary output.

**Step 6: Find inner contours**

The suggested retrieval mode is:

```python
cv2.RETR_CCOMP
```

It gives a two-level contour hierarchy.

A closed outline has an enclosed hole. Selecting contours with a parent helps recover these inner enclosed regions.

Small regions are rejected using `MIN_TRAPEZOID_AREA`.

**Step 7: Approximate contours into polygons**

`cv2.approxPolyDP()` simplifies the contour into a polygon.

The suggested tolerance is approximately:

\[
\epsilon=0.03\times\text{perimeter}
\]

Keep polygons that are:

- Convex.
- Exactly four-sided.
- Large enough.

The tolerance matters: too small retains extra vertices; too large can distort the shape.

**Step 8: Check opposite-side parallelism**

This is the main distinction between a checkpoint and the nearby rectangular vault.

For this task’s definition:

| Shape | Parallel opposite-side pairs |
|---|---:|
| Trapezoid | Exactly 1 |
| Rectangle | 2 |
| General quadrilateral | Usually 0 |

For each side, calculate its direction using `atan2(dy, dx)`.

Parallelism must be compared modulo \(180^\circ\), because a line has the same orientation when its direction is reversed. For example, \(179^\circ\) and \(1^\circ\) differ by only \(2^\circ\) as line orientations.

**Step 9: Restore full-image coordinates and calculate centroids**

Add the crop offset to accepted vertices, then call `centre_of_quad()`.

**Step 10: Create the final binary output**

Create an image with:

```python
np.zeros(frame.shape[:2], np.uint8)
```

Draw only the accepted polygon borders using intensity `255`.

The result must have:

```text
Shape:       (720, 1280)
Channels:    1
Background:  0
Outlines:    255
```

Drawing with antialiasing can introduce intermediate grey values, so it should be avoided for a strictly binary result.

**Current state:** this function returns an entirely black image and an empty detection list.

**`main()` — camera and ROS lifecycle**

See [the main function](./Software/workspace/src/task_1a/task_1a/camera_detection.py:254).

The implemented framework:

1. Initializes ROS with `rclpy.init()`.
2. Creates a node named `camera_feed`.
3. Creates a `PixelToWorld` client.
4. Waits up to 10 seconds for the service.
5. Opens the MJPEG stream.
6. Reads frames continuously.
7. Calls `find_trapezoids()`.
8. Draws detection overlays.
9. Displays the camera and binary windows.
10. Exits on `q` and releases resources.

The executable name is **`camera_detection`**, while the ROS node name is **`camera_feed`**. An executable name and a node name can differ.

FPS is calculated from recent timestamps:

\[
FPS=\frac{N-1}{t_{\text{last}}-t_{\text{first}}}
\]

`time.monotonic()` is suitable for measuring elapsed time because it does not jump when the system clock changes.

Coordinate reporting should happen every 0.5 seconds. Detections are sorted by vertical position and then horizontal position to keep their printed order stable.

The service block should:

1. Create `PixelToWorld.Request()`.
2. Fill `pixel_x` and `pixel_y` as floating-point values.
3. Use `call_async()`.
4. Spin until completion with a timeout.
5. Handle timeout, unsuccessful response, or successful coordinates.

**Current state:** this block contains only `pass`.

Although world-coordinate reporting is described as optional in the README, the current framework still requires the service to be available before opening the camera.

---

**2. [Task 1A: `PixelToWorld.srv`](./Software/workspace/src/shape_interface/srv/PixelToWorld.srv)**

This defines the communication contract between the detection node and the coordinate-conversion server.

```text
Request:
  float64 pixel_x
  float64 pixel_y

Response:
  bool success
  string message
  float64 world_x
  float64 world_y
```

The `---` in a `.srv` file separates request fields from response fields.

A service suits this operation because a particular pixel is submitted and a corresponding result is requested.

The world frame uses:

- Origin at the arena’s top-left corner.
- Positive \(x\) to the right.
- Positive \(y\) downward.
- Distances in metres.

For intuition, using the hardcoded arena crop gives the approximate mapping:

\[
x\approx\frac{u-304}{671}\times2.4384
\]

\[
y\approx\frac{v-24}{671}\times2.4384
\]

The supplied service uses camera calibration parameters; the formula above explains the coordinate relationship rather than reproducing its unavailable source.

---

**3. [Task 1A: `bin/pixel_to_world_service`](./Software/workspace/src/task_1a/bin/pixel_to_world_service)**

This is a precompiled native executable. Its C++ source is absent from this checkout.

Its role is to:

- Advertise the `pixel_to_world` service.
- Receive image coordinates.
- Convert them into arena coordinates.
- Return success/status information.

The launch file supplies `image_width` and `image_height`. Its comments explain that the pixel-to-metre scale depends on image height and camera field of view.

**Viva point:** the client detects the location in image space; the server handles calibration and conversion into physical units.

---

**4. [Task 1A: `CMakeLists.txt`](./Software/workspace/src/task_1a/CMakeLists.txt)**

This file tells the ROS build system how to build and install the package.

Its important operations are:

- `project(task_1a)` names the package.
- `find_package(ament_cmake REQUIRED)` enables the CMake-based ROS build system.
- `find_package(ament_cmake_python REQUIRED)` supports installing the Python package.
- If the C++ service source exists, it builds `pixel_to_world_service`.
- Otherwise, it installs the supplied binary.
- `ament_python_install_package()` installs the Python module directory.
- `install(PROGRAMS ...)` installs `camera_detection.py` under the executable name `camera_detection`.
- `ament_package()` completes package registration.

That installation rule enables:

```bash
ros2 run task_1a camera_detection
```

The file supports both source-based and precompiled distribution of the service.

---

**5. [Task 1A: `package.xml`](./Software/workspace/src/task_1a/package.xml)**

This is the package manifest. It declares the package’s identity and dependencies.

Important dependencies:

| Dependency | Purpose |
|---|---|
| `ament_cmake` | Build system |
| `ament_cmake_python` | Python package installation |
| `rclcpp` | C++ ROS support for the service |
| `shape_interface` | Custom service definition |
| `rclpy` | Python ROS client |
| `python3-opencv` | Vision operations |
| `python3-numpy` | Numerical arrays |

**Difference from CMake:** `package.xml` declares what the package needs; `CMakeLists.txt` specifies how its components are built and installed.

---

**6. [Task 1A: `__init__.py`](./Software/workspace/src/task_1a/task_1a/__init__.py) and [ReadMe.md](./Software/workspace/src/task_1a/ReadMe.md)**

`__init__.py` marks the directory as a Python package. In this checkout, it contains only copyright comments.

The README defines the required output and run procedure. The central requirement is **three white trapezoid outlines on a black, single-channel `1280 × 720` image**.

When counting outlined shapes, retrieval mode matters: one closed outline can produce both an outer and an inner contour. Counting external contours is a useful way to check the number of separate outlined objects.

---

**7. [Task 2B: `path_planner.py`](./Software/workspace/src/task_2b/task_2b/path_planner.py)**

This node turns a camera image into a safe, timed route for `r2`.

```mermaid
flowchart LR
    A[Camera frame] --> B[Raw occupancy]
    B --> C[Remove drivable markings]
    C --> D[Inflate obstacles]
    D --> E[A* and path simplification]
    E --> F[Timed /r2/plan]
    O[/r2/odom] --> E
```

It plans once. It does not continually update the map or replan around moving obstacles.

**Arena and planning constants**

| Parameter | Value | Purpose |
|---|---:|---|
| `ARENA_SIZE` | 2.4384 m | Arena side length |
| `ROBOT_RADIUS` | 0.084 m | Physical footprint radius |
| `SAFETY_MARGIN` | 0.030 m | Extra obstacle clearance |
| `GRID_CELLS` | 244 | Fine occupancy grid dimension |
| `CELL_SIZE` | About 0.01 m | Fine grid resolution |
| `PLAN_CELL` | 0.04 m | Coarser planning resolution |
| `PLAN_SPEED` | 0.24 m/s | Speed used to assign timestamps |
| `GOAL_STOP` | 2 s | Encoded pause at each goal |
| `START_DELAY` | 2 s | Delay between publishing and departure |

The goals are visited in this exact order:

```text
1. (1.2192, 0.4992)
2. (0.3592, 1.5592)
3. (2.0792, 0.6992)
```

**`read_frame()`**

Opens the camera stream, reads one frame, releases the camera handle, and raises an error if acquisition fails.

A single snapshot is sufficient for the intended static arena.

**`crop_arena()`**

Extracts the `671 × 671` arena region using the known pixel bounds.

This ensures that the image corresponds to a square of side `2.4384 m`.

**`frame_to_occupancy()`**

See [the raw occupancy function](./Software/workspace/src/task_2b/task_2b/path_planner.py:110).

Its output is a Boolean NumPy grid:

```python
occupied[row, col]
```

where:

```text
True  = obstacle
False = free
```

Processing:

1. Apply a `5 × 5` median blur.
2. Convert BGR to HSV.
3. Threshold the sand colour.
4. Invert the result so non-sand becomes occupied.
5. Apply morphological opening.
6. Find external contours.
7. Fill their interiors.
8. Resize to `244 × 244`.
9. Treat nonzero resized values as occupied.

HSV separates hue, saturation, and brightness, making the known sand colour easier to threshold.

However, **non-sand does not necessarily mean a physical obstacle**. Painted checkpoints, start rings, goals, and the robot also become occupied. The next function resolves this ambiguity.

**`clear_circle()`**

Sets cells inside a circular world-space region to free.

The fine cell centres are calculated as:

\[
x=(col+0.5)\,CELL\_SIZE,\qquad
y=(row+0.5)\,CELL\_SIZE
\]

A cell is cleared when:

\[
(x-x_c)^2+(y-y_c)^2\le r^2
\]

It modifies the supplied grid in place.

**`remove_drivable()`**

See [the cleanup implementation](./Software/workspace/src/task_2b/task_2b/path_planner.py:131).

This is the main perception contribution in Task 2B. It distinguishes painted floor from physical obstacles.

Its logic has several parts.

**Detect coloured artwork.** It identifies sufficiently saturated and visible pixels whose hue/saturation differs from ordinary sand:

```python
s >= 70
v >= 45
(h < 10) or (h > 30) or (s >= 135)
```

It closes gaps, extracts contours, and fills substantial coloured regions into a `drivable` mask.

**Preserve payloads on coloured pads.** A coloured pad may be drivable while its central payload is solid.

The code identifies approximately circular pads using:

\[
\text{circularity}=\frac{4\pi A}{P^2}
\]

and:

\[
\text{fill ratio}=\frac{A}{\pi r^2}
\]

It considers pads with enclosing-circle radii between `0.12 m` and `0.23 m`, circularity at least `0.65`, and fill ratio at least `0.70`.

For these pads, it protects a central payload region of radius `0.095 m`.

**Recover grey solid objects.** Grey pillars are detected using:

```python
s <= 65
v <= 205
```

Opening removes thin grey details while preserving substantial objects.

Small grey highlights entirely inside artwork can be ignored. Grey components entirely inside known robot/start footprints can also be excluded from the solid-object mask.

Checking whole components helps avoid erasing only part of a nearby pillar.

**Combine the masks conservatively.**

- Expand the artwork mask slightly to cover border pixels.
- Remove detected solids from the artwork mask.
- Resize both masks to the fine grid.
- Free artwork cells.
- Remove small residual fringes.
- Restore detected solid cells.

**Clear known markers and the robot.**

The clearing radii are:

| Object | Clearing radius |
|---|---:|
| Start circles | \(0.084+0.025=0.109\) m |
| Robot image | 0.109 m |
| Goal markers | \(0.084+0.030+0.040=0.154\) m |

Finally, detected solids are restored again, with an exception for the robot’s measured footprint.

**Why this matters:** blindly clearing a circle around a goal could erase part of a nearby pillar. The protected solid mask prevents that.

**`planning_grid()`**

See [the inflation and grid conversion](./Software/workspace/src/task_2b/task_2b/path_planner.py:240).

It expands obstacles by:

\[
0.084+0.030=0.114\text{ m}
\]

This is obstacle inflation.

Instead of checking a circular robot against every obstacle, the planner checks the robot’s centre against expanded obstacles. This is a configuration-space approach.

Inflation uses dilation with an approximately circular kernel. With the fine-grid resolution, the expansion radius rounds up to 12 cells.

The function samples the inflated map at coarse cell centres, giving a `61 × 61` planning grid. Cells close to the arena boundary are also blocked.

**Critical indexing difference:**

```python
occupied[y, x]   # Fine image-style grid
blocked[x, y]    # Planning grid
```

The transpose in this function deliberately changes the convention.

**`to_cell()` and `to_world()`**

`to_cell()` converts metres into coarse grid indices:

\[
i=\left\lfloor x/0.04\right\rfloor,\qquad
j=\left\lfloor y/0.04\right\rfloor
\]

It clamps indices into the grid.

`to_world()` returns a cell’s centre:

\[
x=(i+0.5)\times0.04,\qquad
y=(j+0.5)\times0.04
\]

The half-cell offset represents the cell centre.

**`nearest_free()`**

Finds a nearby free cell when the cell containing a point is occupied.

It searches a bounded neighbourhood and chooses the candidate whose centre is closest in Euclidean distance.

However, `plan_route()` first checks the actual start and goals for collision. **This implementation does not silently replace an obstructed required goal with a nearby destination.**

**`segment_clear()`**

See [the collision check](./Software/workspace/src/task_2b/task_2b/path_planner.py:283).

This checks whether an entire straight segment is collision-free.

It:

1. Rejects invalid coordinates.
2. Rejects endpoints outside the permitted wall clearance.
3. Finds blocked cells near the segment.
4. Checks segment intersection with their closed square boundaries.

The segment is parameterized as:

\[
p(t)=a+t(b-a),\qquad0\le t\le1
\]

For each blocked square, the code finds the intervals of \(t\) that fall inside its horizontal and vertical bounds. An overlapping interval indicates collision.

**Touching an obstacle boundary also counts as collision.** This is more reliable than checking only a few sampled points along the line.

**`astar()`**

See [the A* implementation](./Software/workspace/src/task_2b/task_2b/path_planner.py:322).

A* prioritizes cells using:

\[
f(n)=g(n)+h(n)
\]

- \(g(n)\): accumulated cost from the start.
- \(h(n)\): estimated remaining cost.

It uses eight-connected movement:

- Horizontal/vertical cost: \(1\).
- Diagonal cost: \(\sqrt2\).

Its heuristic is octile distance:

\[
h=\max(dx,dy)+(\sqrt2-1)\min(dx,dy)
\]

This matches the movement model.

`heapq` implements the priority queue. Dictionaries store the best known costs and each cell’s parent.

When the goal is reached, parent links reconstruct the path.

The code rejects diagonal movement if either adjacent orthogonal cell is blocked. This prevents cutting through obstacle corners.

**Viva qualification:** A* finds a shortest route under this grid’s movement costs and restrictions. That does not imply the final route is the globally shortest continuous-space route.

**`simplify_path()`**

A* produces a sequence of cell centres, which can look like a staircase.

This function greedily connects the current point to the farthest later point that passes `segment_clear()`.

It reduces unnecessary waypoints while preserving checked obstacle clearance. It produces straight segments rather than a smooth spline.

**`plan_route()`**

See [the route construction](./Software/workspace/src/task_2b/task_2b/path_planner.py:380).

It plans:

```text
Actual start → Goal 1 → Goal 2 → Goal 3
```

For each leg:

1. Validate the exact destination.
2. Determine start and goal cells.
3. Run A*.
4. Convert cells into world coordinates.
5. Include exact start/goal positions.
6. Simplify the route.
7. Assign timestamps.
8. Append a duplicate goal position with a later timestamp.

Travel time is assigned using:

\[
\Delta t=\frac{\text{segment length}}{0.24}
\]

For example, a `0.48 m` segment takes `2 s` in the planned schedule.

A pause is encoded as:

```python
(5.0, goal_x, goal_y)
(7.0, goal_x, goal_y)
```

Same position, increasing time means remain at that point.

**`PathPlanner` node**

See [the ROS node](./Software/workspace/src/task_2b/task_2b/path_planner.py:425).

Its name is:

```text
task_2b_path_planner
```

It subscribes to `/r2/odom` for the actual starting position and publishes `/r2/plan`.

Its timer runs every `0.2 s`. Once odometry is available, it captures the image and plans once.

`to_path_msg()` converts route tuples into `nav_msgs/Path`:

- Frame: `odom`.
- Path header timestamp: departure time.
- Pose timestamp: departure time plus relative route time.
- Position: world coordinates.
- Orientation: valid identity quaternion, `w = 1`.

The follower uses positions and timestamps; it maintains its own yaw target.

The publisher uses:

```python
QoSProfile(depth=1, durability=TRANSIENT_LOCAL)
```

This retains the latest plan for a compatible subscriber that starts later. The planner remains spinning so that the retained plan remains available.

A late follower receives the original timestamps; the plan is not automatically rescheduled.

---

**8. [Task 2B: `path_follower.py`](./Software/workspace/src/task_2b/task_2b/path_follower.py)**

The follower converts the timed route into wheel commands using odometry feedback.

```text
Timed path + Current odometry
  → Select target position
  → Position and yaw PID
  → Transform arena velocity to body velocity
  → Inverse kinematics
  → Limit wheel speeds
  → Publish wheel commands
```

**Robot constants and inverse kinematics**

The code uses:

```python
WHEEL_RADIUS_M = 0.0255
CHASSIS_RADIUS_M = 0.06412
WHEEL_ANGLES_RAD = radians([30, 150, 270])
```

For each wheel:

\[
\omega_i=
\frac{\cos\alpha_i\,v_x+\sin\alpha_i\,v_y+R\omega_z}{r}
\]

The matrix form is:

\[
\begin{bmatrix}
\omega_1\\
\omega_2\\
\omega_3
\end{bmatrix}
=
\frac1r
\begin{bmatrix}
\cos30^\circ&\sin30^\circ&R\\
\cos150^\circ&\sin150^\circ&R\\
\cos270^\circ&\sin270^\circ&R
\end{bmatrix}
\begin{bmatrix}
v_x\\v_y\\\omega_z
\end{bmatrix}
\]

The resulting wheel order is left, right, back.

**Do not confuse these radii:**

- `0.0255 m`: wheel radius.
- `0.06412 m`: wheel placement radius used in kinematics.
- `0.084 m`: robot collision-footprint radius used by the planner.

**`body_velocity_to_wheel_speeds()`**

Multiplies the inverse-kinematics matrix by the requested body twist:

```python
IK_MATRIX @ [vx, vy, wz]
```

Inputs are in `m/s`, `m/s`, and `rad/s`. Outputs are wheel angular speeds in `rad/s`.

**`body_to_wheels()`**

Limits wheel speeds to a magnitude of `30 rad/s`.

If one wheel exceeds the limit, **all wheel speeds are scaled by the same factor**:

\[
\omega_i'=\omega_i
\frac{30}{\max_j|\omega_j|}
\]

Uniform scaling preserves the ratios between wheel speeds and therefore preserves the commanded motion proportions.

**`yaw_from_quat()`**

Converts the odometry orientation quaternion into planar yaw:

\[
\psi=
\operatorname{atan2}
\left(
2(wz+xy),\
w^2+x^2-y^2-z^2
\right)
\]

The robot moves on a plane, so yaw is the orientation component needed by the controller.

**`wrap()`**

Normalizes angles into:

\[
(-\pi,\pi]
\]

For example, \(350^\circ\) becomes \(-10^\circ\).

This ensures the yaw controller uses the shortest angular error.

**`to_body()`**

Transforms arena-frame velocity into robot-frame velocity:

\[
v_{x,b}=\cos\psi\,v_{x,a}+\sin\psi\,v_{y,a}
\]

\[
v_{y,b}=-\sin\psi\,v_{x,a}+\cos\psi\,v_{y,a}
\]

The position controller calculates errors in the arena frame, while inverse kinematics requires velocities relative to the robot. This transformation connects them.

**`GoToPoint` — position and yaw PID**

See [the controller](./Software/workspace/src/task_2b/task_2b/path_follower.py:99).

It stores integral and previous-error state between updates.

For position:

\[
e=
\begin{bmatrix}
x_t-x\\y_t-y
\end{bmatrix}
\]

\[
I_k=I_{k-1}+e_k\Delta t
\]

\[
D_k=\frac{e_k-e_{k-1}}{\Delta t}
\]

\[
v=K_pe+K_iI+K_dD
\]

The gains are:

```text
Position: Kp = 2.0, Ki = 0.02, Kd = 0.05
Yaw:      Kp = 2.0, Ki = 0.02, Kd = 0.05
```

Their roles are:

- **P:** responds to current error.
- **I:** corrects persistent bias.
- **D:** responds to error changes and provides damping.

The position integral contribution is bounded to 20% of the translation speed cap. Integration is frozen when translation saturates. Yaw has analogous protection.

This is anti-windup: it prevents excessive integral accumulation while the actuator command is already limited.

When position error falls inside the controller tolerance, translation becomes zero and the position integral is cleared. Yaw correction can still operate.

Although the class defaults to `v_max=0.25`, `RobotNode` creates it with:

```python
GoToPoint(v_max=0.30, tol=0.012)
```

Therefore the actual follower uses:

- Translation cap: `0.30 m/s`.
- Angular cap: `0.6 rad/s`.
- Controller position tolerance: `12 mm`.
- Update rate: `50 Hz`, giving nominal \(\Delta t=0.02 s\).

The follower’s speed cap exceeds the planned `0.24 m/s`, giving it some ability to recover tracking lag.

**`path_to_traj()`**

Converts `nav_msgs/Path` back into:

```python
[(t, x, y), ...]
```

Relative time is recovered by subtracting the path header timestamp from each pose timestamp.

Nanoseconds are converted to seconds using \(10^{-9}\).

**`stops_in()`**

Detects consecutive identical positions whose timestamps differ by at least:

```python
MIN_STOP = 0.5
```

These become goal-stop intervals.

The final destination gets an infinite departure time, representing permanent parking.

**`position_at()`**

Finds the scheduled position at any time using linear interpolation:

\[
u=\frac{t-t_0}{t_1-t_0}
\]

\[
p(t)=p_0+u(p_1-p_0)
\]

If both endpoints are identical, the interpolated position stays constant, representing a pause.

**`RobotNode`**

See [the follower node](./Software/workspace/src/task_2b/task_2b/path_follower.py:209).

For `r2`, the node name is:

```text
task_2b_r2
```

Its communication is:

| Direction | Topic | Message |
|---|---|---|
| Subscribe | `/r2/odom` | `nav_msgs/Odometry` |
| Subscribe | `/r2/plan` | `nav_msgs/Path` |
| Publish | `/r2/wheel_commands` | `std_msgs/Float64MultiArray` |

`on_odom()` stores current position and yaw. The first yaw becomes `hold_yaw`, which the robot maintains throughout the route.

Because the robot is holonomic, it can translate sideways while keeping its orientation fixed.

`on_path()` accepts the first nonempty plan, extracts its trajectory, finds stops, and stores the departure time. Later plans are ignored by this implementation.

`send()` publishes the three wheel speeds.

**`tick()` — the control loop**

Runs every `20 ms`.

Its main decisions are:

1. If pose or plan is missing, send zero wheel speeds.
2. If departure time has not arrived, remain stopped.
3. Calculate elapsed trajectory time.
4. Determine the next goal-stop interval.
5. Check whether that goal has physically been reached.
6. Choose a target position.
7. Run the controller.
8. Publish wheel speeds.

During travel, the target is:

```python
position_at(traj, t + LEAD)
```

where:

```python
LEAD = 0.12
```

This is a **time lookahead of 0.12 seconds**, not a distance lookahead.

At the planned speed, it corresponds to about:

\[
0.24\times0.12=0.0288\text{ m}
\]

along a straight moving segment.

The arrival check uses:

```python
GOAL_TOLERANCE = 0.020
```

So a goal counts as reached within `20 mm`, while the controller’s own position tolerance is `12 mm`.

If the robot misses an intermediate goal’s scheduled departure time, `held` accumulates delay so the effective trajectory clock does not simply advance past the missed goal.

The timestamps encode two-second pauses. Actual stopping and departure behaviour also depends on feedback, arrival tolerance, and lookahead.

At the final goal, the node sends zeros and invokes the completion callback.

**`main()`**

Initializes ROS, constructs the robot node, adds it to an executor, and processes callbacks.

The arrival callback logs:

```text
task 2B complete
```

On shutdown, the program attempts to publish zero wheel speeds before destroying the nodes.

---

**9. [Task 2B: `CMakeLists.txt`](./Software/workspace/src/task_2b/CMakeLists.txt)**

This installs:

| Source file | Installed executable |
|---|---|
| `task_2b/path_planner.py` | `path_planner` |
| `task_2b/path_follower.py` | `path_follower` |
| `bin/task_2b_node` | `task_2b_node` |

That enables:

```bash
ros2 run task_2b path_planner
ros2 run task_2b path_follower
```

These Python files are installed directly as executable programs. This package does not use `setup.py` to create their entry points.

---

**10. [Task 2B: `package.xml`](./Software/workspace/src/task_2b/package.xml)**

This declares the package’s runtime dependencies:

| Dependency | Use |
|---|---|
| `rclpy` | Planner and follower ROS nodes |
| `nav_msgs` | `Odometry` and `Path` |
| `geometry_msgs` | `PoseStamped` |
| `std_msgs` | Wheel-command array |
| `python3-numpy` | Grids, matrices, controller vectors |
| `python3-opencv` | Camera processing and morphology |
| `rclcpp` | Native executable support |

The manifest declares an `ament_cmake` build type.

---

**11. [Task 2B: `bin/task_2b_node`](./Software/workspace/src/task_2b/bin/task_2b_node) and [ReadMe.md](./Software/workspace/src/task_2b/ReadMe.md)**

`task_2b_node` is a supplied compiled executable without accompanying source.

It is installed by CMake, but **neither the supplied Task 2B launch file nor the documented planner/follower procedure starts it**. The navigation implementation you can explain from available source is the two Python nodes.

The README defines:

- The three-goal navigation objective.
- Obstacle avoidance requirements.
- Timestamp-based route timing.
- Duplicate goal poses to encode pauses.
- Transient-local delivery of the plan.
- Launch and execution commands.

---

**12. Shared launch files: [task1a.launch.py](./Software/workspace/src/hb_description/launch/task1a.launch.py) and [task2b.launch.py](./Software/workspace/src/hb_description/launch/task2b.launch.py)**

These configure and start the simulation infrastructure.

| Component | Task 1A executable | Task 2B executable | Purpose |
|---|---|---|---|
| Physics/ROS bridge | `task1a_bridge` | `task2b_bridge` | Simulation and robot interface |
| Camera renderer | `task1a_camera` | `task2b_camera` | Overhead image rendering |
| HTTP stream server | `camera_stream` | `camera_stream` | MJPEG feed |
| Coordinate service | `pixel_to_world_service` | `pixel_to_world_service` | Pixel conversion |

They do not automatically start your detection, planner, or follower scripts.

Important launch arguments include:

| Argument | Default |
|---|---|
| `renderer` | `cpu` |
| `viewer_fps` | 60 |
| `camera_name` | `overhead_camera` |
| `camera_width` | 1280 |
| `camera_height` | 720 |
| `camera_fps` | 30 |
| `stream_port` | 8080 |
| `stream_quality` | 90 |
| `stream_bind` | `127.0.0.1` |

There are also switches for the camera, stream, scenery, and pixel service.

The launch APIs mean:

- `DeclareLaunchArgument`: exposes a configurable option.
- `LaunchConfiguration`: reads its supplied value.
- `ParameterValue`: converts it into the required type.
- `IfCondition`: conditionally starts a component.
- `Node`: describes an executable to launch.

The Task 1A launch comments explain that the camera receives simulation poses through shared memory and writes frames into shared memory. `camera_stream` then serves those frames over HTTP.

**Task 2B does not call the pixel service.** Its planner uses the known crop and arena dimensions for image-to-grid conversion.

---

**13. Shared simulation binaries and libraries**

The relevant files under `hb_description/bin/` are:

- `task1a_bridge`: Task 1A simulation/ROS bridge.
- `task1a_camera`: Task 1A overhead renderer.
- `task2b_bridge`: Task 2B simulation/ROS bridge.
- `task2b_camera`: Task 2B overhead renderer.
- `camera_stream`: shared MJPEG server.

They are precompiled executables, so their internal algorithms cannot be explained line by line from this checkout.

Under `hb_description/lib/`:

- `libmujoco.so.3.4.0` supplies the MuJoCo simulation runtime.
- The `libhb_source__rosidl_*` libraries provide generated ROS interface/type support, including C, C++, Python, Fast RTPS, and introspection support.

These are runtime infrastructure, separate from your perception and navigation algorithms.

---

**14. Shared build files: [hb_description/CMakeLists.txt](./Software/workspace/src/hb_description/CMakeLists.txt) and [hb_description/package.xml](./Software/workspace/src/hb_description/package.xml)**

The CMake file installs:

- Simulation executables.
- Shared libraries.
- Launch files.

It also contains interface-generation entries for payload services associated with later tasks.

The manifest declares simulation and launch dependencies, including `rclcpp`, standard ROS messages, `launch`, `launch_ros`, and `task_1a`.

Despite its name, this checkout distributes the robot/arena simulation as compiled binaries rather than editable robot model files.

---

**15. Shared interface package**

[shape_interface/CMakeLists.txt](./Software/workspace/src/shape_interface/CMakeLists.txt) uses `rosidl_generate_interfaces()` to generate language bindings from:

```text
srv/PixelToWorld.srv
srv/GetShape.srv
msg/Shape.msg
```

This generation makes imports such as the following possible:

```python
from shape_interface.srv import PixelToWorld
```

[shape_interface/package.xml](./Software/workspace/src/shape_interface/package.xml) declares the interface generators, runtime dependency, and membership in `rosidl_interface_packages`.

The other definitions are:

- [Shape.msg](./Software/workspace/src/shape_interface/msg/Shape.msg): contains `shape_name` and a floating-point data array.
- [GetShape.srv](./Software/workspace/src/shape_interface/srv/GetShape.srv): an empty request with a response containing success, message, shape name, and data.

These two definitions are associated with shape-related work elsewhere in the repository. **Your Task 1A and Task 2B Python code does not use them.**

---

**16. Test files relevant to Task 2B**

[test_task_2.py](./Software/workspace/tests/test_task_2.py) checks the actual algorithm source while replacing ROS interfaces with test doubles.

Relevant checks cover:

- Inverse kinematics and uniform wheel saturation.
- Coordinate transformations and angle wrapping.
- Controller limits and yaw correction.
- Path-message timestamp conversion.
- Waiting before departure and stopping at the destination.
- A* corner-cutting prevention.
- Segment collision checks.
- Exact goal order and pause encoding.
- Preserving nearby pillars during image cleanup.
- Planner/follower integration with idealized robot dynamics.

[test_task_2_camera.py](./Software/workspace/tests/test_task_2_camera.py) checks perception and planning using captured arena images and synthetic nearby-pillar cases.

The fixture [task2b_start.jpg](./Software/workspace/tests/fixtures/task2b_start.jpg) is an initial camera image used by those regression checks.

I ran both test files: **13 shared Task 2A/2B checks and 2 shared Task 2B/2C camera checks passed.** These are offline checks; I did not run the live MuJoCo simulation.

---

For your viva, be ready to answer these implementation questions directly:

| Question | Answer grounded in your files |
|---|---|
| How is a trapezoid distinguished from a rectangle? | Four convex vertices, with exactly one parallel opposite-side pair under the task’s definition. |
| Why calculate an area centroid? | Averaging vertices generally differs from the centre of the enclosed area. |
| Why clean the raw occupancy grid? | Non-sand pixels include drivable paint, markers, and the robot itself. |
| Why inflate obstacles? | It allows planning the robot centre as a point while accounting for footprint and margin. |
| Which planner is implemented? | Eight-connected A* with an octile heuristic and no diagonal corner cutting. |
| How are zigzags removed? | Greedy line-of-sight shortcuts, checked against blocked-cell squares. |
| How are goal pauses represented? | Identical positions with later timestamps. |
| Why use two navigation nodes? | The planner creates the route once; the follower performs continuous feedback control. |
| What feedback drives the controller? | Position and orientation from `/r2/odom`. |
| Does the robot face every waypoint? | It holds its initial yaw and uses holonomic translation. |
| Does this use SLAM or Nav2? | No. It uses a calibrated overhead image, custom grid planning, and a custom PID follower. |
| Does it avoid newly moving obstacles? | No. The current planner uses one snapshot and does not replan dynamically. |
