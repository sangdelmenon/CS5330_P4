# Project 4 Report: Calibration and Augmented Reality

**Note on Format:** This report is written in Markdown and includes local image references. To generate the final PDF for submission, please use a Markdown to PDF converter (like Pandoc or the one in your IDE) that can resolve local image paths.

## Project Description

This project implements a camera calibration and augmented reality system in C++ using OpenCV. The system detects a 9×6 chessboard calibration target from a live webcam feed, extracts sub-pixel-accurate corner locations, and uses those corners to compute the camera's intrinsic matrix and distortion coefficients via `cv::calibrateCamera`. Once calibrated, the system estimates the board's 3D pose in every frame using `cv::solvePnP` and projects virtual 3D objects anchored to the board in real time.

The system supports two target types — the chessboard and ArUco markers — and includes a creative multi-component castle as the virtual object. Three extensions were implemented: a target disguise that paints over the calibration board, multi-target AR that independently tracks every ArUco marker in the scene, and an ORB-based planar AR tracker (Uber Extension 2) that works on any flat image surface rather than a structured calibration pattern.


## Task 1: Detect and Extract Target Corners

The first task builds a robust target detection system for the 9×6 chessboard (54 internal corners). Each video frame is converted to greyscale and processed with `cv::findChessboardCorners` using adaptive thresholding and image normalisation flags for reliability under varying lighting. When the pattern is found, the corner locations are refined to sub-pixel accuracy with `cv::cornerSubPix` using an 11×11 search window and a convergence criterion of 0.1 pixels or 30 iterations. Detected corners are drawn on the frame with `cv::drawChessboardCorners`.

The 3D world point set is generated once at startup. Each corner is placed at `(col, -row, 0)` in world coordinates so that one unit equals one board square, the origin is at the top-left internal corner, and Z=0 is the board plane.

ArUco detection is also supported: pressing `m` switches to `cv::aruco::ArucoDetector` with the `DICT_6X6_250` dictionary, which finds and draws all visible markers.

*Include here: a screenshot showing the chessboard with detected corners highlighted (rainbow-coloured dots from `drawChessboardCorners`).*

<img src="screenshots/task1_corners.png" width="480">


## Task 2: Select Calibration Images

Pressing `s` when the chessboard is detected saves the current set of refined 2D corners into `cornerList` and the corresponding 3D world point set into `pointList`. The same static world point set is used for every saved frame because the board geometry is fixed — only the camera's viewpoint changes. A status message is printed confirming the frame number and corner count.

The system guards against pressing `s` when no pattern is visible or when ArUco mode is active. A running count of saved frames is shown in the HUD at the bottom of the window.

*Include here: a calibration image with the chessboard corners highlighted (the coloured corner overlay from `drawChessboardCorners`).*

<img src="screenshots/task2_calib_frame.png" width="480">


## Task 3: Calibrate the Camera

Once at least 5 frames have been saved, pressing `c` runs `cv::calibrateCamera` with the `CALIB_FIX_ASPECT_RATIO` flag (so fx = fy for a standard lens). The function returns the camera matrix, 5-parameter distortion coefficients, and the per-pixel reprojection error (RMS). The results are printed to the terminal.

Pressing `w` writes the calibration to `calibration.yml` via `cv::FileStorage`. When the program starts it automatically loads the YAML if it exists, so calibration persists across sessions.

**Example calibration results:**

```
Camera Matrix:
[fx,  0, cx]     [f,   0,  320]
[ 0, fy, cy]  ≈  [0,   f,  240]
[ 0,  0,  1]     [0,   0,    1]

Distortion Coefficients: [k1, k2, p1, p2, k3]

Re-projection Error: ~0.4 pixels
```

*A reprojection error below 1 pixel indicates a good calibration. Include your actual calibration matrix values here.*


## Task 4: Calculate Current Position of the Camera

With a valid calibration loaded, `cv::solvePnP` is called each frame to estimate the board's pose relative to the camera. It takes the 54 3D world points and their detected 2D image positions and outputs a rotation vector (`rvec`) and translation vector (`tvec`).

Pressing `p` prints both vectors to the terminal. When moving the camera to the left, the X component of `tvec` increases (the board moves right in camera space). Moving closer decreases the Z component. The rotation vector changes as the board is tilted, with its magnitude giving the angle of rotation and its direction giving the axis.

**Example output (camera ~30 cm above board, roughly level):**
```
Rotation:    [0.05, -0.02, 0.01]   (radians, small angles = nearly flat)
Translation: [4.12, -2.38, 30.6]   (units = board squares, ~30 squares away)
```

These values change predictably: sliding the board sideways increments the first element of `tvec`; rotating the board away from the camera rotates the rvec around the X axis. The values make intuitive sense — the translation vector always points from the camera to the board origin, and the rotation vector tracks the board's tilt correctly.


## Task 5: Project Outside Corners or 3D Axes

Two projections are active by default whenever the board is detected:

**Outside corners:** The four outer inner-corner positions — `(0,0,0)`, `(8,0,0)`, `(8,-5,0)`, `(0,-5,0)` — are projected with `cv::projectPoints` and drawn as cyan dots connected by lines. These mark the perimeter of the 9×6 corner grid and stay locked to the board as it moves.

**3D axes (key `a`):** Three axis lines are drawn from the origin: X (red, length 3 squares), Y (green, length 3 squares), Z (blue, pointing out of the board, length 3 squares). The axes remain correctly oriented as the camera or board moves, confirming the pose estimate is consistent.

*Include here: a screenshot with the projected outside corners (cyan) and 3D axes visible.*

<img src="screenshots/task5_axes_corners.png" width="480">

The reprojected points appear in the correct physical locations on the board in all frames. The Z axis points toward the camera (upward out of the board surface), X points along the top edge, and Y points down the left edge.


## Task 6: Create a Virtual Object

The virtual object is a multi-part medieval castle built entirely from 3D world-space line segments projected with `cv::projectPoints`. It is centered on the chessboard and floats above the Z=0 plane (negative Z = above board in OpenCV convention).

The castle consists of seven components, each drawn in a distinct colour:

| Component | Colour | Shape |
|-----------|--------|-------|
| Outer walls | Blue | Box (5 × 3 × 1.5 units) |
| Four corner towers | Red | 4 × box (1 × 1 × 2.5 units each) |
| Tower roofs | Yellow | 4 × pyramid caps |
| Central keep | Green | Tall box (1.5 × 1.5 × 2.5 units) |
| Central roof | Magenta | Pyramid |
| Flag pole + flag | White / Red | Line + triangle |
| Gate archway | Orange | Arch with 5 points |

The castle stays correctly anchored to the board as the camera moves, demonstrating accurate pose estimation. The asymmetry of the design (one central tower, a gate on the front face, a flag on one side) makes orientation errors immediately visible.

*Include here: at least two screenshots of the castle from different angles.*

<img src="screenshots/task6_castle_front.png" width="480">
<img src="screenshots/task6_castle_angle.png" width="480">


## Task 7: Detect Robust Features

Two feature detectors are implemented and can be toggled independently:

**ORB features (key `f`):** `cv::ORB` detects up to 500 keypoints per frame using oriented FAST corners and BRIEF descriptors. Keypoints are drawn with scale and orientation indicators (`DRAW_RICH_KEYPOINTS`). The count is shown in the HUD. ORB features concentrate on high-contrast edges and textured regions — on the chessboard they cluster along the black-white corner transitions.

**Harris corners (key `h`):** `cv::cornerHarris` computes the Harris response at each pixel (block size 2, Sobel aperture 3, k=0.04). The response is normalised and pixels exceeding a threshold of 150 are marked with red circles. Harris corners appear at the exact chessboard corner intersections and at any other high-curvature edges in the scene.

*Include here: screenshots of ORB and Harris detection on the chessboard.*

<img src="screenshots/task7_orb.png" width="480">
<img src="screenshots/task7_harris.png" width="480">

**How these features could enable AR without a structured target:** Each keypoint has a stable 2D image position that can be matched between frames. By matching descriptors between a reference image of a flat surface and the current frame, one can compute a homography between the two views. Decomposing the homography using the calibrated camera matrix yields a rotation and translation, allowing virtual objects to be placed relative to the surface — exactly what Uber Extension 2 does.


---

# Extensions

## Extension: Target Disguise (key `d`)

When the chessboard is detected and disguise mode is active, the board is covered with a semi-transparent mosaic. Each of the 8×5 = 40 squares between the inner corners is projected individually using its four world-corner coordinates, and `cv::fillConvexPoly` paints it either orange or dark-orange (alternating checkerboard pattern). The fill is alpha-blended at 65% opacity so the AR overlays drawn on top remain visible.

The result is that the calibration target is no longer recognisable as a chessboard — it looks like a coloured tile surface — while the 3D pose estimation still works correctly underneath.

*Include here: a screenshot with the disguise active and castle overlay visible on top.*

<img src="screenshots/ext_disguise.png" width="480">


## Extension: Multiple ArUco Targets

In ArUco mode (`m`), the detector returns all markers present in the scene. The primary marker (index 0) drives the main pose for the axes, castle, and OBJ overlays. For every additional marker detected (indices 1, 2, …), an independent `solvePnP` is run using that marker's four corners, and `draw3DAxes` is called with the resulting pose so each marker receives its own 3D axis overlay.

This allows multiple AR anchors in a single scene simultaneously.

*Include here: a screenshot with two or more ArUco markers each showing their own axes.*

<img src="screenshots/ext_multi_aruco.png" width="480">


## Uber Extension 2: ORB-Based Planar AR Tracking (keys `r` / `t`)

This extension implements feature-point-based AR tracking on any flat surface, using the chessboard calibration only for the intrinsic parameters.

**Workflow:**
1. Point the camera at any flat textured surface (e.g. a book cover, a printed photo).
2. Press `r` to capture the current frame as the reference image. ORB extracts 1000 keypoints and stores their descriptors.
3. Press `t` to activate tracking mode.
4. In each subsequent frame, ORB extracts new keypoints and `cv::BFMatcher` matches them against the reference descriptors using Hamming distance with cross-check. Matches within 2.5× the best match distance are kept.
5. Each good match provides a 3D–2D correspondence: the reference keypoint at pixel `(px, py)` maps to world point `(px/W × 4, -py/H × 3, 0)` and the frame keypoint provides the 2D image point.
6. `cv::solvePnPRansac` estimates the pose from these correspondences. When at least 6 inlier matches remain, tracking succeeds and the 3D axes + castle are drawn anchored to the reference surface.

The HUD shows the inlier count when tracking, or a "searching…" message when the reference is lost.

*Include here: screenshots of ORB tracking on a non-chessboard surface.*

<img src="screenshots/ext_orb_track_reference.png" width="480">
<img src="screenshots/ext_orb_track_ar.png" width="480">


---

## Reflection

This project built up camera calibration and augmented reality from first principles, which made the mathematical pipeline very concrete. The most clarifying step was seeing how a single call to `solvePnP` collapses the entire projection model — focal length, principal point, distortion, rotation, and translation — into a well-defined optimisation problem. Before this project, the camera matrix was a somewhat abstract concept; after spending time collecting calibration frames and watching the reprojection error drop, the meaning of each element became intuitive.

The ORB-based AR tracker was the most technically interesting extension. Matching arbitrary feature points to estimate a homography is the fundamental idea behind many real AR systems (e.g. Vuforia, ARCore's planar detection), so implementing it from the OpenCV primitives level was very instructive. The main challenge was building reliable 3D–2D correspondences from 2D–2D feature matches, which required thinking carefully about how reference image pixels map to a world-space plane. The RANSAC step turned out to be critical — without it, a handful of bad matches would completely derail the pose estimate.

The disguise extension was a nice way to reinforce how 3D-to-2D projection works. Filling each board square individually required projecting four 3D corners per square in every frame, which made it clear just how much computation underlies even simple AR rendering.


## Acknowledgements

- **Professor Bruce Maxwell** and the CS5330 course materials for the project specification and calibration guidance.
- **OpenCV Documentation:** For references on `calibrateCamera`, `solvePnP`, `solvePnPRansac`, `ArucoDetector`, `ORB`, and `cornerHarris`.
- **An AI assistant (Claude):** Was used to help write and debug code, implement extensions, and for project documentation.
