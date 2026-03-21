# Project 4: Calibration and Augmented Reality

---

## Team Members

| Name | NUID | Program | Section |
|------|------|---------|---------|
| **Sangeeth Deleep Menon** | 002524579 | MSCS, Boston | CS5330 Section 03 (CRN 40669, Online) |
| **Raj Gupta** | 002068701 | MSCS, Boston | CS5330 Section 01 (CRN 38745, Online) |

---

## Project Description

For this project we built a complete camera calibration and augmented reality pipeline in C++ using OpenCV. The program detects a printed 9x6 chessboard, collects calibration frames from multiple angles, and computes the camera's intrinsic matrix and distortion coefficients. With that calibration in hand, it tracks the board's 3D pose in every frame and renders virtual objects that stay locked to the board no matter how the camera moves.

The virtual objects are two chess pieces, a pawn and a queen, each individually toggled. We also added support for ArUco markers as an alternative tracking target, and implemented an ORB feature tracker so that any flat textured surface can serve as an AR anchor without a printed board.

---

## Building the Project

The project requires CMake, a C++17 compiler, and OpenCV with the `calib3d`, `objdetect`, and `features2d` modules.

```bash
mkdir -p cmake-build-debug && cd cmake-build-debug
cmake ..
make
```

This produces two executables inside `cmake-build-debug/`:

| Executable | Description |
|------------|-------------|
| `Project4` | Command-line application. Supports webcam, video files, and images. |
| `Project4_GUI` | GUI application with an OpenCV sidebar for buttons and sliders. |

---

## Running the Applications

### Command-Line

```bash
# Webcam (default)
./cmake-build-debug/Project4

# Video or image file
./cmake-build-debug/Project4 path/to/video.mp4
```

### GUI

```bash
./cmake-build-debug/Project4_GUI
```

The GUI opens an OpenCV window with a sidebar panel on the right. Every feature is accessible through buttons and toggle switches so you never need to memorize key bindings.

---

## Key Bindings (Command-Line)

| Key | Action |
|-----|--------|
| `s` | Save current frame as a calibration image (chessboard mode only) |
| `c` | Run camera calibration (requires at least 5 saved frames) |
| `w` | Write calibration data to `calibration.yml` |
| `a` | Toggle 3D coordinate axes overlay |
| `v` | Toggle chess pawn |
| `b` | Toggle chess queen |
| `o` | Toggle OBJ model overlay |
| `p` | Print current rotation and translation vectors to terminal |
| `f` | Toggle ORB feature detection display |
| `h` | Toggle Harris corner detection display |
| `m` | Toggle ArUco marker mode |
| `d` | Toggle target disguise |
| `r` | Capture reference frame for ORB AR tracking |
| `t` | Toggle ORB AR tracking mode |
| `x` | Save screenshot |
| `q` / `ESC` | Quit |

---

## Implementation Summary

| Task | What We Did |
|------|-------------|
| 1. Target detection | `cv::findChessboardCorners` with `cornerSubPix` for sub-pixel accuracy; `cv::aruco::ArucoDetector` for ArUco mode |
| 2. Calibration frames | Press `s` to save corner sets and their 3D world points |
| 3. Camera calibration | `cv::calibrateCamera` with `CALIB_FIX_ASPECT_RATIO`; results saved and loaded via `cv::FileStorage` YAML |
| 4. Pose estimation | `cv::solvePnP` runs every frame using the stored intrinsics |
| 5. Axes and corners | Arrowed axes 5 squares long via `cv::arrowedLine`; outer board boundary via `cv::projectPoints` |
| 6. Chess pieces | Pawn (`v`) and Queen (`b`) built from stacked octagonal rings, 82% alpha fill + anti-aliased wireframe |
| 7. Robust features | ORB keypoints via `cv::ORB`; Harris corners via `cv::cornerHarris`, both shown live |
| Ext. Target disguise | Per-square projected quad fill with `cv::fillConvexPoly` and alpha blending |
| Ext. Multiple ArUco | Independent `solvePnP` and `draw3DAxes` for every detected marker |
| Ext. OBJ loader | Custom Wavefront OBJ parser rendered with `cv::line` and vertex dots |
| Ext. ORB tracking | 2000-keypoint ORB, BFMatcher cross-check, `solvePnPRansac` (reprojection error 5.0, min 12 inliers) |

---

## Project Files

| File | Description |
|------|-------------|
| `main.cpp` | CLI main loop, key handling, and AR state machine |
| `main_gui.cpp` | GUI application entry point |
| `gui_opencv.cpp` / `gui_opencv.h` | OpenCV sidebar rendering and button logic |
| `chessboarddetection.cpp` / `.h` | Chessboard corner detection, ArUco detection, and world point generation |
| `cameracalibration.cpp` / `.h` | Wrapper around `calibrateCamera` with YAML save and load |
| `augmentedreality.cpp` / `.h` | 3D axes, outside corners, chess pawn and queen rendering, and target disguise |
| `featuredetection.cpp` / `.h` | ORB feature detection and Harris corner detection |
| `orbtracking.cpp` / `.h` | ORB-based planar AR tracker |
| `modelloader.cpp` / `.h` | Wavefront OBJ file parser |
| `CMakeLists.txt` | Build configuration for both CLI and GUI targets |

---

## Extensions

**Target Disguise (`d`):** Press `d` while the chessboard is visible and every square gets painted over with a semi-transparent orange mosaic. Each square is filled individually using its projected 3D corners, so the disguise stays locked to the board while pose estimation keeps running underneath.

**Multiple ArUco Targets:** In ArUco mode the system detects every marker in the frame simultaneously. Each one gets its own independent pose estimate and its own set of 3D axes, so you can have multiple AR anchors active at the same time.

**OBJ Model Loader (`o`):** We wrote a custom parser that reads any Wavefront OBJ file and projects every face edge onto the board. The model is a low-poly house with a footprint of roughly 4 by 3.2 board squares. A small green dot is drawn at each projected vertex to make the wireframe easier to read.

**Uber Extension 2: ORB AR Tracking (`r` / `t`):** This lets you use any flat textured surface as an AR target without a printed board. Point the camera at the surface and press `r` to save a reference image, then press `t` to start tracking. ORB features are matched frame by frame using BFMatcher with cross-check, and `solvePnPRansac` estimates the pose. When at least 12 inliers are found, the 3D axes and both chess pieces appear anchored to that surface.

---

## Time Travel Days
1 day used.

---

## Demo Video
**Panopto:** https://northeastern.hosted.panopto.com/Panopto/Pages/Viewer.aspx?id=e25c0af1-604f-4f13-aee1-b4140037a390

---

## Acknowledgements
- OpenCV documentation for calibration, pose estimation, ArUco, and ORB references
- Course materials provided by Prof. Bruce Maxwell
- An AI assistant (Claude) was used to help write and debug code, and for project documentation
