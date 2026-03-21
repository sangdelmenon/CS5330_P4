# Project 4: Calibration and Augmented Reality

## Team Members
- **Sangeeth Deleep Menon** | NUID: 002524579 | MSCS - Boston | CS5330 Section 03 (CRN: 40669, Online)

## Project Description
This project implements a camera calibration and augmented reality (AR) system in C++ using OpenCV. The system detects a chessboard calibration target, computes the camera's intrinsic matrix and distortion coefficients, then uses the calibrated camera to estimate the board's pose in real time and project virtual 3D objects onto the scene. Two target types are supported: the standard 9×6 chessboard and ArUco markers. The system also supports ORB-based planar AR tracking (Uber Extension 2), which allows any flat surface to serve as an AR target by matching ORB feature descriptors against a captured reference frame.

## Building the Project
This project uses CMake and requires OpenCV (with `calib3d`, `objdetect`, `features2d` modules). A C++17 compatible compiler is required.
1. **Navigate to the project directory.**
2. **Create a build directory and run CMake and make:**
    ```bash
    mkdir -p cmake-build-debug && cd cmake-build-debug
    cmake ..
    make
    ```
    This creates two executables inside `cmake-build-debug/`: `Project4` (command-line) and `Project4_GUI` (OpenCV-based GUI).

## Running the Applications

### Command-Line Application
```bash
# Webcam mode (default)
./cmake-build-debug/Project4

# Pre-recorded video or image file
./cmake-build-debug/Project4 path/to/video.mp4
```

### GUI Application
```bash
./cmake-build-debug/Project4_GUI
```
The GUI provides sidebar controls for toggling all AR overlays, saving calibration frames, running calibration, and switching between chessboard and ArUco modes without using key presses.

### Key Bindings (Command-Line Application)
| Key | Action |
|-----|--------|
| `s` | Save current frame as calibration image (chessboard mode only) |
| `c` | Run camera calibration (requires ≥ 5 saved frames) |
| `w` | Write calibration (camera matrix + distortion) to `calibration.yml` |
| `a` | Toggle 3D coordinate axes overlay |
| `v` | Toggle virtual castle overlay |
| `o` | Toggle OBJ model overlay |
| `p` | Print current rotation vector and translation vector to terminal |
| `f` | Toggle ORB feature detection display |
| `h` | Toggle Harris corner detection display |
| `m` | Toggle ArUco marker mode (vs chessboard) |
| `d` | Toggle target disguise (paints over the chessboard) |
| `r` | Capture current frame as ORB AR tracking reference |
| `t` | Toggle ORB AR tracking mode (Uber Extension 2) |
| `x` | Save screenshot |
| `q` / `ESC` | Quit |

## Executable Files
1. **`Project4` (Command-Line Interface)**
   - Core application run from the terminal. Supports webcam and video/image file input. All calibration and AR operations are controlled via key presses.

2. **`Project4_GUI` (Graphical User Interface)**
   - Interactive OpenCV window with a sidebar panel. Provides buttons and sliders for all calibration and AR operations — no keyboard shortcuts required.

## Methods Overview
| Task | Implementation |
|------|---------------|
| 1. Target detection | `cv::findChessboardCorners` + `cornerSubPix`; `cv::aruco::ArucoDetector` for ArUco |
| 2. Calibration frame selection | User presses `s` to store corner set and corresponding 3D world points |
| 3. Camera calibration | `cv::calibrateCamera` with `CALIB_FIX_ASPECT_RATIO`; save/load via `cv::FileStorage` YAML |
| 4. Pose estimation | `cv::solvePnP` per frame using stored calibration intrinsics |
| 5. 3D axes / outside corners | `cv::projectPoints` for world-space axis endpoints and board corner positions |
| 6. Virtual castle | Multi-part wireframe object (walls, 4 towers, pyramidal roofs, keep, flagpole, gate) built from boxes and pyramids with 7 colours |
| 7. Robust features | ORB (`cv::ORB`) and Harris corners (`cv::cornerHarris`) displayed live |
| Ext. Target disguise | Per-square projected quad fill using `cv::fillConvexPoly` + alpha blend |
| Ext. Multiple ArUco AR | Independent `solvePnP` + `draw3DAxes` on every detected marker |
| Ext. OBJ model | Custom parser renders `Lowpoly_tree_sample2.obj` (low-poly house) via `cv::line` on detected faces |
| Ext. ORB AR tracking | `cv::BFMatcher` (Hamming, cross-check) + `solvePnPRansac` on a user-captured reference image |

## Project Files
| File | Description |
|------|-------------|
| `main.cpp` | CLI main loop, key handling, AR state machine |
| `main_gui.cpp` | GUI application entry point |
| `gui_opencv.cpp` / `gui_opencv.h` | OpenCV-based GUI sidebar and rendering |
| `chessboarddetection.cpp` / `.h` | Chessboard corner detection, ArUco detection, world point generation |
| `cameracalibration.cpp` / `.h` | `calibrateCamera` wrapper, YAML save/load |
| `augmentedreality.cpp` / `.h` | 3D axes, outside corners, castle, OBJ overlay, target disguise |
| `featuredetection.cpp` / `.h` | ORB feature detection, Harris corner detection |
| `orbtracking.cpp` / `.h` | ORB-based planar AR tracker (Uber Extension 2) |
| `modelloader.cpp` / `.h` | Wavefront OBJ file parser |
| `CMakeLists.txt` | Build configuration for both CLI and GUI targets |

## Extensions
- **Uber Extension 2 — ORB AR Tracking:** Press `r` to capture any flat surface as the reference image, then press `t` to activate tracking. ORB features are matched each frame using a BFMatcher with Hamming distance and cross-check filtering. Good matches are used to build 3D–2D correspondences (mapping reference pixels to world-plane coordinates) and `solvePnPRansac` estimates the camera pose. When tracking succeeds, 3D axes and the castle overlay are drawn on the tracked surface.
- **Multiple ArUco Targets:** In ArUco mode (`m`), 3D axes are drawn independently on every detected ArUco marker in the scene, not just the first one.
- **Target Disguise:** Pressing `d` overlays a semi-transparent orange/dark-orange mosaic over the detected chessboard, painting over each square individually using its projected corners, so the calibration target is no longer visible as a checkerboard.

## Time Travel Days
0 days used.

## Videos
[Insert link to demo video]

## Acknowledgements
- OpenCV documentation for calibration, pose estimation, ArUco, and ORB references
- Course materials provided by Prof. Bruce Maxwell
- An AI assistant (Claude) was used to help write and debug code, and for project documentation.
