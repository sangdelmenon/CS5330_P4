# Project 4: Calibration and Augmented Reality

## Team Members
- **Sangeeth Deleep Menon** | NUID: 002524579 | MSCS - Boston | CS5330 Section 03 (CRN: 40669, Online)
- **Raj Gupta** | NUID: 002068701 | MSCS - Boston | CS5330 Section 01 (CRN: 38745, Online)

## Project Description
For this project we built a camera calibration and augmented reality system in C++ using OpenCV. The program detects a printed chessboard target, figures out the camera's intrinsic parameters and distortion coefficients from multiple saved frames, and then uses that calibration to track the board's position and orientation in real time. Once the pose is known each frame, we render virtual 3D objects that stay locked to the board as the camera moves around. We also added support for ArUco markers as an alternative target, and implemented an ORB feature matching tracker so that any flat textured surface can be used as an AR target without needing a printed chessboard at all.

## Building the Project
The project uses CMake and requires OpenCV with the calib3d, objdetect, and features2d modules, along with a C++17 compiler.
1. **Navigate to the project directory.**
2. **Create a build directory and compile:**
    ```bash
    mkdir -p cmake-build-debug && cd cmake-build-debug
    cmake ..
    make
    ```
    This produces two executables inside `cmake-build-debug/`: `Project4` for the command line and `Project4_GUI` for the graphical interface.

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
The GUI opens an OpenCV window with a sidebar panel containing buttons and sliders for every feature, so you can toggle overlays and run calibration without memorizing key bindings.

### Key Bindings (Command-Line Application)
| Key | Action |
|-----|--------|
| `s` | Save current frame as calibration image (chessboard mode only) |
| `c` | Run camera calibration (requires at least 5 saved frames) |
| `w` | Write calibration data to `calibration.yml` |
| `a` | Toggle 3D coordinate axes overlay |
| `v` | Toggle chess pawn overlay |
| `b` | Toggle chess queen overlay |
| `o` | Toggle OBJ model overlay |
| `p` | Print current rotation and translation vectors to terminal |
| `f` | Toggle ORB feature detection display |
| `h` | Toggle Harris corner detection display |
| `m` | Toggle ArUco marker mode |
| `d` | Toggle target disguise |
| `r` | Capture current frame as ORB tracking reference |
| `t` | Toggle ORB AR tracking mode |
| `x` | Save screenshot |
| `q` / `ESC` | Quit |

## Executable Files
1. **`Project4` (Command-Line Interface)**
   - The main application that runs in a terminal window. It supports webcam input as well as video and image files. Every calibration and AR feature is controlled through keyboard shortcuts.

2. **`Project4_GUI` (Graphical User Interface)**
   - An OpenCV window with a sidebar panel on the right. All the same features are available through buttons and sliders so no keyboard shortcuts are needed.

## Methods Overview
| Task | Implementation |
|------|---------------|
| 1. Target detection | `cv::findChessboardCorners` + `cornerSubPix`; `cv::aruco::ArucoDetector` for ArUco |
| 2. Calibration frame selection | User presses `s` to store corner set and corresponding 3D world points |
| 3. Camera calibration | `cv::calibrateCamera` with `CALIB_FIX_ASPECT_RATIO`; save/load via `cv::FileStorage` YAML |
| 4. Pose estimation | `cv::solvePnP` per frame using stored calibration intrinsics |
| 5. 3D axes / outside corners | `cv::arrowedLine` (5 squares long, labeled tips); `cv::projectPoints` for outer board corners |
| 6. Virtual chess pieces | Pawn (`v`) and Queen (`b`), each toggled independently. Lathe-style 8-sided octagonal rings with 82% alpha `cv::fillConvexPoly` fill and anti-aliased wireframe edges |
| 7. Robust features | ORB via `cv::ORB` and Harris corners via `cv::cornerHarris`, both displayed live |
| Ext. Target disguise | Per-square projected quad fill using `cv::fillConvexPoly` with alpha blending |
| Ext. Multiple ArUco AR | Independent `solvePnP` and `draw3DAxes` called for every detected marker |
| Ext. OBJ model | Custom parser that renders `Lowpoly_tree_sample2.obj` using `cv::line` on each face |
| Ext. ORB AR tracking | `cv::ORB::create(2000)` with `cv::BFMatcher` cross-check matching and `solvePnPRansac` (reprojection error 5.0, minimum 12 inliers) |

## Project Files
| File | Description |
|------|-------------|
| `main.cpp` | CLI main loop, key handling, and AR state machine |
| `main_gui.cpp` | GUI application entry point |
| `gui_opencv.cpp` / `gui_opencv.h` | OpenCV sidebar rendering and button logic |
| `chessboarddetection.cpp` / `.h` | Chessboard corner detection, ArUco detection, and world point generation |
| `cameracalibration.cpp` / `.h` | Wrapper around `calibrateCamera` with YAML save and load |
| `augmentedreality.cpp` / `.h` | 3D axes, outside corners, chess pawn and queen rendering with filled faces and wireframe, and target disguise |
| `featuredetection.cpp` / `.h` | ORB feature detection and Harris corner detection |
| `orbtracking.cpp` / `.h` | ORB-based planar AR tracker using 2000-keypoint ORB, BFMatcher cross-check, and `solvePnPRansac` |
| `modelloader.cpp` / `.h` | Wavefront OBJ file parser |
| `CMakeLists.txt` | Build configuration for both CLI and GUI targets |

## Extensions
**Target Disguise:** When you press `d` while the chessboard is visible, every square on the board gets painted over with a semi-transparent orange mosaic. The squares are filled individually using their projected 3D corners, so the disguise stays locked to the board and the calibration target is completely hidden while pose estimation keeps running underneath.

**Multiple ArUco Targets:** In ArUco mode, the system finds every marker in the frame at the same time. Each detected marker gets its own independent pose estimate and its own set of 3D axes drawn on top of it, so you can have multiple AR anchors active simultaneously.

**OBJ Model Loader:** We wrote a custom parser that reads a Wavefront OBJ file and projects every face edge onto the board using `cv::line`. The model used is a low-poly house with a footprint of roughly 4 by 3.2 board squares. Each projected vertex also gets a small green dot drawn on it to make the shape easier to read.

**Uber Extension 2 (ORB AR Tracking):** This lets you use any flat textured surface as an AR target. You point the camera at the surface and press `r` to save a reference image. Then press `t` to start tracking. Each frame, ORB features are extracted and matched against the reference using BFMatcher with cross-check. Good matches are used to estimate the camera pose with `solvePnPRansac`, and when at least 12 inliers are found the 3D axes and both chess pieces are drawn anchored to that surface.

## Time Travel Days
1 day used.

## Videos
**Panopto Link: https://northeastern.hosted.panopto.com/Panopto/Pages/Viewer.aspx?id=e25c0af1-604f-4f13-aee1-b4140037a390**

## Acknowledgements
- OpenCV documentation for calibration, pose estimation, ArUco, and ORB references
- Course materials provided by Prof. Bruce Maxwell
- An AI assistant (Claude) was used to help write and debug code, and for project documentation.
