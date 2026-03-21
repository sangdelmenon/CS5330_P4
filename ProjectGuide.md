# Project 4: Guide

**Calibration and Augmented Reality**
**CS 5330 | What has been done and what needs to be done**

## 1. What Has Been Done

The entire codebase is complete and working. All seven tasks from the project specification have been implemented, along with several major extensions. Here is a summary of everything that is already built and tested.

**1.1 Calibration Pipeline (Tasks 1-4)**

* **Target Detection:** The system automatically finds either a standard 9x6 checkerboard or an ArUco marker. The mode is toggled via the `m` keystroke. It uses `cv::findChessboardCorners` and `cv::cornerSubPix` for high accuracy.
* **Calibration:** The program allows the user to save frames using the `s` key. It can run the calibration (`cv::calibrateCamera`) by pressing `c`. The resulting intrinsic parameters are saved to or loaded from `calibration.yml` via `cv::FileStorage`.
* **Pose Estimation:** Once calibrated, the system continuously calculates the camera's rotation (`rvec`) and translation (`tvec`) relative to the target using `cv::solvePnP`.

**1.2 Augmented Reality (Tasks 5-6)**

* **Axes & Corners:** The system projects 3D coordinate axes (X=red, Y=green, Z=blue) as arrowed lines, each 5 squares long, using `cv::arrowedLine` with labeled tips (X/Y/Z). The outside boundary of the checkerboard is also drawn with yellow dots.
* **Virtual Object:** A complex, multi-colored 3D castle has been built using `cv::projectPoints`. It features semi-transparent filled faces (60% alpha blend via `cv::fillConvexPoly` + `cv::addWeighted`) overlaid with anti-aliased wireframe edges. Structure: blue outer walls (2.5 units tall), four red corner towers (1.2×1.2 footprint, 4.0 units tall) with yellow pyramid roofs, a green central keep (2.0×2.0, 4.0 units tall) with a magenta roof, an orange gate archway, and a flagpole with a red flag on top.

**1.3 Feature Detection (Task 7)**

* **Robust Features:** The system includes real-time toggles for detecting and displaying ORB features (key `f`) and Harris corners (key `h`) on the video stream.

**1.4 Extensions**

* **ArUco Markers:** Implemented an alternative to the checkerboard using ArUco markers, which are asymmetrical and often more stable for AR tracking.
* **Multiple ArUco AR:** In ArUco mode, 3D axes are drawn independently on every detected marker in the scene, not just the first one.
* **OBJ Model Loading:** A custom model loader reads and renders `Lowpoly_tree_sample2.obj` (a low-poly house) on the target. Face edges are drawn with anti-aliased `cv::line` (thickness 2) and small green vertex dots (`cv::circle`) are drawn at each projected vertex. The model footprint is 4.0×3.2 board squares, centered on the board.
* **Target Disguise:** Pressing `d` overlays a semi-transparent orange/dark-orange mosaic over the detected chessboard. Every square — including the outer border — is filled via `cv::fillConvexPoly` with a 65% alpha blend so the calibration target is fully hidden.
* **ORB AR Tracking:** Press `r` to capture any flat surface as a reference image (extracts 2000 ORB keypoints), then `t` to activate tracking. BFMatcher (Hamming, cross-check) finds good matches (distance ≤ 2.0× best), and `solvePnPRansac` (reprojectionError=5.0, min 12 inliers) estimates the camera pose — no printed checkerboard required.

---

## 2. What We Need to Do Next

The main thing remaining is to run the calibration properly, capture the necessary screenshots, record a demo, and write the report.

**2.1 Set Up the Workspace**

* **Print the Target:** We need a physical copy of the `checkerboard.png` target. Tape it to something flat and rigid like a piece of cardboard or a clipboard so it doesn't bend.
* **Alternative:** You can display the checkerboard on a flat tablet or phone screen, but printing is usually easier to avoid glare.
* Make sure the room is well-lit so the corner detection does not flicker.

**2.2 Build and Run**
Make sure OpenCV is installed. Build the project via CMake in CLion or terminal:

```bash
mkdir -p cmake-build-debug && cd cmake-build-debug && cmake .. && make

```

Make sure `Lowpoly_tree_sample2.obj` is copied into the working directory if you want to test the OBJ extension. Run the program:

```bash
./Project4

```

**2.3 Key Bindings Reference**

| Key | Action |
| --- | --- |
| **s** | Save the current frame's corners for calibration |
| **c** | Run the calibration calculation (requires >= 5 saved frames) |
| **w** | Write the calibration matrix and distortion coefficients to a file |
| **a** | Toggle 3D axes display |
| **v** | Toggle the virtual wireframe castle display |
| **o** | Toggle the loaded OBJ model display |
| **p** | Print the current pose (rotation and translation matrices) to the terminal |
| **f** | Toggle ORB feature detection |
| **h** | Toggle Harris corner detection |
| **m** | Toggle ArUco marker mode (switches away from chessboard) |
| **d** | Toggle target disguise (paints over the chessboard) |
| **r** | Capture current frame as ORB AR tracking reference |
| **t** | Toggle ORB AR tracking mode |
| **x** | Save a screenshot of the current OpenCV window |
| **q / ESC** | Quit |

---

## 3. Step-by-Step: Generate Report Data

> **IMPORTANT RULE FOR ALL SCREENSHOTS**
> Each screenshot must show **only one feature at a time**. Before enabling a new feature, press the same key again to toggle the previous one **OFF**. The `v` (castle) key is easy to forget — make sure it is off before capturing feature detection shots.
> Use `x` to save a screenshot at any point.

---

**3.1 Rebuild After Code Fixes**

Before testing, rebuild the project to pick up the latest bug fixes (disguise coverage, ArUco grayscale, ORB reference):

```bash
cd cmake-build-debug && make
```

---

**3.2 Run the Calibration**

1. Launch the program from `cmake-build-debug/`: `./Project4`
2. Point the camera at the printed checkerboard in good lighting.
3. Move the board (or camera) to different angles and distances.
4. Press `s` to save a frame. Repeat **at least 5–10 times** from clearly different perspectives (tilted left, right, close, far, rotated).
5. Press `c` to run calibration. The terminal will print:
   - The **camera matrix** (3×3)
   - The **distortion coefficients**
   - The **re-projection error** (RMS). A value under 1.0 is excellent; under 3.0 is acceptable.
   - Write these numbers down — they go into the report.
6. Press `w` to save `calibration.yml`.

---

**3.3 Screenshot 1 — Target Detection**

**Files needed:**
- `ProjectFiles/checkerboard.png` — already in the repo. Print it on A4/letter paper and tape it to a flat rigid surface (cardboard, clipboard). Alternatively, display it full-screen on a tablet or phone held flat.

**What to do:** With the board in view and no keys toggled on, the program automatically draws detected corners.

**What you should see:** A rainbow-colored grid of circles drawn at every inner corner of the chessboard by `drawChessboardCorners`. Yellow dots appear at the four outer corners of the board. Nothing else is overlaid.

**How to capture:** Just point the camera at the board and press `x`. No extra keys needed.

---

**3.4 Screenshot 2 — 3D Axes**

**Files needed:**
- Same printed/displayed `checkerboard.png` as above.

**What to do:**
1. Make sure `v`, `f`, `h`, `d`, `t` are all OFF.
2. Press `a` to toggle axes ON.
3. **Tilt the camera at roughly 45° to the board** so the Z axis (blue, pointing up) is clearly separated from X and Y. Looking straight down makes all axes appear flat.
4. Press `x` to save.
5. Press `a` again to toggle OFF.

**What you should see:** Three arrowed colored lines shooting out from the origin corner of the board — red (X along the board), green (Y along the board), blue (Z pointing up out of the board). Each axis is **5 squares long** with an arrowhead and a letter label (X/Y/Z) at the tip. All three axes must be clearly visible and distinct — if any axis collapses to a dot or short line, change the camera angle.

---

**3.5 Screenshot 3 — Virtual Castle**

**Files needed:**
- Same printed/displayed `checkerboard.png` as above.

**What to do:**
1. Make sure all other keys are OFF.
2. Press `v` to toggle the castle ON.
3. Tilt the camera to a 45-degree angle for a dramatic view and press `x`.
4. Press `v` to toggle OFF.

**What you should see:** A multi-colored castle floating on the board with **semi-transparent filled faces** (stone gray walls, dark towers, blue-red roofs, green keep, purple keep roof) and anti-aliased colored wireframe edges over them — blue outer walls, four red corner towers with yellow pyramid roofs, a green central keep with a magenta roof, an orange gate arch on the front, and a white flagpole with a red flag on top. All parts are anchored to the board and move correctly as the camera moves. The castle is significantly taller and wider than the old wireframe-only version.

---

**3.6 Screenshot 4 — ORB Feature Detection**

**Files needed:** None — ORB runs on whatever the camera sees. Having the chessboard in view gives good results but is not required.

**What to do:**
1. Make sure `v`, `h`, `a`, `d`, `t` are all OFF.
2. Press `f` to toggle ORB features ON.
3. Press `x` to save.
4. Press `f` to toggle OFF.

**What you should see:** Green circles with a line through them (keypoints) scattered densely across the frame wherever interesting texture exists — lots of them on the chessboard squares and on background objects. The top-left HUD should show "ORB: NNN key pts".

---

**3.7 Screenshot 5 — Harris Corner Detection**

**Files needed:** None — Harris runs on whatever the camera sees. Having the chessboard in view works well since it has many sharp corners.

**What to do:**
1. Make sure `v`, `f`, `a`, `d`, `t` are all OFF.
2. Press `h` to toggle Harris ON.
3. Press `x` to save.
4. Press `h` to toggle OFF.

**What you should see:** Small colored dots at sharp corners in the image — concentrated heavily at the chessboard corner intersections and at edges of objects in the background. The top-left HUD shows "Harris: NN corners".

---

**3.8 Screenshot 6 — Target Disguise (Extension)**

**Files needed:**
- Same printed/displayed `checkerboard.png` as above. The chessboard must be fully detected before the disguise activates.
- Must have rebuilt after the code fix (step 3.1).

**What to do:**
1. Make sure all other keys are OFF.
2. Point camera at the chessboard so it is detected (you will see the corner grid drawn).
3. Press `d` to toggle disguise ON.
4. Press `x` to save.
5. Press `d` to toggle OFF.

**What you should see:** The entire chessboard is covered by an alternating orange / dark-orange semi-transparent mosaic. Every square — including the outer border squares — is filled with a colored quad projected in 3D. The black-and-white checkerboard pattern should no longer be visible underneath.

---

**3.9 Screenshot 7 — OBJ Model (Extension)**

**Files needed:**
- Same printed/displayed `checkerboard.png` as above.
- `Lowpoly_tree_sample2.obj` — a low-poly house model. This file is already created and placed in both the project root and `cmake-build-debug/`.

**File placement (already done):**
The file `Lowpoly_tree_sample2.obj` is at:
- `CS5330_P4/Lowpoly_tree_sample2.obj` (project root, for version control)
- `CS5330_P4/cmake-build-debug/Lowpoly_tree_sample2.obj` (where the executable reads it)

If you ever delete `cmake-build-debug/` and regenerate it, re-copy the file:
```bash
cp Lowpoly_tree_sample2.obj cmake-build-debug/
```

**Prerequisite:** Calibration must be loaded (`calibration.yml` present in `cmake-build-debug/`). Restart the program — on startup the terminal must print:
```
OBJ model loaded: 25 vertices.
```
If it does not print this line, copy the OBJ file manually:
```bash
cp /Users/san/CLionProjects/CS5330_P4/Lowpoly_tree_sample2.obj cmake-build-debug/
```

**What to do:**
1. Make sure all other keys are OFF.
2. Point camera at the chessboard.
3. Press `o` to toggle OBJ model ON.
4. Press `x` to save.
5. Press `o` to toggle OFF.

**What you should see:** A green wireframe low-poly house (4.0×3.2 squares footprint) projected and anchored on the chessboard target. Face edges are drawn with anti-aliased `cv::line` (thickness 2) and small bright-green dots appear at each projected vertex. The house includes a pyramid roof, chimney, door outline, and window outline.

---

**3.10 Screenshot 8 — ArUco Marker Mode (Extension)**

**Files needed:**
- An ArUco marker image from dictionary `DICT_6X6_250`. Generate one using the steps below.

**How to get the ArUco marker:**
1. Open this URL in a browser: `https://chev.me/arucogen/`
2. Set **Dictionary** to `6x6 (250)`
3. Set **Marker ID** to `0`
4. Set **Marker size** to `200` mm
5. Click **Download SVG** or right-click the marker image and save it.

**How to display the marker:**
- **Option A (recommended):** Open the saved image or the webpage on your phone/tablet and display it full-screen. Hold it flat and steady in front of the webcam.
- **Option B:** Print the marker on paper. Make sure it prints with a white border around the black pattern — the white border is required for detection.

**What to do:**
1. Make sure all other keys are OFF.
2. Press `m` to switch to ArUco mode. The HUD at the bottom will show "ArUco".
3. Hold the marker in front of the camera.
4. Press `a` to show axes on the detected marker.
5. Press `x` to save.
6. Press `a` then `m` to return to chessboard mode.

**What you should see:** A green border drawn around the detected ArUco marker with its ID number labeled. 3D axes are projected at the marker's corner. If you have two markers in view at once, axes appear independently on each one.

---

**3.11 Screenshot 9 — ORB AR Tracking (Extension)**

**Files needed:** No downloaded files. You need a **flat textured surface** — a book cover, a magazine, a poster on a wall, or even the desk surface. Avoid plain white walls or blank paper as they have no features for ORB to match.

**What to do (order matters — do not skip steps):**
1. Make sure all other keys are OFF and you are in chessboard mode (not ArUco).
2. Point the camera at the flat textured surface.
3. Press `r` — this captures the reference image. The terminal must print:
   ```
   ORB reference captured: XXXX keypoints.
   ```
   If the keypoint count is below ~300, the surface has too little texture. Move to a more detailed surface and press `r` again. Good surfaces: book covers, magazine pages, posters, patterned fabric. The tracker now uses 2000 ORB keypoints per frame for improved robustness.
4. Press `t` to enable tracking. The HUD will show `ORB Track: searching...` in orange.
5. Move the camera slightly and back toward the same surface. When locked on, the HUD turns **green** and shows `ORB Track: NN inliers`.
6. Press `x` to save **only when the HUD is green**. An orange HUD means it is still searching.
7. Press `t` to toggle OFF.

**What you should see:** 3D axes and the wireframe castle appear floating on the flat surface you captured — with no printed chessboard in view. The overlay tracks as you move the camera. The HUD shows a green inlier count when tracking is successful.

---

**3.12 Observe Pose Translation (Task 4)**

1. Return to chessboard mode with the board detected.
2. Press `p` — the terminal prints the current `rvec` (rotation) and `tvec` (translation).
3. Move the camera slowly to the **right** and press `p` again. The X value in `tvec` should increase.
4. Move the camera **up** and press `p`. The Y value in `tvec` should change.
5. Move the camera **closer** and press `p`. The Z value in `tvec` should decrease (camera is closer to the board).
6. Note these observations — the report requires a written description of how translation values change with camera movement.

---

**3.13 Record a Demo Video**

Use macOS screen recorder (`Cmd+Shift+5`) to capture a 30–60 second clip:
1. Show the chessboard being detected with corners drawn.
2. Turn on the castle (`v`) and move the camera around to prove it stays anchored in 3D.
3. Turn on the disguise (`d`) briefly to show the board being hidden.
4. Switch to ORB tracking (`r` then `t`) and show the castle on a plain surface.
5. Upload to YouTube (Unlisted) or Google Drive and save the link for the report.

---

## 4. Updating the Report

The report needs to be submitted as a PDF. It must not include code. Here is exactly what needs to go into it based on the Canvas rubric:

**4.1 Project Overview**

* Write a short description (under 200 words) of the project, explaining how we used camera calibration to calculate pose and anchor 3D objects to a 2D physical target.
* Explicitly state that we primarily used the standard 9x6 Checkerboard, but also implemented ArUco markers as an extension. Describe any lighting or glare challenges we had with detection.

**4.2 Calibration Data**

* Include a screenshot of the detected target (corners highlighted).
* Copy and paste the Camera Matrix and the Re-projection Error from the terminal output. (A good error is usually < 1.0, but 2-3 is fine for high-res webcams).

**4.3 Pose Estimation Discussion**

* Include a short write-up describing how the translation (`tvec`) values change when you move the camera side-to-side (e.g., "When moving the camera to the right, the X translation value increased/decreased..."). Do the numbers make logical sense given the coordinate system?

**4.4 AR Screenshots & Description**

* Include a screenshot of the 3D axes projected onto the board.
* Include a screenshot of our virtual object (the multi-colored castle). Describe the object in the text (e.g., "A castle with semi-transparent filled faces (60% alpha blend via `cv::fillConvexPoly` + `cv::addWeighted`) and anti-aliased wireframe edges, featuring blue outer walls, red corner towers with yellow pyramid roofs, a green central keep with a magenta roof, and a red flagpole, all anchored to the board via `cv::projectPoints`").

**4.5 Feature Detection Discussion**

* Include screenshots of the ORB and Harris corner detections.
* Explain: How could these feature points be used for AR? (Hint: Explain that if we track robust features like ORB/SURF between frames, we could do "markerless" AR. Once we know the camera intrinsic matrix, we can track natural features in the room instead of relying on a printed checkerboard).

**4.6 Extensions**

* Show screenshots and briefly describe our extensions:
1. **ArUco Markers** — alternative target detection using ArUco markers.
2. **Multiple ArUco AR** — independent 3D axes drawn on every detected marker simultaneously.
3. **OBJ Model Loader** — custom parser rendering `Lowpoly_tree_sample2.obj` (low-poly house, 4×3.2 board squares) with vertex dots and anti-aliased face edges.
4. **Target Disguise** — semi-transparent orange mosaic overlay that hides every chessboard square (including border squares) in real time.
5. **ORB AR Tracking** — markerless AR on any flat textured surface using 2000-keypoint ORB matching, BFMatcher with cross-check, and `solvePnPRansac` (reprojectionError=5.0, min 12 inliers).



**4.7 Demo Link**

* Include the link to the demo video.

---

## 5. File Overview

| File | What It Does |
| --- | --- |
| `main.cpp` | Main video loop, key handling, and state management (toggling modes). |
| `cameracalibration.cpp/.h` | Handles `calibrateCamera` logic and saving/loading YAML files. |
| `augmentedreality.cpp/.h` | All drawing logic: arrowed 3D axes (5 squares, `cv::arrowedLine`), outside corners, `drawCastle` (filled faces + wireframe), `drawTargetDisguise`. |
| `chessboarddetection.cpp/.h` | Wraps `findChessboardCorners` and the ArUco marker detection logic. |
| `featuredetection.cpp/.h` | Implements Harris corner and ORB feature detection toggles. |
| `orbtracking.cpp/.h` | ORB-based planar AR tracker — 2000-keypoint ORB, BFMatcher cross-check, `solvePnPRansac` (reprojError=5.0, min 12 inliers) for robust markerless tracking. |
| `modelloader.cpp/.h` | Extension: parses `.obj` files and loads 3D vertices/faces to project onto the target. |
| `gui_opencv.cpp/.h` | OpenCV-based GUI sidebar with buttons and sliders for all AR and calibration controls. |
| `main_gui.cpp` | Entry point for the GUI application (`Project4_GUI`). |
| `calibration.yml` | Generated at runtime: Stores the intrinsic camera matrix and distortion coefficients. |