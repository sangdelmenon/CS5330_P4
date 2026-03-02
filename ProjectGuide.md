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

* **Axes & Corners:** The system can project 3D coordinate axes (X=red, Y=green, Z=blue) with a length of 3 squares, and draw the outside boundary of the checkerboard.
* **Virtual Object:** A complex, multi-colored 3D wireframe castle has been built from scratch using 3D geometric coordinates mapped via `cv::projectPoints`. It features blue walls, four red towers with yellow roof pyramids, a green central keep with a magenta roof, an orange gate archway, and a flagpole. It is perfectly anchored to the target.

**1.3 Feature Detection (Task 7)**

* **Robust Features:** The system includes real-time toggles for detecting and displaying ORB features (key `f`) and Harris corners (key `h`) on the video stream.

**1.4 Extensions**

* **ArUco Markers:** Implemented an alternative to the checkerboard using ArUco markers, which are asymmetrical and often more stable for AR tracking.
* **OBJ Model Loading:** Wrote a custom model loader that reads and renders a 3D `.obj` file (`Lowpoly_tree_sample2.obj`) directly onto the target.

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
| **x** | Save a screenshot of the current OpenCV window |
| **q / ESC** | Quit |

---

## 3. Step-by-Step: Generate Report Data

Follow these steps to generate all the data and screenshots needed for the report.

**3.1 Run the Calibration (Tasks 1-3)**

1. Launch the program. Point the camera at the printed checkerboard.
2. Move the camera around to view the board from different angles and distances.
3. Press `s` to save a frame. Do this at least 5-10 times from clearly different perspectives.
4. Press `c` to run the calibration. Look at the terminal output: it will print the camera matrix, distortion coefficients, and Re-projection Error. Write these numbers down for the report!
5. Press `w` to save the `calibration.yml` file.

**3.2 Capture Screenshots (Tasks 1, 5, 6, 7)**
Use the `x` key to save screenshots directly from the program for the following:

* **Target Detection:** A screenshot showing the checkerboard with the colorful OpenCV corners drawn on it.
* **Axes & Outside Corners:** Press `a` to show the 3D axes and take a screenshot.
* **Virtual Object:** Press `v` to show the wireframe castle. Take a screenshot from a cool angle.
* **Extensions:** Press `o` to show the 3D tree OBJ model and take a screenshot. Also, switch to ArUco mode (`m`) and take a screenshot of AR working on the marker.
* **Features:** Press `f` for ORB features and take a screenshot. Press `h` for Harris corners and take a screenshot.

**3.3 Observe Pose Translation (Task 4)**

1. While the target is detected, press `p` in the terminal to print the Rotation (`rvec`) and Translation (`tvec`) matrices.
2. Move the camera slowly to the left/right and up/down, pressing `p` to see how the numbers change. Take notes on this, as the report requires a description of how these values change as you move side-to-side.

**3.4 Record a Demo Video**
Use a screen recorder (Cmd+Shift+5 on macOS) to capture a 30-60 second video. Show the calibration working, then turn on the virtual castle and move the camera around to prove that the object is anchored in 3D space. Upload this to YouTube (Unlisted) or Google Drive and get the link.

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
* Include a screenshot of our virtual object (the multi-colored castle). Describe the object in the text (e.g., "A wireframe castle with blue walls, red towers, a green keep, and a red flag, constructed using `cv::line` and 3D geometric coordinates").

**4.5 Feature Detection Discussion**

* Include screenshots of the ORB and Harris corner detections.
* Explain: How could these feature points be used for AR? (Hint: Explain that if we track robust features like ORB/SURF between frames, we could do "markerless" AR. Once we know the camera intrinsic matrix, we can track natural features in the room instead of relying on a printed checkerboard).

**4.6 Extensions**

* Show screenshots and briefly describe our extensions:
1. The ArUco marker implementation.
2. The custom OBJ file loader rendering the 3D tree.



**4.7 Demo Link**

* Include the link to the demo video.

---

## 5. File Overview

| File | What It Does |
| --- | --- |
| `main.cpp` | Main video loop, key handling, and state management (toggling modes). |
| `cameracalibration.cpp/.h` | Handles `calibrateCamera` logic and saving/loading YAML files. |
| `augmentedreality.cpp/.h` | Contains all the drawing logic: 3D axes, outside corners, and the custom `drawCastle` wireframe rendering. |
| `chessboarddetection.cpp/.h` | Wraps `findChessboardCorners` and the ArUco marker detection logic. |
| `featuredetection.cpp/.h` | Implements Harris corner and ORB feature detection toggles. |
| `modelloader.cpp/.h` | Extension: parses `.obj` files and loads 3D vertices/faces to project onto the target. |
| `calibration.yml` | Generated at runtime: Stores the intrinsic camera matrix and distortion coefficients. |