/*
  main_gui.cpp
  CS 5330 - Project 4: Calibration and Augmented Reality
  GUI version — OpenCV sidebar, all features + extensions.

  All buttons work. Actions that need the current frame
  (Save Frame, Calibrate, Screenshot) use boolean flags that
  are consumed at the top of the main loop after detection.
*/
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "chessboarddetection.h"
#include "cameracalibration.h"
#include "augmentedreality.h"
#include "featuredetection.h"
#include "modelloader.h"
#include "orbtracking.h"
#include "gui_opencv.h"

namespace fs = std::filesystem;

// ── Constants ────────────────────────────────────────────────────────────────
static const cv::Size       PATTERN_SIZE(9, 6);
static const std::string    CALIB_FILE  = "calibration.yml";
static const std::string    MODEL_FILE  = "Lowpoly_tree_sample2.obj";
static const std::string    WIN_NAME    = "Project 4: Calibration & AR  [GUI]";

// ── Global GUI pointer (needed by the static mouse callback) ─────────────────
static CVGUI *g_gui = nullptr;
static void onMouse(int event, int x, int y, int /*flags*/, void* /*data*/)
{
    if (event == cv::EVENT_LBUTTONDOWN && g_gui)
        g_gui->handleClick(x, y);
}

// ── Helpers ──────────────────────────────────────────────────────────────────

// Format a double to N decimal places as a string.
static std::string fmt(double v, int dp = 3)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(dp) << v;
    return ss.str();
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    // ── Open video source ──
    cv::VideoCapture cap;
    if (argc > 1) cap.open(argv[1]);
    else          cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open video source.\n";
        return 1;
    }

    // ── Load calibration if available ──
    cv::Mat cameraMatrix, distCoeffs;
    bool calibrated = loadCalibration(CALIB_FILE, cameraMatrix, distCoeffs);
    if (!calibrated) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);
        std::cout << "No calibration found. Save frames (s) then calibrate (c).\n";
    }

    // ── Load OBJ model (optional) ──
    std::vector<Vertex>       objV;
    std::vector<TextureCoord> objT;
    std::vector<Normal>       objN;
    std::vector<Face>         objF;
    bool modelLoaded = false;
    if (fs::exists(MODEL_FILE))
        modelLoaded = loadOBJModel(MODEL_FILE, objV, objT, objN, objF);

    // ── Calibration data ──
    std::vector<std::vector<cv::Point2f>> cornerList;
    std::vector<std::vector<cv::Vec3f>>   pointList;
    std::vector<cv::Vec3f> worldPoints = generateChessboardPoints(PATTERN_SIZE);

    // ── Application state ──
    bool   showAxes     = false;
    bool   showPawn     = false;
    bool   showQueen    = false;
    bool   showOBJ      = false;
    bool   showDisguise = false;
    bool   showORB      = false;
    bool   showHarris   = false;
    bool   useAruco     = false;
    bool   orbTrackMode = false;
    int    saveCounter  = 0;
    double lastRms      = -1.0;

    cv::Mat rvec, tvec;
    bool poseValid = false;

    ORBTracker orbTracker;

    // ── Pending-action flags (set by buttons, consumed in main loop) ──
    bool flagSave          = false;
    bool flagCalibrate     = false;
    bool flagScreenshot    = false;
    bool flagCaptureRef    = false;

    // ── Create GUI ──
    CVGUI gui;
    g_gui = &gui;

    gui.setupButtons(
        /*onSave*/         [&]() { flagSave      = true; },
        /*onCalibrate*/    [&]() { flagCalibrate = true; },
        /*onWrite*/        [&]() {
            if (calibrated)
                saveCalibration(CALIB_FILE, cameraMatrix, distCoeffs,
                                lastRms >= 0 ? lastRms : 0.0);
            else
                std::cout << "[!] Not calibrated yet.\n";
        },
        /*onToggleAxes*/   [&]() {
            showAxes = !showAxes;
            std::cout << "3D Axes: " << (showAxes ? "ON" : "OFF") << "\n";
        },
        /*onTogglePawn*/   [&]() {
            showPawn = !showPawn;
            std::cout << "Chess Pawn: " << (showPawn ? "ON" : "OFF") << "\n";
        },
        /*onToggleQueen*/  [&]() {
            showQueen = !showQueen;
            std::cout << "Chess Queen: " << (showQueen ? "ON" : "OFF") << "\n";
        },
        /*onToggleOBJ*/    [&]() {
            showOBJ = !showOBJ;
            std::cout << "OBJ Model: " << (showOBJ ? "ON" : "OFF") << "\n";
        },
        /*onToggleDisguise*/[&]() {
            showDisguise = !showDisguise;
            std::cout << "Target Disguise: " << (showDisguise ? "ON" : "OFF") << "\n";
        },
        /*onToggleORB*/    [&]() {
            showORB = !showORB;
            if (showORB) showHarris = false;
            std::cout << "ORB Features: " << (showORB ? "ON" : "OFF") << "\n";
        },
        /*onToggleHarris*/ [&]() {
            showHarris = !showHarris;
            if (showHarris) showORB = false;
            std::cout << "Harris Corners: " << (showHarris ? "ON" : "OFF") << "\n";
        },
        /*onToggleAruco*/  [&]() {
            useAruco = !useAruco;
            std::cout << "Mode: " << (useAruco ? "ArUco" : "Chessboard") << "\n";
        },
        /*onCaptureRef*/   [&]() { flagCaptureRef = true; },
        /*onToggleTrack*/  [&]() {
            orbTrackMode = !orbTrackMode;
            std::cout << "ORB Tracking: " << (orbTrackMode ? "ON" : "OFF") << "\n";
            if (orbTrackMode && !orbTracker.hasReference)
                std::cout << "  Click 'Capture Ref' on your target first.\n";
        },
        /*onPrintPose*/    [&]() {
            if (poseValid)
                std::cout << "Rotation:    " << rvec.t() << "\n"
                          << "Translation: " << tvec.t() << "\n";
            else
                std::cout << "No valid pose.\n";
        },
        /*onScreenshot*/   [&]() { flagScreenshot = true; }
    );

    cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(WIN_NAME, onMouse);

    std::cout << "\n=== Project 4 GUI — key shortcuts still work ===\n"
              << "s  c  w  a  v  o  d  f  h  m  r  t  p  x  q\n\n";

    // ─── Main loop ────────────────────────────────────────────────────────────
    while (true)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Clean copy for ORB reference capture (before any overlays)
        cv::Mat cleanFrame = frame.clone();

        std::vector<cv::Point2f>              corners;
        std::vector<std::vector<cv::Point2f>> allMarkerCorners;
        std::vector<int>                      allMarkerIds;
        bool targetFound = false;

        // ── Detect target ──
        if (useAruco) {
            targetFound = detectAruco(frame, allMarkerCorners, allMarkerIds);
            if (targetFound && !allMarkerIds.empty()) {
                corners = allMarkerCorners[0];
                std::vector<cv::Point3f> mw = {{0,0,0},{1,0,0},{1,-1,0},{0,-1,0}};
                if (calibrated)
                    poseValid = cv::solvePnP(mw, corners,
                                             cameraMatrix, distCoeffs, rvec, tvec);
            }
        } else {
            targetFound = detectChessboard(frame, PATTERN_SIZE, corners);
            if (targetFound && calibrated)
                poseValid = cv::solvePnP(worldPoints, corners,
                                         cameraMatrix, distCoeffs, rvec, tvec);
        }

        // ── Consume pending flags ──
        if (flagSave) {
            flagSave = false;
            if (targetFound && !useAruco) {
                cornerList.push_back(corners);
                pointList.push_back(worldPoints);
                std::cout << "[OK] Saved calibration frame #"
                          << cornerList.size() << "\n";
            } else if (!targetFound) {
                std::cout << "[!] Target not found — cannot save.\n";
            } else {
                std::cout << "[!] Save only works in chessboard mode.\n";
            }
        }

        if (flagCalibrate) {
            flagCalibrate = false;
            lastRms = runCalibration(cornerList, pointList,
                                     frame.size(), cameraMatrix, distCoeffs);
            if (lastRms >= 0) calibrated = true;
        }

        if (flagCaptureRef) {
            flagCaptureRef = false;
            orbTracker.setReference(cleanFrame);
        }

        // ── AR overlays ──
        if (targetFound && poseValid && calibrated) {
            if (showDisguise && !useAruco)
                drawTargetDisguise(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);

            if (showAxes)
                draw3DAxes(frame, cameraMatrix, distCoeffs, rvec, tvec);

            if (!useAruco)
                drawOutsideCorners(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);

            if (showPawn && !useAruco)
                drawChessPawn(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);

            if (showQueen && !useAruco)
                drawChessQueen(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);

            if (showOBJ && modelLoaded) {
                std::vector<cv::Point2f> ip;
                for (auto &v : objV) {
                    std::vector<cv::Point3f> p3 = {{v.x, v.y, v.z}};
                    std::vector<cv::Point2f> p2;
                    cv::projectPoints(p3, rvec, tvec, cameraMatrix, distCoeffs, p2);
                    ip.push_back(p2[0]);
                }
                for (auto &face : objF)
                    for (size_t i = 0; i < face.vertexIndices.size(); i++) {
                        int a = face.vertexIndices[i] - 1;
                        int b = face.vertexIndices[(i+1) % face.vertexIndices.size()] - 1;
                        if (a>=0 && a<(int)ip.size() && b>=0 && b<(int)ip.size())
                            cv::line(frame, ip[a], ip[b], cv::Scalar(0,180,0), 1);
                    }
            }

            // Multiple ArUco markers — draw axes on every additional marker
            if (useAruco && calibrated && (int)allMarkerIds.size() > 1) {
                std::vector<cv::Point3f> mw = {{0,0,0},{1,0,0},{1,-1,0},{0,-1,0}};
                for (int mi = 1; mi < (int)allMarkerIds.size(); mi++) {
                    cv::Mat rv, tv;
                    if (cv::solvePnP(mw, allMarkerCorners[mi],
                                     cameraMatrix, distCoeffs, rv, tv))
                        draw3DAxes(frame, cameraMatrix, distCoeffs, rv, tv);
                }
            }
        }

        // ── ORB tracking (Uber Extension 2) ──
        if (orbTrackMode && calibrated) {
            cv::Mat oRvec, oTvec;
            bool tracked = orbTracker.track(frame, cameraMatrix, distCoeffs,
                                            oRvec, oTvec);
            if (tracked) {
                draw3DAxes(frame, cameraMatrix, distCoeffs, oRvec, oTvec);
                drawChessPawn(frame, cameraMatrix, distCoeffs, oRvec, oTvec,
                              cv::Size(5, 4));
                drawChessQueen(frame, cameraMatrix, distCoeffs, oRvec, oTvec,
                               cv::Size(5, 4));
            }
            std::string tmsg = tracked
                ? "ORB Track: " + std::to_string(orbTracker.lastInliers) + " inliers"
                : (orbTracker.hasReference ? "ORB Track: searching..."
                                           : "ORB Track: capture reference first");
            cv::putText(frame, tmsg, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        tracked ? cv::Scalar(0,255,0) : cv::Scalar(0,130,255), 2);
        }

        // ── Feature detection overlays ──
        if (showORB)    detectORBFeatures(frame);
        if (showHarris) detectHarrisCorners(frame);

        // ── HUD (bottom of camera frame) ──
        {
            std::string hud = (useAruco ? "ArUco" : "Chessboard");
            hud += targetFound ? " [FOUND]" : " [not found]";
            hud += "  Frames:" + std::to_string(cornerList.size());
            if (calibrated) hud += "  [CALIBRATED]";
            cv::putText(frame, hud, cv::Point(8, frame.rows - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,220,220), 1);
        }

        // ── Screenshot (after all overlays, before sidebar is added) ──
        if (flagScreenshot) {
            flagScreenshot = false;
            std::vector<std::string> parts;
            if (useAruco)      parts.push_back("aruco");
            if (showAxes)      parts.push_back("axes");
            if (showPawn)      parts.push_back("pawn");
            if (showQueen)     parts.push_back("queen");
            if (showOBJ)       parts.push_back("obj");
            if (showORB)       parts.push_back("orb_features");
            if (showHarris)    parts.push_back("harris");
            if (showDisguise)  parts.push_back("disguise");
            if (orbTrackMode)  parts.push_back("orb_tracking");
            if (parts.empty()) parts.push_back("detection");

            std::string base;
            for (size_t i = 0; i < parts.size(); i++) {
                if (i > 0) base += "_";
                base += parts[i];
            }
            namespace fs = std::filesystem;
            fs::create_directories("screenshots");
            std::string fname = "screenshots/" + base + "_" + std::to_string(saveCounter++) + ".png";
            cv::imwrite(fname, frame);
            std::cout << "[OK] Screenshot saved: " << fname << "\n";
        }

        // ── Build status panel lines ──
        std::vector<std::string> status;
        status.push_back(std::string("Target: ") +
                         (useAruco ? "ArUco" : "Chessboard") +
                         (targetFound ? "  FOUND" : "  not found"));
        status.push_back("Calib frames: " + std::to_string(cornerList.size()) +
                         (calibrated ? "  [OK] Calibrated" : "  [!] Not calibrated"));
        if (calibrated && lastRms >= 0)
            status.push_back("Reproj error: " + fmt(lastRms) + " px");
        if (poseValid && targetFound) {
            if (!tvec.empty())
                status.push_back("T: [" + fmt(tvec.at<double>(0),1) + ", "
                                        + fmt(tvec.at<double>(1),1) + ", "
                                        + fmt(tvec.at<double>(2),1) + "]");
        }
        if (orbTrackMode) {
            if (!orbTracker.hasReference)
                status.push_back("ORB Track: [!] no reference");
            else if (orbTracker.lastInliers > 0)
                status.push_back("ORB Track: " +
                                 std::to_string(orbTracker.lastInliers) + " inliers");
            else
                status.push_back("ORB Track: searching...");
        }

        // ── Sync toggle button visuals ──
        gui.updateToggles(showAxes, showPawn, showQueen, showOBJ, showDisguise,
                          showORB, showHarris, useAruco, orbTrackMode);

        cv::Mat display = gui.buildDisplay(frame, status);
        cv::imshow(WIN_NAME, display);

        // ── Keyboard shortcuts (mirror all CLI keys) ──
        int key = cv::waitKey(30) & 0xFF;
        if      (key == 'q' || key == 27) break;
        else if (key == 's') flagSave      = true;
        else if (key == 'c') flagCalibrate = true;
        else if (key == 'w') {
            if (calibrated)
                saveCalibration(CALIB_FILE, cameraMatrix, distCoeffs,
                                lastRms >= 0 ? lastRms : 0.0);
            else std::cout << "[!] Not calibrated yet.\n";
        }
        else if (key == 'a') { showAxes    = !showAxes;
                                std::cout << "Axes: "    << (showAxes?"ON":"OFF")    << "\n"; }
        else if (key == 'v') { showPawn  = !showPawn;
                                std::cout << "Chess Pawn: "  << (showPawn?"ON":"OFF")  << "\n"; }
        else if (key == 'b') { showQueen = !showQueen;
                                std::cout << "Chess Queen: " << (showQueen?"ON":"OFF") << "\n"; }
        else if (key == 'o') { showOBJ     = !showOBJ;
                                std::cout << "OBJ: "     << (showOBJ?"ON":"OFF")     << "\n"; }
        else if (key == 'd') { showDisguise= !showDisguise;
                                std::cout << "Disguise: "<< (showDisguise?"ON":"OFF")<< "\n"; }
        else if (key == 'f') { showORB     = !showORB;   if(showORB) showHarris=false; }
        else if (key == 'h') { showHarris  = !showHarris;if(showHarris) showORB=false; }
        else if (key == 'm') { useAruco    = !useAruco;
                                std::cout << "Mode: " << (useAruco?"ArUco":"Chessboard") << "\n"; }
        else if (key == 'r') flagCaptureRef = true;
        else if (key == 't') {
            orbTrackMode = !orbTrackMode;
            std::cout << "ORB Tracking: " << (orbTrackMode?"ON":"OFF") << "\n";
        }
        else if (key == 'p') {
            if (poseValid) std::cout << "R: " << rvec.t() << "\nT: " << tvec.t() << "\n";
            else           std::cout << "No valid pose.\n";
        }
        else if (key == 'x') flagScreenshot = true;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
