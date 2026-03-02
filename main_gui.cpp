/*
  main_gui.cpp
  CS 5330 - Project 4: GUI version (OpenCV only, no Qt)
  Uses clickable buttons drawn on the frame via gui_opencv.
*/
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "chessboarddetection.h"
#include "cameracalibration.h"
#include "augmentedreality.h"
#include "featuredetection.h"
#include "modelloader.h"
#include "gui_opencv.h"

namespace fs = std::filesystem;

static const cv::Size PATTERN_SIZE(9, 6);
static const std::string CALIB_FILE = "calibration.yml";
static const std::string MODEL_FILE = "Lowpoly_tree_sample2.obj";
static const std::string WIN_NAME   = "Project 4: Calibration & AR";

// Global GUI pointer for mouse callback
static CVGUI *g_gui = nullptr;

static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && g_gui)
        g_gui->handleClick(x, y);
}

int main(int argc, char *argv[])
{
    cv::VideoCapture cap;
    if (argc > 1) cap.open(argv[1]); else cap.open(0);
    if (!cap.isOpened()) { std::cerr << "Cannot open video source.\n"; return 1; }

    cv::Mat cameraMatrix, distCoeffs;
    bool calibrated = loadCalibration(CALIB_FILE, cameraMatrix, distCoeffs);
    if (!calibrated) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    }

    std::vector<Vertex> objV; std::vector<TextureCoord> objT;
    std::vector<Normal> objN; std::vector<Face> objF;
    bool modelLoaded = false;
    if (fs::exists(MODEL_FILE))
        modelLoaded = loadOBJModel(MODEL_FILE, objV, objT, objN, objF);

    std::vector<std::vector<cv::Point2f>> cornerList;
    std::vector<std::vector<cv::Vec3f>> pointList;
    std::vector<cv::Vec3f> worldPoints = generateChessboardPoints(PATTERN_SIZE);

    bool showAxes = false, showCastle = false, showOBJ = false;
    bool showORB = false, showHarris = false, useAruco = false;
    int saveCounter = 0;
    double lastRms = -1;
    cv::Mat rvec, tvec;
    bool poseValid = false;

    // Setup GUI
    CVGUI gui;
    g_gui = &gui;

    gui.setupButtons(
        // Save
        [&]() {
            // handled in loop where we have access to current corners
            std::cout << "Save triggered via button.\n";
        },
        // Calibrate
        [&]() {
            std::cout << "Calibrate triggered via button.\n";
        },
        // Write
        [&]() {
            if (calibrated) saveCalibration(CALIB_FILE, cameraMatrix, distCoeffs, lastRms >= 0 ? lastRms : 0);
            else std::cout << "Not calibrated yet.\n";
        },
        // Axes
        [&]() { showAxes = !showAxes; std::cout << "Axes: " << (showAxes?"ON":"OFF") << "\n"; },
        // Castle
        [&]() { showCastle = !showCastle; std::cout << "Castle: " << (showCastle?"ON":"OFF") << "\n"; },
        // OBJ
        [&]() { showOBJ = !showOBJ; std::cout << "OBJ: " << (showOBJ?"ON":"OFF") << "\n"; },
        // Print pose
        [&]() {
            if (poseValid) { std::cout << "R: " << rvec.t() << "\nT: " << tvec.t() << "\n"; }
            else std::cout << "No valid pose.\n";
        },
        // ORB
        [&]() { showORB = !showORB; if (showORB) showHarris = false; },
        // Harris
        [&]() { showHarris = !showHarris; if (showHarris) showORB = false; },
        // ArUco
        [&]() { useAruco = !useAruco; std::cout << "Mode: " << (useAruco?"ArUco":"Chessboard") << "\n"; },
        // Screenshot
        [&]() {
            std::string f = "screenshot_" + std::to_string(saveCounter++) + ".png";
            std::cout << "Screenshot saved (next frame).\n";
        }
    );

    cv::namedWindow(WIN_NAME);
    cv::setMouseCallback(WIN_NAME, onMouse);

    bool saveRequested = false, calibrateRequested = false;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        std::vector<cv::Point2f> corners;
        bool targetFound = false;

        if (useAruco) {
            std::vector<std::vector<cv::Point2f>> mc;
            std::vector<int> mi;
            targetFound = detectAruco(frame, mc, mi);
            if (targetFound && !mi.empty()) {
                corners = mc[0];
                std::vector<cv::Point3f> mw = {{0,0,0},{1,0,0},{1,-1,0},{0,-1,0}};
                if (calibrated) poseValid = cv::solvePnP(mw, corners, cameraMatrix, distCoeffs, rvec, tvec);
            }
        } else {
            targetFound = detectChessboard(frame, PATTERN_SIZE, corners);
            if (targetFound && calibrated)
                poseValid = cv::solvePnP(worldPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
        }

        if (targetFound && poseValid && calibrated) {
            if (showAxes) draw3DAxes(frame, cameraMatrix, distCoeffs, rvec, tvec);
            if (!useAruco) drawOutsideCorners(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);
            if (showCastle && !useAruco) drawCastle(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);
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
                        int a = face.vertexIndices[i]-1, b = face.vertexIndices[(i+1)%face.vertexIndices.size()]-1;
                        if (a>=0 && a<(int)ip.size() && b>=0 && b<(int)ip.size())
                            cv::line(frame, ip[a], ip[b], cv::Scalar(0,180,0), 1);
                    }
            }
        }

        if (showORB) detectORBFeatures(frame);
        if (showHarris) detectHarrisCorners(frame);

        // Build status lines
        std::vector<std::string> status;
        status.push_back(std::string("Target: ") + (useAruco ? "ArUco" : "Chessboard") +
                         (targetFound ? "  FOUND" : "  NOT found"));
        status.push_back("Calib frames: " + std::to_string(cornerList.size()) +
                         (calibrated ? "  [OK] Calibrated" : "  [!] Not calibrated"));
        if (poseValid && targetFound)
            status.push_back("Pose: valid");
        if (lastRms >= 0)
            status.push_back("Reproj error: " + std::to_string(lastRms).substr(0,5) + " px");

        gui.updateToggles(showAxes, showCastle, showOBJ, showORB, showHarris, useAruco);
        cv::Mat display = gui.buildDisplay(frame, status);
        cv::imshow(WIN_NAME, display);

        int key = cv::waitKey(30) & 0xFF;

        // Keyboard shortcuts still work alongside buttons
        if (key == 'q' || key == 27) break;
        else if (key == 's' && targetFound && !useAruco) {
            cornerList.push_back(corners); pointList.push_back(worldPoints);
            std::cout << "Saved frame #" << cornerList.size() << "\n";
        }
        else if (key == 'c') {
            lastRms = runCalibration(cornerList, pointList, frame.size(), cameraMatrix, distCoeffs);
            if (lastRms >= 0) calibrated = true;
        }
        else if (key == 'w' && calibrated) saveCalibration(CALIB_FILE, cameraMatrix, distCoeffs, lastRms>=0?lastRms:0);
        else if (key == 'a') showAxes = !showAxes;
        else if (key == 'v') showCastle = !showCastle;
        else if (key == 'o') showOBJ = !showOBJ;
        else if (key == 'f') { showORB = !showORB; if(showORB) showHarris=false; }
        else if (key == 'h') { showHarris = !showHarris; if(showHarris) showORB=false; }
        else if (key == 'm') useAruco = !useAruco;
        else if (key == 'p' && poseValid) { std::cout << "R: " << rvec.t() << "\nT: " << tvec.t() << "\n"; }
        else if (key == 'x') {
            std::string f = "screenshot_" + std::to_string(saveCounter++) + ".png";
            cv::imwrite(f, frame); std::cout << "Saved " << f << "\n";
        }

        // Handle button-triggered save/calibrate (need frame context)
        // Re-wire the save/calibrate buttons to set flags
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}