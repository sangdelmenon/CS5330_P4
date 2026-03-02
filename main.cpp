/*
  main.cpp
  CS 5330 - Project 4: Calibration and Augmented Reality

  Key bindings:
    s       save current frame for calibration
    c       run calibration (need >= 5 frames)
    w       write calibration to file
    a       toggle 3D axes display
    v       toggle virtual castle display
    o       toggle OBJ model display
    p       print current pose (rotation + translation)
    f       toggle ORB feature detection
    h       toggle Harris corner detection
    m       toggle ArUco marker mode (vs chessboard)
    x       save screenshot
    q/ESC   quit
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

namespace fs = std::filesystem;

// ─── Configuration ─────────────────────────────────────────────────────────
static const cv::Size PATTERN_SIZE(9, 6);  // 9 cols x 6 rows of internal corners
static const std::string CALIB_FILE = "calibration.yml";
static const std::string MODEL_FILE = "Lowpoly_tree_sample2.obj";

// ─── Main ──────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    // ── Open video source ──
    cv::VideoCapture cap;
    if (argc > 1) {
        // If argument given, try as video file or image
        cap.open(argv[1]);
    } else {
        cap.open(0); // webcam
    }
    if (!cap.isOpened()) {
        std::cerr << "Error: cannot open video source.\n";
        return 1;
    }

    // ── Load existing calibration if available ──
    cv::Mat cameraMatrix, distCoeffs;
    bool calibrated = loadCalibration(CALIB_FILE, cameraMatrix, distCoeffs);
    if (!calibrated) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        std::cout << "No calibration found. Start by pressing 's' to save frames, then 'c' to calibrate.\n";
    }

    // ── Load OBJ model (optional) ──
    std::vector<Vertex> objVertices;
    std::vector<TextureCoord> objTextures;
    std::vector<Normal> objNormals;
    std::vector<Face> objFaces;
    bool modelLoaded = false;
    if (fs::exists(MODEL_FILE)) {
        modelLoaded = loadOBJModel(MODEL_FILE, objVertices, objTextures, objNormals, objFaces);
        if (modelLoaded)
            std::cout << "OBJ model loaded: " << objVertices.size() << " vertices.\n";
    }

    // ── Calibration data ──
    std::vector<std::vector<cv::Point2f>> cornerList;
    std::vector<std::vector<cv::Vec3f>> pointList;
    std::vector<cv::Vec3f> worldPoints = generateChessboardPoints(PATTERN_SIZE);

    // ── State flags ──
    bool showAxes     = false;
    bool showCastle   = false;
    bool showOBJ      = false;
    bool showORB      = false;
    bool showHarris   = false;
    bool useAruco     = false;
    int  saveCounter  = 0;
    double lastRms    = -1;

    // ── Pose ──
    cv::Mat rvec, tvec;
    bool poseValid = false;

    std::cout << "\n";
    std::cout << "=== CS 5330 Project 4: Calibration & AR ===\n";
    std::cout << "Keys:\n";
    std::cout << "  s   save calibration frame\n";
    std::cout << "  c   run calibration\n";
    std::cout << "  w   write calibration to file\n";
    std::cout << "  a   toggle 3D axes\n";
    std::cout << "  v   toggle virtual castle\n";
    std::cout << "  o   toggle OBJ model\n";
    std::cout << "  p   print pose\n";
    std::cout << "  f   toggle ORB features\n";
    std::cout << "  h   toggle Harris corners\n";
    std::cout << "  m   toggle ArUco mode\n";
    std::cout << "  x   save screenshot\n";
    std::cout << "  q   quit\n\n";

    // ─── Main loop ─────────────────────────────────────────────────────────
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        std::vector<cv::Point2f> corners;
        bool targetFound = false;

        // ── Detect target ──
        if (useAruco) {
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<int> markerIds;
            targetFound = detectAruco(frame, markerCorners, markerIds);

            // Use first detected marker for pose estimation
            if (targetFound && !markerIds.empty()) {
                // ArUco marker has 4 corners
                corners = markerCorners[0];

                // World points for a single ArUco marker (1x1 square)
                std::vector<cv::Point3f> markerWorld = {
                    {0, 0, 0}, {1, 0, 0}, {1, -1, 0}, {0, -1, 0}
                };

                if (calibrated) {
                    poseValid = cv::solvePnP(markerWorld, corners,
                                             cameraMatrix, distCoeffs, rvec, tvec);
                }
            }
        } else {
            targetFound = detectChessboard(frame, PATTERN_SIZE, corners);

            if (targetFound && calibrated) {
                poseValid = cv::solvePnP(worldPoints, corners,
                                         cameraMatrix, distCoeffs, rvec, tvec);
            }
        }

        // ── AR overlays (only when pose is valid) ──
        if (targetFound && poseValid && calibrated) {
            if (showAxes) {
                draw3DAxes(frame, cameraMatrix, distCoeffs, rvec, tvec);
            }

            if (!useAruco) {
                drawOutsideCorners(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);
            }

            if (showCastle && !useAruco) {
                drawCastle(frame, cameraMatrix, distCoeffs, rvec, tvec, PATTERN_SIZE);
            }

            if (showOBJ && modelLoaded) {
                // Project OBJ model vertices
                std::vector<cv::Point2f> imgPts;
                for (auto &v : objVertices) {
                    std::vector<cv::Point3f> pt3 = {{v.x, v.y, v.z}};
                    std::vector<cv::Point2f> pt2;
                    cv::projectPoints(pt3, rvec, tvec, cameraMatrix, distCoeffs, pt2);
                    imgPts.push_back(pt2[0]);
                }
                // Draw edges
                for (auto &face : objFaces) {
                    for (size_t i = 0; i < face.vertexIndices.size(); i++) {
                        int a = face.vertexIndices[i] - 1;
                        int b = face.vertexIndices[(i + 1) % face.vertexIndices.size()] - 1;
                        if (a >= 0 && a < (int)imgPts.size() && b >= 0 && b < (int)imgPts.size())
                            cv::line(frame, imgPts[a], imgPts[b], cv::Scalar(0, 180, 0), 1);
                    }
                }
            }
        }

        // ── Feature detection overlays ──
        if (showORB)    detectORBFeatures(frame);
        if (showHarris) detectHarrisCorners(frame);

        // ── HUD ──
        std::string status = useAruco ? "ArUco" : "Chessboard";
        status += targetFound ? " [FOUND]" : " [not found]";
        status += "  Calib frames: " + std::to_string(cornerList.size());
        if (calibrated) status += "  [CALIBRATED]";
        cv::putText(frame, status, cv::Point(10, frame.rows - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

        cv::imshow("Project 4: Calibration & AR", frame);

        // ── Key handling ──
        int key = cv::waitKey(30) & 0xFF;

        if (key == 'q' || key == 27) break;

        else if (key == 's') {
            if (targetFound && !useAruco) {
                cornerList.push_back(corners);
                pointList.push_back(worldPoints);
                std::cout << "Saved calibration frame #" << cornerList.size()
                          << " (" << corners.size() << " corners)\n";
            } else if (!targetFound) {
                std::cout << "Target not found — cannot save.\n";
            } else {
                std::cout << "Save calibration only works in chessboard mode.\n";
            }
        }
        else if (key == 'c') {
            lastRms = runCalibration(cornerList, pointList, frame.size(),
                                     cameraMatrix, distCoeffs);
            if (lastRms >= 0) calibrated = true;
        }
        else if (key == 'w') {
            if (calibrated) {
                saveCalibration(CALIB_FILE, cameraMatrix, distCoeffs,
                                lastRms >= 0 ? lastRms : 0);
            } else {
                std::cout << "Not calibrated yet.\n";
            }
        }
        else if (key == 'a') {
            showAxes = !showAxes;
            std::cout << "3D Axes: " << (showAxes ? "ON" : "OFF") << "\n";
        }
        else if (key == 'v') {
            showCastle = !showCastle;
            std::cout << "Castle: " << (showCastle ? "ON" : "OFF") << "\n";
        }
        else if (key == 'o') {
            showOBJ = !showOBJ;
            std::cout << "OBJ model: " << (showOBJ ? "ON" : "OFF") << "\n";
        }
        else if (key == 'p') {
            if (poseValid) {
                std::cout << "Rotation:    " << rvec.t() << "\n";
                std::cout << "Translation: " << tvec.t() << "\n";
            } else {
                std::cout << "No valid pose.\n";
            }
        }
        else if (key == 'f') {
            showORB = !showORB;
            if (showORB) showHarris = false; // only one feature type at a time
            std::cout << "ORB features: " << (showORB ? "ON" : "OFF") << "\n";
        }
        else if (key == 'h') {
            showHarris = !showHarris;
            if (showHarris) showORB = false;
            std::cout << "Harris corners: " << (showHarris ? "ON" : "OFF") << "\n";
        }
        else if (key == 'm') {
            useAruco = !useAruco;
            std::cout << "Mode: " << (useAruco ? "ArUco markers" : "Chessboard") << "\n";
        }
        else if (key == 'x') {
            std::string fname = "screenshot_" + std::to_string(saveCounter++) + ".png";
            cv::imwrite(fname, frame);
            std::cout << "Saved " << fname << "\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}