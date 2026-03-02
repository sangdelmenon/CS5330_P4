#include "CameraCalibration.h"
#include "ChessboardDetection.h"
#include "AugmentedReality.h"
#include "FeatureDetection.h"
#include "ModelLoader.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>
#include <atomic>
#include <regex>

std::atomic<char> keyPressed(' ');
std::atomic<bool> displayVirtualObjectPersistent(false);
std::atomic<bool> displayFeatures(false);

void captureKeyInput() {
    char key;
    while (true) {
        std::cin >> key;
        keyPressed.store(key);
        if (key == 'q') break;
    }
}

bool readCalibrationData(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoefficients) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    std::string line;
    std::regex number_regex("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?");
    std::smatch match;

    while (std::getline(file, line)) {
        if (line.find("Camera Matrix:") != std::string::npos) {
            for (int i = 0; i < 3; ++i) {
                std::getline(file, line);
                int j = 0;
                while (std::regex_search(line, match, number_regex) && j < 3) {
                    cameraMatrix.at<double>(i, j) = std::stod(match.str(0));
                    line = match.suffix().str();
                    j++;
                }
            }
        }
        else if (line.find("Distortion Coefficients:") != std::string::npos) {
            for (int i = 0; i < 5; ++i) {
                std::getline(file, line);
                if (std::regex_search(line, match, number_regex)) {
                    distCoefficients.at<double>(i, 0) = std::stod(match.str(0));
                }
            }
        }
    }
    file.close();
    return true;
}

std::vector<cv::Point3f> defineAxesPoints() {
    std::vector<cv::Point3f> axesPoints;
    axesPoints.push_back(cv::Point3f(0, 0, 0));
    axesPoints.push_back(cv::Point3f(3, 0, 0));
    axesPoints.push_back(cv::Point3f(0, 3, 0));
    axesPoints.push_back(cv::Point3f(0, 0, -3));
    return axesPoints;
}

bool isInsideRectangle(const cv::Point2f& p, const std::vector<cv::Point2f>& rect) {
    double area1 = cv::contourArea(rect);
    double area2 = cv::contourArea(std::vector<cv::Point2f>{rect[0], rect[1], p}) +
        cv::contourArea(std::vector<cv::Point2f>{rect[1], rect[2], p}) +
        cv::contourArea(std::vector<cv::Point2f>{rect[2], rect[3], p}) +
        cv::contourArea(std::vector<cv::Point2f>{rect[3], rect[0], p});
    return std::abs(area1 - area2) < 1e-3;
}

void drawVirtualObject(cv::Mat& frame, const cv::Mat& cameraMatrix, const cv::Mat& distCoefficients, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point2f>& chessboardCorners, const cv::Size& patternSize) {
    std::vector<cv::Point2f> chessboardBoundary = {
        chessboardCorners.front(),
        chessboardCorners[patternSize.width - 1],
        chessboardCorners.back(),
        chessboardCorners[patternSize.width * (patternSize.height - 1)]
    };

    float baseSize = 0.5f;
    float height = 0.5f;

    float boardCenterX = patternSize.width / 2.0f - 0.5f;
    float boardCenterY = -(patternSize.height / 2.0f - 0.5f);

    float offsetX = boardCenterX;
    float offsetY = boardCenterY;

    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(-0.5f + offsetX, -0.5f + offsetY, 0),
        cv::Point3f(0.5f + offsetX, -0.5f + offsetY, 0),
        cv::Point3f(0.5f + offsetX, 0.5f + offsetY, 0),
        cv::Point3f(-0.5f + offsetX, 0.5f + offsetY, 0),
        cv::Point3f(offsetX, offsetY, height)
    };

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoefficients, imagePoints);

    for (const auto& point : imagePoints) {
        if (!isInsideRectangle(point, chessboardBoundary)) {
            baseSize *= 0.75f;
            height *= 0.75f;
            objectPoints = {
                cv::Point3f(-baseSize + offsetX, -baseSize + offsetY, 0),
                cv::Point3f(baseSize + offsetX, -baseSize + offsetY, 0),
                cv::Point3f(baseSize + offsetX, baseSize + offsetY, 0),
                cv::Point3f(-baseSize + offsetX, baseSize + offsetY, 0),
                cv::Point3f(offsetX, offsetY, height)
            };
            cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoefficients, imagePoints);
            break;
        }
    }

    for (int i = 0; i < 4; ++i) {
        cv::line(frame, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 255, 0), 5);
        cv::line(frame, imagePoints[i], imagePoints[4], cv::Scalar(0, 255, 0), 5);
    }

    for (int i = 0; i < 4; ++i) {
        cv::line(frame, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 0, 255), 10);
    }
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    std::thread keyInputThread(captureKeyInput);

    cv::Size patternSize(9, 6);
    std::vector<cv::Point2f> corner_set;
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    cv::Mat frame, cameraMatrix = cv::Mat::eye(3, 3, CV_64F), distCoefficients = cv::Mat::zeros(8, 1, CV_64F);
    bool foundPreviously = false;

    // Use relative path for calibration data
    std::string calibrationFilePath = "calibration_data.csv";
    if (!readCalibrationData(calibrationFilePath, cameraMatrix, distCoefficients)) {
        std::cerr << "Warning: Could not read calibration data. Starting fresh." << std::endl;
    }

    // Load 3D model (relative path)
    std::vector<Vertex> vertices;
    std::vector<TextureCoord> textures;
    std::vector<Normal> normals;
    std::vector<Face> faces;
    std::string modelPath = "Lowpoly_tree_sample2.obj";
    bool modelLoaded = loadOBJModel(modelPath, vertices, textures, normals, faces);

    bool display3DAxes = false;
    bool displayVirtualObject = false;

    std::cout << "Camera Matrix:\n" << cameraMatrix << "\n";
    std::cout << "Distortion Coefficients:\n" << distCoefficients << "\n";
    std::cout << "Press 's' to save calibration image.\nPress 'c' to perform calibration.\nPress 'p' to print board's pose.\nPress 'd' to display the virtual object.\nPress 'f' for robust features.\nPress 'q' to exit.\n";

    std::vector<cv::Point3f> axesPoints = defineAxesPoints();
    cv::Mat rvec, tvec;
    bool solvePnP_success = false;
    std::vector<cv::Vec3f> objectPoints;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        bool found = findChessboardCorners(frame, patternSize, corner_set);
        if (found) {
            objectPoints.clear();
            for (int i = 0; i < patternSize.height; ++i) {
                for (int j = 0; j < patternSize.width; ++j) {
                    objectPoints.push_back(cv::Vec3f(j, -i, 0.0f));
                }
            }

            solvePnP_success = cv::solvePnP(objectPoints, corner_set, cameraMatrix, distCoefficients, rvec, tvec);
            if (!foundPreviously) {
                std::cout << "Number of corners found: " << corner_set.size() << std::endl;
            }
            foundPreviously = true;
        }
        else {
            foundPreviously = false;
        }

        if (found && display3DAxes && solvePnP_success) {
            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoefficients, imagePoints);
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
            cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
        }

        if (found && displayVirtualObject && solvePnP_success && modelLoaded) {
            std::vector<cv::Point2f> modelImagePoints;
            for (const Vertex& vertex : vertices) {
                std::vector<cv::Point3f> singlePoint = { cv::Point3f(vertex.x, vertex.y, vertex.z) };
                std::vector<cv::Point2f> projectedPoint;
                cv::projectPoints(singlePoint, rvec, tvec, cameraMatrix, distCoefficients, projectedPoint);
                modelImagePoints.push_back(projectedPoint[0]);
            }
            for (const auto& point : modelImagePoints) {
                cv::circle(frame, point, 2, cv::Scalar(0, 255, 0), -1);
            }
            for (const auto& face : faces) {
                for (size_t i = 0; i < face.vertexIndices.size(); i++) {
                    cv::Point2f p1 = modelImagePoints[face.vertexIndices[i] - 1];
                    cv::Point2f p2 = modelImagePoints[face.vertexIndices[(i + 1) % face.vertexIndices.size()] - 1];
                    cv::line(frame, p1, p2, cv::Scalar(255, 0, 0), 1);
                }
            }
        }

        char key = keyPressed.load();
        if (key != ' ') {
            if (key == 's' && found) {
                corner_list.push_back(corner_set);
                std::vector<cv::Vec3f> point_set;
                for (int i = 0; i < patternSize.height; ++i) {
                    for (int j = 0; j < patternSize.width; ++j) {
                        point_set.push_back(cv::Vec3f(j, -i, 0.0f));
                    }
                }
                point_list.push_back(point_set);
                std::cout << "Saved calibration image with " << corner_set.size() << " corners." << std::endl;
            }
            else if (key == 'c') {
                if (corner_list.size() >= 5) {
                    std::vector<cv::Mat> rvecs, tvecs;
                    double reProjectionError = cv::calibrateCamera(point_list, corner_list, frame.size(), cameraMatrix, distCoefficients, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);
                    std::cout << "Calibration done with re-projection error: " << reProjectionError << std::endl;
                    saveCalibrationData(calibrationFilePath, cameraMatrix, distCoefficients, reProjectionError);
                }
                else {
                    std::cerr << "Not enough calibration images. Need at least 5." << std::endl;
                }
            }
            if (key == 'd') displayVirtualObjectPersistent.store(true);
            if (key == 'p') display3DAxes = !display3DAxes;
            if (key == 'o') displayVirtualObject = !displayVirtualObject;
            if (key == 'f') displayFeatures = !displayFeatures;

            keyPressed.store(' ');
        }

        if (displayVirtualObjectPersistent.load() && solvePnP_success && found) {
            drawVirtualObject(frame, cameraMatrix, distCoefficients, rvec, tvec, corner_set, patternSize);
        }

        if (displayFeatures.load()) {
            detectAndDrawFeatures(frame);
        }

        cv::imshow("Frame", frame);
        cv::waitKey(1);
        if (key == 'q') break;
    }

    if (keyInputThread.joinable()) keyInputThread.join();
    cap.release();
    cv::destroyAllWindows();
    return 0;
}