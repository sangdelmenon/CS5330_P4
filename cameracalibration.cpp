/*
  cameracalibration.cpp
  CS 5330 - Project 4
  Camera calibration with FileStorage-based save/load.
*/
#include "cameracalibration.h"
#include <iostream>

// ─── Run calibration ───────────────────────────────────────────────────────
double runCalibration(const std::vector<std::vector<cv::Point2f>> &cornerList,
                      const std::vector<std::vector<cv::Vec3f>> &pointList,
                      const cv::Size &imageSize,
                      cv::Mat &cameraMatrix,
                      cv::Mat &distCoeffs)
{
    if (cornerList.size() < 5) {
        std::cerr << "Need at least 5 calibration images. Currently have "
                  << cornerList.size() << ".\n";
        return -1;
    }

    // Initialise camera matrix
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;

    // Distortion coefficients (5 parameters)
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(pointList, cornerList, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs,
                                     cv::CALIB_FIX_ASPECT_RATIO);

    std::cout << "\n=== Calibration Results ===\n";
    std::cout << "Camera Matrix:\n" << cameraMatrix << "\n\n";
    std::cout << "Distortion Coefficients:\n" << distCoeffs.t() << "\n\n";
    std::cout << "Re-projection Error: " << rms << " pixels\n";
    std::cout << "Calibration images used: " << cornerList.size() << "\n\n";

    return rms;
}

// ─── Save calibration using cv::FileStorage (YAML) ────────────────────────
bool saveCalibration(const std::string &filename,
                     const cv::Mat &cameraMatrix,
                     const cv::Mat &distCoeffs,
                     double reprojError)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Could not save calibration to " << filename << "\n";
        return false;
    }

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "reprojection_error" << reprojError;
    fs.release();

    std::cout << "Calibration saved to " << filename << "\n";
    return true;
}

// ─── Load calibration from YAML ────────────────────────────────────────────
bool loadCalibration(const std::string &filename,
                     cv::Mat &cameraMatrix,
                     cv::Mat &distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if (cameraMatrix.empty() || distCoeffs.empty()) return false;

    std::cout << "Calibration loaded from " << filename << "\n";
    std::cout << "Camera Matrix:\n" << cameraMatrix << "\n";
    return true;
}