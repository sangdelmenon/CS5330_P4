/*
chessboarddetection.cpp
  CS 5330 - Project 4
  Target detection: chessboard and ArUco markers.
*/
#include "chessboarddetection.h"

// ─── Chessboard detection ──────────────────────────────────────────────────
bool detectChessboard(cv::Mat &frame, const cv::Size &patternSize,
                      std::vector<cv::Point2f> &corners)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, patternSize, corners,
                     cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found && !corners.empty()) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        cv::drawChessboardCorners(frame, patternSize, cv::Mat(corners), found);
    }
    return found;
}

// ─── ArUco marker detection ────────────────────────────────────────────────
bool detectAruco(cv::Mat &frame,
                 std::vector<std::vector<cv::Point2f>> &markerCorners,
                 std::vector<int> &markerIds)
{
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    detector.detectMarkers(frame, markerCorners, markerIds);

    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        return true;
    }
    return false;
}

// ─── Generate 3D world points for chessboard ───────────────────────────────
std::vector<cv::Vec3f> generateChessboardPoints(const cv::Size &patternSize)
{
    std::vector<cv::Vec3f> points;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            points.push_back(cv::Vec3f(j, -i, 0.0f));
        }
    }
    return points;
}