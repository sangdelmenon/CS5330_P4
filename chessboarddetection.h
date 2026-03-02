/*
  chessboarddetection.h
  CS 5330 - Project 4: Calibration and Augmented Reality
  Target detection: chessboard pattern and ArUco markers.
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

// Chessboard corner detection with sub-pixel refinement.
// Draws corners on frame if found.
bool detectChessboard(cv::Mat &frame, const cv::Size &patternSize,
                      std::vector<cv::Point2f> &corners);

// ArUco marker detection. Returns true if at least one marker found.
// Draws detected markers on frame.
bool detectAruco(cv::Mat &frame,
                 std::vector<std::vector<cv::Point2f>> &markerCorners,
                 std::vector<int> &markerIds);

// Generate the 3D world point set for the chessboard.
// Origin at top-left corner, Z=0, units = squares.
std::vector<cv::Vec3f> generateChessboardPoints(const cv::Size &patternSize);