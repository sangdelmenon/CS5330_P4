/*
cameracalibration.h
  CS 5330 - Project 4
  Camera calibration: run calibration, save/load intrinsics.
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Run camera calibration. Requires at least 5 calibration frames.
// Returns the re-projection error, or -1 on failure.
double runCalibration(const std::vector<std::vector<cv::Point2f>> &cornerList,
                      const std::vector<std::vector<cv::Vec3f>> &pointList,
                      const cv::Size &imageSize,
                      cv::Mat &cameraMatrix,
                      cv::Mat &distCoeffs);

// Save calibration to YAML file using cv::FileStorage (robust).
bool saveCalibration(const std::string &filename,
                     const cv::Mat &cameraMatrix,
                     const cv::Mat &distCoeffs,
                     double reprojError);

// Load calibration from YAML file.
bool loadCalibration(const std::string &filename,
                     cv::Mat &cameraMatrix,
                     cv::Mat &distCoeffs);