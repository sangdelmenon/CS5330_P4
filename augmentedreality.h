#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

void project3DAxes(const cv::Mat& frame, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Vec3d& rvec, const cv::Vec3d& tvec);