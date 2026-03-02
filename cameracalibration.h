#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

void calibrateCamera(const std::vector<std::vector<cv::Point2f>>& corner_list,
    const std::vector<std::vector<cv::Vec3f>>& point_list,
    const cv::Size& imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs);

bool saveCalibrationData(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoefficients, double reProjectionError);