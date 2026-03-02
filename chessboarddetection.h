#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

bool findChessboardCorners(const cv::Mat& frame, const cv::Size& patternSize, std::vector<cv::Point2f>& corner_set);