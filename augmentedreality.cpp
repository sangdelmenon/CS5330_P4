#include "AugmentedReality.h"

void project3DAxes(const cv::Mat& frame, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    std::vector<cv::Point3f> axis = { {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, -1} };
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 5);
    cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 5);
    cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 5);
}