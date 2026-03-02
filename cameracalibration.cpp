#include "CameraCalibration.h"
#include <fstream>

void calibrateCamera(const std::vector<std::vector<cv::Point2f>>& corner_list,
    const std::vector<std::vector<cv::Vec3f>>& point_list,
    const cv::Size& imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs) {
    if (corner_list.size() >= 5) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 2) = imageSize.width / 2;
        cameraMatrix.at<double>(1, 2) = imageSize.height / 2;

        distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        double rms = cv::calibrateCamera(point_list, corner_list, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        std::cout << "Camera matrix: " << cameraMatrix << std::endl;
        std::cout << "Distortion coefficients: " << distCoeffs << std::endl;
        std::cout << "Re-projection error: " << rms << std::endl;
    }
}

bool saveCalibrationData(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoefficients, double reProjectionError) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << "Camera Matrix:" << std::endl << cameraMatrix << std::endl;
    outFile << "Distortion Coefficients:" << std::endl << distCoefficients << std::endl;
    outFile << "Re-Projection Error: " << reProjectionError << std::endl;
    outFile.close();
    return true;
}