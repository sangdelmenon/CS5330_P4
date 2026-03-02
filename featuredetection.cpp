#include "FeatureDetection.h"

void detectAndDrawFeatures(cv::Mat& frame) {
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Detect features using ORB
    detector->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);

    // Draw the keypoints onto the frame
    cv::drawKeypoints(frame, keypoints, frame, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
}