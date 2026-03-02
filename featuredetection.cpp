/*
featuredetection.cpp
  CS 5330 - Project 4
  ORB features and Harris corner detection.
*/
#include "featuredetection.h"

// ─── ORB Feature Detection ────────────────────────────────────────────────
void detectORBFeatures(cv::Mat &frame)
{
    cv::Ptr<cv::ORB> detector = cv::ORB::create(500);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);
    cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(0, 255, 0),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::putText(frame, "ORB: " + std::to_string(keypoints.size()) + " features",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
}

// ─── Harris Corner Detection ──────────────────────────────────────────────
void detectHarrisCorners(cv::Mat &frame, double threshold)
{
    cv::Mat gray, harrisResponse;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_32F);

    // Harris corner detection
    cv::cornerHarris(gray, harrisResponse, 2, 3, 0.04);

    // Normalise response for thresholding
    cv::Mat normResponse;
    cv::normalize(harrisResponse, normResponse, 0, 255, cv::NORM_MINMAX);

    int count = 0;
    for (int r = 0; r < normResponse.rows; r++) {
        for (int c = 0; c < normResponse.cols; c++) {
            if (normResponse.at<float>(r, c) > threshold) {
                cv::circle(frame, cv::Point(c, r), 4, cv::Scalar(0, 0, 255), 2);
                count++;
            }
        }
    }

    cv::putText(frame, "Harris: " + std::to_string(count) + " corners",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
}