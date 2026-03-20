/*
  orbtracking.h
  CS 5330 - Project 4
  Uber Extension 2: ORB feature-based planar AR tracking.

  Usage:
    1. Call setReference(frame) to train on a reference image.
    2. Call track(frame, ...) each loop iteration to estimate pose.
    3. If track() returns true, use the returned rvec/tvec with
       any AR overlay function (draw3DAxes, drawCastle, etc.).
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

struct ORBTracker {
    cv::Mat  refGray;                      // greyscale reference image
    std::vector<cv::KeyPoint> refKps;      // reference keypoints
    cv::Mat  refDesc;                      // reference descriptors

    // World-space size of the reference plane (in "board square" units).
    // Adjust to control how big the AR overlay appears relative to the target.
    float worldWidth  = 4.0f;
    float worldHeight = 3.0f;

    bool hasReference  = false;
    int  lastInliers   = 0;   // set by track(), useful for HUD

    // Set the reference image from a live frame.
    bool setReference(const cv::Mat &bgrFrame);

    // Set the reference image from a file (e.g. "reference.png").
    bool setReferenceFromFile(const std::string &filename);

    // Estimate pose of the reference plane in the current frame.
    // Returns true when enough inlier matches were found.
    // rvec and tvec are set in the same convention as chessboard pose.
    bool track(const cv::Mat &bgrFrame,
               const cv::Mat &cameraMatrix,
               const cv::Mat &distCoeffs,
               cv::Mat &rvec, cv::Mat &tvec);
};
