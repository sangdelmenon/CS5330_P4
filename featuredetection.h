/*
featuredetection.h
  CS 5330 - Project 4
  Robust feature detection: ORB and Harris corners.
*/
#pragma once
#include <opencv2/opencv.hpp>

// Detect and draw ORB features on the frame.
void detectORBFeatures(cv::Mat &frame);

// Detect and draw Harris corners on the frame.
void detectHarrisCorners(cv::Mat &frame, double threshold = 150.0);