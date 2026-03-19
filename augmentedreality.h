/*
augmentedreality.h
  CS 5330 - Project 4
  3D projection: axes, corners, and virtual objects.
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// Draw 3D coordinate axes at the origin (length = 3 squares).
void draw3DAxes(cv::Mat &frame, const cv::Mat &cameraMatrix,
                const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec);

// Draw the projected outside corners of the chessboard.
void drawOutsideCorners(cv::Mat &frame, const cv::Mat &cameraMatrix,
                        const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                        const cv::Size &patternSize);

// Draw a creative multi-color castle floating above the board.
void drawCastle(cv::Mat &frame, const cv::Mat &cameraMatrix,
                const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                const cv::Size &patternSize);

// Helper: project a set of 3D points and draw lines between consecutive pairs.
void drawWireframe(cv::Mat &frame, const std::vector<cv::Point3f> &pts3d,
                   const std::vector<std::pair<int,int>> &edges,
                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                   const cv::Mat &rvec, const cv::Mat &tvec,
                   cv::Scalar colour, int thickness = 2);

// Extension: disguise the chessboard by painting a semi-transparent coloured
// overlay over the board area so it no longer looks like a calibration target.
void drawTargetDisguise(cv::Mat &frame, const cv::Mat &cameraMatrix,
                        const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                        const cv::Size &patternSize);