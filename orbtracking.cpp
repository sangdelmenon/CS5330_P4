/*
  orbtracking.cpp
  CS 5330 - Project 4
  Uber Extension 2: ORB feature-based planar AR tracking.

  Algorithm:
    - Train on a reference image: extract 1000 ORB keypoints + descriptors.
    - Each frame: extract ORB keypoints + descriptors.
    - Match with BFMatcher (Hamming, cross-check) and filter by distance.
    - Build 3D-2D correspondences:
        reference pixel (px, py) in image of size (W, H)
        → world point   (px/W * worldWidth, -py/H * worldHeight, 0)
        → image point   corresponding keypoint in the current frame
    - solvePnPRansac to get rvec / tvec.
*/
#include "orbtracking.h"
#include <iostream>

// ─── Internal helpers ──────────────────────────────────────────────────────

static cv::Ptr<cv::ORB> makeORB()
{
    return cv::ORB::create(2000);
}

// ─── setReference ──────────────────────────────────────────────────────────

bool ORBTracker::setReference(const cv::Mat &bgrFrame)
{
    if (bgrFrame.empty()) return false;

    cv::cvtColor(bgrFrame, refGray, cv::COLOR_BGR2GRAY);

    auto orb = makeORB();
    orb->detectAndCompute(refGray, cv::noArray(), refKps, refDesc);

    hasReference = !refKps.empty() && !refDesc.empty();
    if (hasReference)
        std::cout << "ORB reference captured: " << refKps.size() << " keypoints.\n";
    else
        std::cerr << "ORB: no keypoints found in reference image.\n";

    return hasReference;
}

bool ORBTracker::setReferenceFromFile(const std::string &filename)
{
    cv::Mat img = cv::imread(filename);
    if (img.empty()) {
        std::cerr << "ORB: could not read reference file: " << filename << "\n";
        return false;
    }
    return setReference(img);
}

// ─── track ─────────────────────────────────────────────────────────────────

bool ORBTracker::track(const cv::Mat &bgrFrame,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs,
                       cv::Mat &rvec, cv::Mat &tvec)
{
    lastInliers = 0;
    if (!hasReference) return false;

    // Extract features from current frame
    cv::Mat gray;
    cv::cvtColor(bgrFrame, gray, cv::COLOR_BGR2GRAY);

    auto orb = makeORB();
    std::vector<cv::KeyPoint> frameKps;
    cv::Mat frameDesc;
    orb->detectAndCompute(gray, cv::noArray(), frameKps, frameDesc);

    if (frameDesc.empty() || (int)frameKps.size() < 15) return false;

    // Match: cross-check eliminates most false matches
    cv::BFMatcher matcher(cv::NORM_HAMMING, /*crossCheck=*/true);
    std::vector<cv::DMatch> matches;
    matcher.match(refDesc, frameDesc, matches);

    if ((int)matches.size() < 15) return false;

    // Keep matches within 2.0× the best distance (floor at 30 Hamming)
    double minDist = 1e9;
    for (auto &m : matches) minDist = std::min(minDist, (double)m.distance);
    double thresh = std::max(2.0 * minDist, 30.0);

    std::vector<cv::DMatch> good;
    for (auto &m : matches)
        if (m.distance <= thresh) good.push_back(m);

    if ((int)good.size() < 15) return false;

    // Build 3D ↔ 2D correspondences
    const int W = refGray.cols;
    const int H = refGray.rows;

    std::vector<cv::Point3f> pts3D;
    std::vector<cv::Point2f> pts2D;
    pts3D.reserve(good.size());
    pts2D.reserve(good.size());

    for (auto &m : good) {
        cv::Point2f rp = refKps[m.queryIdx].pt;
        cv::Point2f fp = frameKps[m.trainIdx].pt;

        // Map reference pixel → world plane (Z = 0)
        pts3D.push_back({ (rp.x / W) * worldWidth,
                          -(rp.y / H) * worldHeight,
                          0.0f });
        pts2D.push_back(fp);
    }

    // RANSAC-based PnP
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(pts3D, pts2D,
                                 cameraMatrix, distCoeffs,
                                 rvec, tvec,
                                 /*useExtrinsicGuess=*/false,
                                 /*iterationsCount=*/100,
                                 /*reprojectionError=*/5.0f,
                                 /*confidence=*/0.99,
                                 inliers);

    lastInliers = (int)inliers.size();
    return ok && lastInliers >= 12;
}
