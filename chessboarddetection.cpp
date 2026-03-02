#include "ChessboardDetection.h"

bool findChessboardCorners(const cv::Mat& frame, const cv::Size& patternSize, std::vector<cv::Point2f>& corner_set) {
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(grayFrame, patternSize, corner_set);
    if (found && !corner_set.empty()) {
        cv::cornerSubPix(grayFrame, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        cv::drawChessboardCorners(frame, patternSize, cv::Mat(corner_set), found);
    }
    return found;
}