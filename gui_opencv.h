
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <functional>

struct Button {
    cv::Rect rect;
    std::string label;
    cv::Scalar bgColor;
    cv::Scalar activeColor;
    bool toggled = false;
    bool isToggle = false;
    std::function<void()> onClick;
};

class CVGUI {
public:
    CVGUI();

    cv::Mat buildDisplay(const cv::Mat &cameraFrame,
                         const std::vector<std::string> &statusLines);

    bool handleClick(int x, int y);

    void setupButtons(
        std::function<void()> onSave,
        std::function<void()> onCalibrate,
        std::function<void()> onWriteCal,
        std::function<void()> onToggleAxes,
        std::function<void()> onToggleCastle,
        std::function<void()> onToggleOBJ,
        std::function<void()> onPrintPose,
        std::function<void()> onToggleORB,
        std::function<void()> onToggleHarris,
        std::function<void()> onToggleAruco,
        std::function<void()> onScreenshot
    );

    void updateToggles(bool axes, bool castle, bool obj, bool orb, bool harris, bool aruco);

private:
    std::vector<Button> buttons;

    static const int SIDEBAR_W  = 200;
    static const int BTN_H      = 36;
    static const int BTN_PAD    = 6;
    static const int STATUS_H   = 100;

    void drawButton(cv::Mat &canvas, const Button &btn);
    void drawStatusPanel(cv::Mat &canvas, const std::vector<std::string> &lines,
                         int x, int y, int w, int h);
};