#include "gui_opencv.h"

static const cv::Scalar BG_DARK(40,40,45), BG_PANEL(55,55,60);
static const cv::Scalar BTN_NORMAL(80,80,85), BTN_ON(50,140,50), BTN_ACTION(140,90,40);
static const cv::Scalar TEXT_WHITE(240,240,240), TEXT_DIM(160,160,160), ACCENT(0,200,255);

CVGUI::CVGUI() {}

void CVGUI::setupButtons(
    std::function<void()> onSave, std::function<void()> onCalibrate,
    std::function<void()> onWriteCal, std::function<void()> onToggleAxes,
    std::function<void()> onToggleCastle, std::function<void()> onToggleOBJ,
    std::function<void()> onPrintPose, std::function<void()> onToggleORB,
    std::function<void()> onToggleHarris, std::function<void()> onToggleAruco,
    std::function<void()> onScreenshot)
{
    buttons.clear();
    int x = BTN_PAD, y = BTN_PAD + 20, w = SIDEBAR_W - 2 * BTN_PAD;
    auto add = [&](const std::string &lbl, bool tog, cv::Scalar bg, std::function<void()> cb) {
        Button b; b.rect = cv::Rect(x, y, w, BTN_H); b.label = lbl;
        b.bgColor = bg; b.activeColor = BTN_ON; b.isToggle = tog; b.onClick = cb;
        buttons.push_back(b); y += BTN_H + BTN_PAD;
    };
    add("Save Frame (s)", false, BTN_ACTION, onSave);
    add("Calibrate (c)", false, BTN_ACTION, onCalibrate);
    add("Write Calib (w)", false, BTN_ACTION, onWriteCal);
    y += 10;
    add("3D Axes (a)", true, BTN_NORMAL, onToggleAxes);
    add("Castle (v)", true, BTN_NORMAL, onToggleCastle);
    add("OBJ Model (o)", true, BTN_NORMAL, onToggleOBJ);
    y += 10;
    add("ORB Features (f)", true, BTN_NORMAL, onToggleORB);
    add("Harris Corners (h)", true, BTN_NORMAL, onToggleHarris);
    y += 10;
    add("ArUco Mode (m)", true, BTN_NORMAL, onToggleAruco);
    y += 10;
    add("Print Pose (p)", false, BTN_NORMAL, onPrintPose);
    add("Screenshot (x)", false, BTN_NORMAL, onScreenshot);
}

void CVGUI::updateToggles(bool axes, bool castle, bool obj, bool orb, bool harris, bool aruco) {
    for (auto &b : buttons) {
        if (b.label.find("3D Axes") == 0) b.toggled = axes;
        else if (b.label.find("Castle") == 0) b.toggled = castle;
        else if (b.label.find("OBJ") == 0) b.toggled = obj;
        else if (b.label.find("ORB") == 0) b.toggled = orb;
        else if (b.label.find("Harris") == 0) b.toggled = harris;
        else if (b.label.find("ArUco") == 0) b.toggled = aruco;
    }
}

void CVGUI::drawButton(cv::Mat &canvas, const Button &btn) {
    cv::Scalar bg = (btn.isToggle && btn.toggled) ? btn.activeColor : btn.bgColor;
    cv::rectangle(canvas, btn.rect, bg, cv::FILLED);
    cv::rectangle(canvas, btn.rect, cv::Scalar(100,100,100), 1);
    int baseline = 0;
    cv::Size tsz = cv::getTextSize(btn.label, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
    cv::Point org(btn.rect.x + (btn.rect.width - tsz.width) / 2,
                  btn.rect.y + (btn.rect.height + tsz.height) / 2);
    cv::putText(canvas, btn.label, org, cv::FONT_HERSHEY_SIMPLEX, 0.42, TEXT_WHITE, 1, cv::LINE_AA);
}

void CVGUI::drawStatusPanel(cv::Mat &canvas, const std::vector<std::string> &lines,
                            int x, int y, int w, int h) {
    cv::rectangle(canvas, cv::Rect(x, y, w, h), BG_PANEL, cv::FILLED);
    cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(80,80,80), 1);
    int ty = y + 18;
    for (auto &line : lines) {
        cv::Scalar col = TEXT_DIM;
        if (line.find("[OK]") != std::string::npos || line.find("FOUND") != std::string::npos)
            col = cv::Scalar(0, 220, 0);
        else if (line.find("[!]") != std::string::npos || line.find("NOT") != std::string::npos)
            col = cv::Scalar(0, 100, 255);
        cv::putText(canvas, line, cv::Point(x+8, ty), cv::FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv::LINE_AA);
        ty += 16;
    }
}

cv::Mat CVGUI::buildDisplay(const cv::Mat &cameraFrame, const std::vector<std::string> &statusLines) {
    int camW = cameraFrame.cols, camH = cameraFrame.rows;
    int totalW = SIDEBAR_W + camW, totalH = std::max(camH + STATUS_H, 580);
    cv::Mat display(totalH, totalW, CV_8UC3, BG_DARK);

    cv::putText(display, "CALIBRATION", cv::Point(BTN_PAD, BTN_PAD+14),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, ACCENT, 1, cv::LINE_AA);
    if (buttons.size() > 3)
        cv::putText(display, "DISPLAY", cv::Point(BTN_PAD, buttons[3].rect.y-6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, ACCENT, 1, cv::LINE_AA);
    if (buttons.size() > 6)
        cv::putText(display, "FEATURES", cv::Point(BTN_PAD, buttons[6].rect.y-6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, ACCENT, 1, cv::LINE_AA);
    if (buttons.size() > 8)
        cv::putText(display, "MODE", cv::Point(BTN_PAD, buttons[8].rect.y-6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, ACCENT, 1, cv::LINE_AA);
    if (buttons.size() > 9)
        cv::putText(display, "TOOLS", cv::Point(BTN_PAD, buttons[9].rect.y-6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, ACCENT, 1, cv::LINE_AA);

    for (auto &btn : buttons) drawButton(display, btn);
    if (camW > 0 && camH > 0)
        cameraFrame.copyTo(display(cv::Rect(SIDEBAR_W, 0, camW, camH)));
    drawStatusPanel(display, statusLines, SIDEBAR_W, camH, camW, STATUS_H);
    return display;
}

bool CVGUI::handleClick(int x, int y) {
    for (auto &btn : buttons) {
        if (btn.rect.contains(cv::Point(x, y))) {
            if (btn.isToggle) btn.toggled = !btn.toggled;
            if (btn.onClick) btn.onClick();
            return true;
        }
    }
    return false;
}