/*
  gui_opencv.h
  CS 5330 - Project 4
  OpenCV-only sidebar GUI for the Calibration & AR application.

  Layout:
    ┌──────────┬───────────────────────┐
    │ Sidebar  │   Camera feed         │
    │ (220px)  │                       │
    │          ├───────────────────────┤
    │          │   Status panel        │
    └──────────┴───────────────────────┘
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <functional>

// ── Colour palette ──────────────────────────────────────────────────────────
namespace GuiColors {
    const cv::Scalar BG_DARK   (30,  30,  35);
    const cv::Scalar BG_SIDEBAR(42,  42,  48);
    const cv::Scalar BG_PANEL  (50,  50,  56);
    const cv::Scalar BTN_BASE  (70,  72,  78);
    const cv::Scalar BTN_ON    (35, 140,  55);   // green  – toggle active
    const cv::Scalar BTN_ACT   (40,  90, 160);   // blue   – action button
    const cv::Scalar BTN_WARN  (40,  80, 180);   // orange-ish for capture
    const cv::Scalar TEXT_HI   (240,240,240);
    const cv::Scalar TEXT_DIM  (155,155,165);
    const cv::Scalar TEXT_OK   ( 60, 210,  80);
    const cv::Scalar TEXT_WARN (  0, 140, 255);
    const cv::Scalar TEXT_ERR  ( 60,  60, 230);
    // Section header accents
    const cv::Scalar ACC_CALIB (  0, 200, 255);  // cyan
    const cv::Scalar ACC_AR    (  0, 220, 140);  // teal
    const cv::Scalar ACC_FEAT  (200, 200,  60);  // yellow
    const cv::Scalar ACC_MODE  (200, 100, 220);  // purple
    const cv::Scalar ACC_TRACK (220, 160,  40);  // amber
    const cv::Scalar ACC_TOOLS (160, 160, 170);  // grey
}

// ── Button descriptor ────────────────────────────────────────────────────────
struct GuiButton {
    cv::Rect   rect;
    std::string label;
    bool        isToggle  = false;
    bool        toggled   = false;
    cv::Scalar  baseColor;
    std::function<void()> onClick;
};

// ── Main GUI class ───────────────────────────────────────────────────────────
class CVGUI {
public:
    // Dimensions
    static const int SIDEBAR_W = 220;
    static const int STATUS_H  = 140;
    static const int BTN_H     = 30;
    static const int BTN_PAD   = 5;

    CVGUI() = default;

    // Must be called once before the main loop.
    // Callbacks:
    //   onSave          – save calibration frame (needs frame context: use a flag)
    //   onCalibrate     – run calibration        (needs frame context: use a flag)
    //   onWrite         – write calibration to file
    //   onToggleAxes    – toggle 3D axes
    //   onToggleCastle  – toggle castle
    //   onToggleOBJ     – toggle OBJ model
    //   onToggleDisguise– toggle target disguise (extension)
    //   onToggleORB     – toggle ORB feature display
    //   onToggleHarris  – toggle Harris display
    //   onToggleAruco   – toggle ArUco mode
    //   onCaptureRef    – capture ORB tracking reference (extension)
    //   onToggleTrack   – toggle ORB tracking mode (extension)
    //   onPrintPose     – print pose to terminal
    //   onScreenshot    – save screenshot (needs frame context: use a flag)
    void setupButtons(
        std::function<void()> onSave,
        std::function<void()> onCalibrate,
        std::function<void()> onWrite,
        std::function<void()> onToggleAxes,
        std::function<void()> onToggleCastle,
        std::function<void()> onToggleOBJ,
        std::function<void()> onToggleDisguise,
        std::function<void()> onToggleORB,
        std::function<void()> onToggleHarris,
        std::function<void()> onToggleAruco,
        std::function<void()> onCaptureRef,
        std::function<void()> onToggleTrack,
        std::function<void()> onPrintPose,
        std::function<void()> onScreenshot
    );

    // Sync toggle button visuals with current app state.
    void updateToggles(bool axes, bool castle, bool obj, bool disguise,
                       bool orb, bool harris, bool aruco, bool track);

    // Build the full composite display image.
    cv::Mat buildDisplay(const cv::Mat &cameraFrame,
                         const std::vector<std::string> &statusLines);

    // Returns true if the click hit a button.
    bool handleClick(int x, int y);

private:
    std::vector<GuiButton> buttons_;

    void drawButton    (cv::Mat &c, const GuiButton &b) const;
    void drawSection   (cv::Mat &c, const std::string &title,
                        int y, cv::Scalar accent) const;
    void drawStatusPanel(cv::Mat &c, const std::vector<std::string> &lines,
                         int x, int y, int w, int h) const;
    void drawSidebar   (cv::Mat &c) const;
};
