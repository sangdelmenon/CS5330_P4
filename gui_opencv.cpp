/*
  gui_opencv.cpp
  CS 5330 - Project 4
  OpenCV-only sidebar GUI implementation.
*/
#include "gui_opencv.h"
#include <sstream>
#include <iomanip>

using namespace GuiColors;

// ── Internal layout helpers ──────────────────────────────────────────────────

// Draw a single button (filled rect + centred label).
void CVGUI::drawButton(cv::Mat &c, const GuiButton &b) const
{
    cv::Scalar bg = (b.isToggle && b.toggled) ? BTN_ON : b.baseColor;
    cv::rectangle(c, b.rect, bg, cv::FILLED);
    // border: brighter when active
    cv::Scalar border = (b.isToggle && b.toggled)
                        ? cv::Scalar(80, 200, 100) : cv::Scalar(90, 90, 100);
    cv::rectangle(c, b.rect, border, 1);

    // Centred label
    int base = 0;
    cv::Size ts = cv::getTextSize(b.label, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &base);
    cv::Point org(b.rect.x + (b.rect.width  - ts.width)  / 2,
                  b.rect.y + (b.rect.height + ts.height) / 2 - 1);
    cv::putText(c, b.label, org,
                cv::FONT_HERSHEY_SIMPLEX, 0.40, TEXT_HI, 1, cv::LINE_AA);
}

// Draw a section header line with a small accent bar.
void CVGUI::drawSection(cv::Mat &c, const std::string &title,
                        int y, cv::Scalar accent) const
{
    // Accent bar
    cv::rectangle(c, cv::Rect(BTN_PAD, y, 3, 14), accent, cv::FILLED);
    cv::putText(c, title, cv::Point(BTN_PAD + 7, y + 11),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, accent, 1, cv::LINE_AA);
}

// Draw the status panel below the camera feed.
void CVGUI::drawStatusPanel(cv::Mat &c,
                             const std::vector<std::string> &lines,
                             int x, int y, int w, int h) const
{
    cv::rectangle(c, cv::Rect(x, y, w, h), BG_PANEL, cv::FILLED);
    // Top separator
    cv::line(c, cv::Point(x, y), cv::Point(x + w, y), cv::Scalar(70,70,80), 1);

    // Title
    cv::putText(c, "STATUS", cv::Point(x + 8, y + 14),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, TEXT_DIM, 1, cv::LINE_AA);

    int ty = y + 30;
    for (auto &ln : lines) {
        cv::Scalar col = TEXT_DIM;
        if (ln.find("[OK]")   != std::string::npos) col = TEXT_OK;
        if (ln.find("FOUND")  != std::string::npos) col = TEXT_OK;
        if (ln.find("[!]")    != std::string::npos) col = TEXT_WARN;
        if (ln.find("ERR")    != std::string::npos) col = TEXT_ERR;
        if (ln.find("Track")  != std::string::npos && ln.find("inlier") != std::string::npos)
            col = TEXT_OK;

        cv::putText(c, ln, cv::Point(x + 8, ty),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv::LINE_AA);
        ty += 16;
        if (ty > y + h - 6) break;
    }
}

// Draw the sidebar background and title.
void CVGUI::drawSidebar(cv::Mat &c) const
{
    cv::rectangle(c, cv::Rect(0, 0, SIDEBAR_W, c.rows), BG_SIDEBAR, cv::FILLED);
    // Right border
    cv::line(c, cv::Point(SIDEBAR_W - 1, 0),
             cv::Point(SIDEBAR_W - 1, c.rows), cv::Scalar(60,60,70), 1);
    // App title
    cv::putText(c, "CS5330  Project 4", cv::Point(BTN_PAD + 2, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, ACC_CALIB, 1, cv::LINE_AA);
    cv::line(c, cv::Point(BTN_PAD, 22),
             cv::Point(SIDEBAR_W - BTN_PAD, 22), cv::Scalar(60,60,70), 1);
}

// ── setupButtons ─────────────────────────────────────────────────────────────

void CVGUI::setupButtons(
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
    std::function<void()> onScreenshot)
{
    buttons_.clear();

    const int x = BTN_PAD;
    const int w = SIDEBAR_W - 2 * BTN_PAD;
    int y = 28;   // start below title

    auto section = [&](int &yref, int extra = 0) {
        yref += extra;   // caller adds section header height + gap
    };

    // Helper lambda to add a button
    auto add = [&](const std::string &lbl, bool isToggle,
                   cv::Scalar base, std::function<void()> cb)
    {
        GuiButton b;
        b.rect      = cv::Rect(x, y, w, BTN_H);
        b.label     = lbl;
        b.isToggle  = isToggle;
        b.baseColor = base;
        b.onClick   = cb;
        buttons_.push_back(b);
        y += BTN_H + BTN_PAD;
    };

    // ── CALIBRATION ──────────────────────────────
    y += 4;   // gap after title line
    // store y for section header reference in drawDisplay — we just advance y
    y += 16;  // section header height
    add("Save Frame  (s)", false, BTN_ACT,  onSave);
    add("Calibrate   (c)", false, BTN_ACT,  onCalibrate);
    add("Write Calib (w)", false, BTN_BASE, onWrite);

    // ── AR OVERLAYS ───────────────────────────────
    y += 10;
    y += 16;
    add("3D Axes    (a)", true, BTN_BASE, onToggleAxes);
    add("Castle     (v)", true, BTN_BASE, onToggleCastle);
    add("OBJ Model  (o)", true, BTN_BASE, onToggleOBJ);
    add("Disguise   (d)", true, BTN_BASE, onToggleDisguise);

    // ── FEATURES ──────────────────────────────────
    y += 10;
    y += 16;
    add("ORB Features (f)", true, BTN_BASE, onToggleORB);
    add("Harris Crnrs (h)", true, BTN_BASE, onToggleHarris);

    // ── TARGET MODE ───────────────────────────────
    y += 10;
    y += 16;
    add("ArUco Mode  (m)", true, BTN_BASE, onToggleAruco);

    // ── ORB TRACKING ──────────────────────────────
    y += 10;
    y += 16;
    add("Capture Ref (r)", false, BTN_WARN, onCaptureRef);
    add("Track Mode  (t)", true,  BTN_BASE, onToggleTrack);

    // ── TOOLS ─────────────────────────────────────
    y += 10;
    y += 16;
    add("Print Pose  (p)", false, BTN_BASE, onPrintPose);
    add("Screenshot  (x)", false, BTN_BASE, onScreenshot);
}

// ── updateToggles ────────────────────────────────────────────────────────────

void CVGUI::updateToggles(bool axes, bool castle, bool obj, bool disguise,
                           bool orb, bool harris, bool aruco, bool track)
{
    for (auto &b : buttons_) {
        const std::string &l = b.label;
        if      (l.find("3D Axes")   != std::string::npos) b.toggled = axes;
        else if (l.find("Castle")    != std::string::npos) b.toggled = castle;
        else if (l.find("OBJ")       != std::string::npos) b.toggled = obj;
        else if (l.find("Disguise")  != std::string::npos) b.toggled = disguise;
        else if (l.find("ORB Feat")  != std::string::npos) b.toggled = orb;
        else if (l.find("Harris")    != std::string::npos) b.toggled = harris;
        else if (l.find("ArUco")     != std::string::npos) b.toggled = aruco;
        else if (l.find("Track Mode")!= std::string::npos) b.toggled = track;
    }
}

// ── buildDisplay ─────────────────────────────────────────────────────────────

cv::Mat CVGUI::buildDisplay(const cv::Mat &cameraFrame,
                             const std::vector<std::string> &statusLines)
{
    const int camW = cameraFrame.empty() ? 640 : cameraFrame.cols;
    const int camH = cameraFrame.empty() ? 480 : cameraFrame.rows;

    // Sidebar height must accommodate all buttons; compute from last button
    int sidebarNeeded = 30;
    if (!buttons_.empty())
        sidebarNeeded = buttons_.back().rect.y + BTN_H + BTN_PAD * 2;

    const int totalH = std::max(camH + STATUS_H, sidebarNeeded);
    const int totalW = SIDEBAR_W + camW;

    cv::Mat display(totalH, totalW, CV_8UC3, BG_DARK);

    // ── Draw sidebar ──
    drawSidebar(display);

    // ── Draw section headers  ──
    // We reconstruct the same y positions used in setupButtons.
    // (Keep in sync with setupButtons layout.)
    {
        int y = 28;
        y += 4;
        drawSection(display, "CALIBRATION", y, ACC_CALIB);  y += 16;
        y += 3 * (BTN_H + BTN_PAD);    // 3 calibration buttons
        y += 10;
        drawSection(display, "AR OVERLAYS", y, ACC_AR);     y += 16;
        y += 4 * (BTN_H + BTN_PAD);    // 4 AR buttons
        y += 10;
        drawSection(display, "FEATURES", y, ACC_FEAT);      y += 16;
        y += 2 * (BTN_H + BTN_PAD);    // 2 feature buttons
        y += 10;
        drawSection(display, "TARGET MODE", y, ACC_MODE);   y += 16;
        y += 1 * (BTN_H + BTN_PAD);    // 1 mode button
        y += 10;
        drawSection(display, "ORB TRACKING", y, ACC_TRACK); y += 16;
        y += 2 * (BTN_H + BTN_PAD);    // 2 tracking buttons
        y += 10;
        drawSection(display, "TOOLS", y, ACC_TOOLS);
    }

    // ── Draw buttons ──
    for (auto &b : buttons_) drawButton(display, b);

    // ── Paste camera frame ──
    if (!cameraFrame.empty()) {
        cv::Rect camROI(SIDEBAR_W, 0, camW, camH);
        cameraFrame.copyTo(display(camROI));
    }

    // ── Draw status panel ──
    drawStatusPanel(display, statusLines,
                    SIDEBAR_W, camH, camW, STATUS_H);

    return display;
}

// ── handleClick ──────────────────────────────────────────────────────────────

bool CVGUI::handleClick(int x, int y)
{
    for (auto &b : buttons_) {
        if (b.rect.contains(cv::Point(x, y))) {
            if (b.isToggle) b.toggled = !b.toggled;
            if (b.onClick) b.onClick();
            return true;
        }
    }
    return false;
}
