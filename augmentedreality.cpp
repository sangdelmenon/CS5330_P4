/*
  augmentedreality.cpp
  CS 5330 - Project 4
  Virtual object rendering. The castle is designed in chessboard world
  coordinates (1 unit = 1 square) centered on the board, floating above
  the Z=0 plane (Z points away from the board, so negative Z = above).
*/
#include "augmentedreality.h"
#include <cmath>

// ─── Helper: project + draw edges ──────────────────────────────────────────
void drawWireframe(cv::Mat &frame, const std::vector<cv::Point3f> &pts3d,
                   const std::vector<std::pair<int,int>> &edges,
                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                   const cv::Mat &rvec, const cv::Mat &tvec,
                   cv::Scalar colour, int thickness)
{
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts3d, rvec, tvec, cameraMatrix, distCoeffs, pts2d);

    for (auto &[a, b] : edges) {
        if (a >= 0 && a < (int)pts2d.size() && b >= 0 && b < (int)pts2d.size())
            cv::line(frame, pts2d[a], pts2d[b], colour, thickness);
    }
}

// ─── Draw 3D coordinate axes (length = 3 squares) ─────────────────────────
void draw3DAxes(cv::Mat &frame, const cv::Mat &cameraMatrix,
                const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec)
{
    std::vector<cv::Point3f> axes = {
        {0, 0, 0}, {3, 0, 0}, {0, -3, 0}, {0, 0, -3}
    };
    std::vector<cv::Point2f> img;
    cv::projectPoints(axes, rvec, tvec, cameraMatrix, distCoeffs, img);

    cv::line(frame, img[0], img[1], cv::Scalar(0, 0, 255), 3);   // X = red
    cv::line(frame, img[0], img[2], cv::Scalar(0, 255, 0), 3);   // Y = green
    cv::line(frame, img[0], img[3], cv::Scalar(255, 0, 0), 3);   // Z = blue

    cv::putText(frame, "X", img[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
    cv::putText(frame, "Y", img[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
    cv::putText(frame, "Z", img[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);
}

// ─── Draw projected outside corners ────────────────────────────────────────
void drawOutsideCorners(cv::Mat &frame, const cv::Mat &cameraMatrix,
                        const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                        const cv::Size &patternSize)
{
    int w = patternSize.width - 1;
    int h = patternSize.height - 1;
    std::vector<cv::Point3f> corners = {
        {0, 0, 0}, {(float)w, 0, 0},
        {(float)w, -(float)h, 0}, {0, -(float)h, 0}
    };
    std::vector<cv::Point2f> img;
    cv::projectPoints(corners, rvec, tvec, cameraMatrix, distCoeffs, img);

    for (int i = 0; i < 4; i++) {
        cv::circle(frame, img[i], 8, cv::Scalar(0, 255, 255), -1);
        cv::line(frame, img[i], img[(i+1)%4], cv::Scalar(0, 255, 255), 2);
    }
}

// ─── Helper: make a box (8 points + 12 edges) ─────────────────────────────
static void addBox(std::vector<cv::Point3f> &pts,
                   std::vector<std::pair<int,int>> &edges,
                   float x, float y, float z,
                   float w, float h, float d)
{
    // z is the base height (negative = above board)
    // d is the depth going further above (more negative)
    int base = (int)pts.size();

    // 8 corners of the box
    pts.push_back({x,     y,     z});       // 0: front-left-bottom
    pts.push_back({x + w, y,     z});       // 1: front-right-bottom
    pts.push_back({x + w, y - h, z});       // 2: back-right-bottom
    pts.push_back({x,     y - h, z});       // 3: back-left-bottom
    pts.push_back({x,     y,     z - d});   // 4: front-left-top
    pts.push_back({x + w, y,     z - d});   // 5: front-right-top
    pts.push_back({x + w, y - h, z - d});   // 6: back-right-top
    pts.push_back({x,     y - h, z - d});   // 7: back-left-top

    // Bottom face
    edges.push_back({base+0, base+1});
    edges.push_back({base+1, base+2});
    edges.push_back({base+2, base+3});
    edges.push_back({base+3, base+0});
    // Top face
    edges.push_back({base+4, base+5});
    edges.push_back({base+5, base+6});
    edges.push_back({base+6, base+7});
    edges.push_back({base+7, base+4});
    // Vertical edges
    edges.push_back({base+0, base+4});
    edges.push_back({base+1, base+5});
    edges.push_back({base+2, base+6});
    edges.push_back({base+3, base+7});
}

// ─── Helper: make a pyramid (5 points + 8 edges) ──────────────────────────
static void addPyramid(std::vector<cv::Point3f> &pts,
                       std::vector<std::pair<int,int>> &edges,
                       float cx, float cy, float baseZ,
                       float halfW, float halfH, float tipZ)
{
    int base = (int)pts.size();

    pts.push_back({cx - halfW, cy + halfH, baseZ});   // 0: front-left
    pts.push_back({cx + halfW, cy + halfH, baseZ});   // 1: front-right
    pts.push_back({cx + halfW, cy - halfH, baseZ});   // 2: back-right
    pts.push_back({cx - halfW, cy - halfH, baseZ});   // 3: back-left
    pts.push_back({cx, cy, tipZ});                     // 4: tip

    // Base
    edges.push_back({base+0, base+1});
    edges.push_back({base+1, base+2});
    edges.push_back({base+2, base+3});
    edges.push_back({base+3, base+0});
    // Sides to tip
    edges.push_back({base+0, base+4});
    edges.push_back({base+1, base+4});
    edges.push_back({base+2, base+4});
    edges.push_back({base+3, base+4});
}

// ─── Draw a multi-color castle ─────────────────────────────────────────────
void drawCastle(cv::Mat &frame, const cv::Mat &cameraMatrix,
                const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                const cv::Size &patternSize)
{
    // Center the castle on the board
    float cx = (patternSize.width - 1) / 2.0f;
    float cy = -(patternSize.height - 1) / 2.0f;

    // ── 1. Castle walls (blue) ──
    {
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        // Main wall: centered, 5 wide, 3 deep, 1.5 tall
        addBox(pts, edges, cx - 2.5f, cy + 1.5f, 0, 5.0f, 3.0f, 1.5f);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(255, 100, 50), 2); // blue
    }

    // ── 2. Four corner towers (red) ──
    {
        float tw = 1.0f;  // tower width
        float th = 2.5f;  // tower height
        float offsets[][2] = {
            {cx - 2.5f, cy + 1.5f},           // front-left
            {cx + 1.5f, cy + 1.5f},           // front-right
            {cx + 1.5f, cy - 1.5f + tw},      // back-right
            {cx - 2.5f, cy - 1.5f + tw}       // back-left
        };

        for (auto &off : offsets) {
            std::vector<cv::Point3f> pts;
            std::vector<std::pair<int,int>> edges;
            addBox(pts, edges, off[0], off[1], 0, tw, tw, th);
            drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                          cv::Scalar(0, 0, 220), 2); // red
        }
    }

    // ── 3. Tower roofs / pyramids (yellow) ──
    {
        float tw = 1.0f;
        float towerTop = -2.5f;   // top of tower walls
        float roofTip  = -3.5f;   // tip of pyramid roof
        float offsets[][2] = {
            {cx - 2.0f, cy + 1.0f},
            {cx + 2.0f, cy + 1.0f},
            {cx + 2.0f, cy - 1.0f},
            {cx - 2.0f, cy - 1.0f}
        };

        for (auto &off : offsets) {
            std::vector<cv::Point3f> pts;
            std::vector<std::pair<int,int>> edges;
            addPyramid(pts, edges, off[0], off[1], towerTop, 0.6f, 0.6f, roofTip);
            drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                          cv::Scalar(0, 220, 255), 2); // yellow
        }
    }

    // ── 4. Central keep / tall tower (green) ──
    {
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        addBox(pts, edges, cx - 0.75f, cy + 0.75f, -1.5f, 1.5f, 1.5f, 2.5f);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0, 200, 0), 2); // green
    }

    // ── 5. Central roof pyramid (magenta) ──
    {
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        addPyramid(pts, edges, cx, cy, -4.0f, 0.9f, 0.9f, -5.5f);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(255, 0, 255), 2); // magenta
    }

    // ── 6. Flag pole + flag on central tower (cyan + red) ──
    {
        // Pole
        std::vector<cv::Point3f> polePts = {{cx, cy, -5.5f}, {cx, cy, -7.0f}};
        std::vector<std::pair<int,int>> poleEdges = {{0, 1}};
        drawWireframe(frame, polePts, poleEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(200, 200, 200), 2); // white pole

        // Flag (triangle)
        std::vector<cv::Point3f> flagPts = {
            {cx, cy, -7.0f},
            {cx + 1.0f, cy, -6.5f},
            {cx, cy, -6.0f}
        };
        std::vector<std::pair<int,int>> flagEdges = {{0, 1}, {1, 2}, {2, 0}};
        drawWireframe(frame, flagPts, flagEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0, 0, 255), 3); // red flag
    }

    // ── 7. Gate archway on front wall (orange) ──
    {
        float gx = cx;
        float gy = cy + 1.5f; // front face
        std::vector<cv::Point3f> gatePts = {
            {gx - 0.3f, gy, 0},          // 0: bottom-left
            {gx + 0.3f, gy, 0},          // 1: bottom-right
            {gx + 0.3f, gy, -0.8f},      // 2: top-right
            {gx - 0.3f, gy, -0.8f},      // 3: top-left
            {gx,        gy, -1.0f}        // 4: arch peak
        };
        std::vector<std::pair<int,int>> gateEdges = {
            {0, 1}, {0, 3}, {1, 2}, {2, 4}, {3, 4}
        };
        drawWireframe(frame, gatePts, gateEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0, 140, 255), 3); // orange
    }
}