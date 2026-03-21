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
            cv::line(frame, pts2d[a], pts2d[b], colour, thickness, cv::LINE_AA);
    }
}

// ─── Draw 3D coordinate axes (length = 5 squares, arrowed) ───────────────
void draw3DAxes(cv::Mat &frame, const cv::Mat &cameraMatrix,
                const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec)
{
    std::vector<cv::Point3f> axes = {
        {0, 0, 0}, {5, 0, 0}, {0, -5, 0}, {0, 0, -5}
    };
    std::vector<cv::Point2f> img;
    cv::projectPoints(axes, rvec, tvec, cameraMatrix, distCoeffs, img);

    cv::arrowedLine(frame, img[0], img[1], cv::Scalar(0, 0, 255), 4, cv::LINE_AA, 0, 0.15);
    cv::arrowedLine(frame, img[0], img[2], cv::Scalar(0, 255, 0), 4, cv::LINE_AA, 0, 0.15);
    cv::arrowedLine(frame, img[0], img[3], cv::Scalar(255, 0, 0), 4, cv::LINE_AA, 0, 0.15);

    cv::putText(frame, "X", img[1], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
    cv::putText(frame, "Y", img[2], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
    cv::putText(frame, "Z", img[3], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,0), 2);
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
    float cx = (patternSize.width - 1) / 2.0f;
    float cy = -(patternSize.height - 1) / 2.0f;

    // ── Dimensions ──────────────────────────────────────────────────────────
    const float wallH     = 2.5f;
    const float tw        = 1.2f;   // corner tower footprint
    const float towerH    = 4.0f;   // corner tower height
    const float keepW     = 2.0f;   // central keep footprint
    const float keepH     = 2.0f;
    const float keepBaseZ = -wallH;
    const float keepWallH = 4.0f;

    // ── Face-fill helpers ────────────────────────────────────────────────────
    cv::Mat overlay = frame.clone();

    auto fillQ = [&](cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, cv::Point3f p3,
                     cv::Scalar col) {
        std::vector<cv::Point3f> pts3d = {p0, p1, p2, p3};
        std::vector<cv::Point2f> pts2d;
        cv::projectPoints(pts3d, rvec, tvec, cameraMatrix, distCoeffs, pts2d);
        std::vector<cv::Point> poly(4);
        for (int i = 0; i < 4; i++)
            poly[i] = cv::Point(cvRound(pts2d[i].x), cvRound(pts2d[i].y));
        cv::fillConvexPoly(overlay, poly, col);
    };
    auto fillT = [&](cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, cv::Scalar col) {
        std::vector<cv::Point3f> pts3d = {p0, p1, p2};
        std::vector<cv::Point2f> pts2d;
        cv::projectPoints(pts3d, rvec, tvec, cameraMatrix, distCoeffs, pts2d);
        std::vector<cv::Point> poly(3);
        for (int i = 0; i < 3; i++)
            poly[i] = cv::Point(cvRound(pts2d[i].x), cvRound(pts2d[i].y));
        cv::fillConvexPoly(overlay, poly, col);
    };

    // ── Color palette ────────────────────────────────────────────────────────
    const cv::Scalar stoneWall  (100,  95,  85);
    const cv::Scalar stoneTower (  75,  70,  65);
    const cv::Scalar towerRoofC ( 30,  60, 160);
    const cv::Scalar keepWallC  ( 35, 100,  35);
    const cv::Scalar keepRoofC  (130,  20, 130);

    // ── Fill pass ────────────────────────────────────────────────────────────
    // 1. Main wall
    {
        float x = cx-2.5f, y = cy+1.5f, w = 5.0f, d = 3.0f, h = wallH;
        fillQ({x,y,0},{x+w,y,0},{x+w,y,-h},{x,y,-h}, stoneWall);
        fillQ({x,y-d,0},{x+w,y-d,0},{x+w,y-d,-h},{x,y-d,-h}, stoneWall);
        fillQ({x,y,0},{x,y-d,0},{x,y-d,-h},{x,y,-h}, stoneWall);
        fillQ({x+w,y,0},{x+w,y-d,0},{x+w,y-d,-h},{x+w,y,-h}, stoneWall);
        fillQ({x,y,-h},{x+w,y,-h},{x+w,y-d,-h},{x,y-d,-h}, stoneWall);
    }

    // 2. Corner towers
    {
        float offsets[][2] = {
            {cx-2.5f, cy+1.5f},
            {cx+1.5f, cy+1.5f},
            {cx+1.5f, cy-1.5f+tw},
            {cx-2.5f, cy-1.5f+tw}
        };
        for (auto &off : offsets) {
            float x=off[0], y=off[1], h=towerH;
            fillQ({x,y,0},{x+tw,y,0},{x+tw,y,-h},{x,y,-h}, stoneTower);
            fillQ({x,y-tw,0},{x+tw,y-tw,0},{x+tw,y-tw,-h},{x,y-tw,-h}, stoneTower);
            fillQ({x,y,0},{x,y-tw,0},{x,y-tw,-h},{x,y,-h}, stoneTower);
            fillQ({x+tw,y,0},{x+tw,y-tw,0},{x+tw,y-tw,-h},{x+tw,y,-h}, stoneTower);
            fillQ({x,y,-h},{x+tw,y,-h},{x+tw,y-tw,-h},{x,y-tw,-h}, stoneTower);
        }
    }

    // 3. Tower pyramid roofs
    {
        float tTopZ = -towerH, tipZ = tTopZ-1.5f;
        float offsets[][2] = {
            {cx-1.9f, cy+0.9f}, {cx+2.1f, cy+0.9f},
            {cx+2.1f, cy-0.9f}, {cx-1.9f, cy-0.9f}
        };
        float hw = 0.6f, hh = 0.6f;
        for (auto &off : offsets) {
            float ox=off[0], oy=off[1];
            fillT({ox-hw,oy+hh,tTopZ},{ox+hw,oy+hh,tTopZ},{ox,oy,tipZ}, towerRoofC);
            fillT({ox+hw,oy+hh,tTopZ},{ox+hw,oy-hh,tTopZ},{ox,oy,tipZ}, towerRoofC);
            fillT({ox+hw,oy-hh,tTopZ},{ox-hw,oy-hh,tTopZ},{ox,oy,tipZ}, towerRoofC);
            fillT({ox-hw,oy-hh,tTopZ},{ox-hw,oy+hh,tTopZ},{ox,oy,tipZ}, towerRoofC);
        }
    }

    // 4. Central keep
    {
        float x=cx-keepW/2, y=cy+keepH/2, z=keepBaseZ, h=keepWallH;
        fillQ({x,y,z},{x+keepW,y,z},{x+keepW,y,z-h},{x,y,z-h}, keepWallC);
        fillQ({x,y-keepH,z},{x+keepW,y-keepH,z},{x+keepW,y-keepH,z-h},{x,y-keepH,z-h}, keepWallC);
        fillQ({x,y,z},{x,y-keepH,z},{x,y-keepH,z-h},{x,y,z-h}, keepWallC);
        fillQ({x+keepW,y,z},{x+keepW,y-keepH,z},{x+keepW,y-keepH,z-h},{x+keepW,y,z-h}, keepWallC);
        fillQ({x,y,z-h},{x+keepW,y,z-h},{x+keepW,y-keepH,z-h},{x,y-keepH,z-h}, keepWallC);
    }

    // 5. Keep pyramid roof
    {
        float kTopZ = keepBaseZ-keepWallH, tipZ = kTopZ-2.0f;
        float hw=1.1f, hh=1.1f;
        fillT({cx-hw,cy+hh,kTopZ},{cx+hw,cy+hh,kTopZ},{cx,cy,tipZ}, keepRoofC);
        fillT({cx+hw,cy+hh,kTopZ},{cx+hw,cy-hh,kTopZ},{cx,cy,tipZ}, keepRoofC);
        fillT({cx+hw,cy-hh,kTopZ},{cx-hw,cy-hh,kTopZ},{cx,cy,tipZ}, keepRoofC);
        fillT({cx-hw,cy-hh,kTopZ},{cx-hw,cy+hh,kTopZ},{cx,cy,tipZ}, keepRoofC);
    }

    cv::addWeighted(overlay, 0.60, frame, 0.40, 0, frame);

    // ── Wireframe pass ───────────────────────────────────────────────────────
    // 1. Castle walls (blue)
    {
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        addBox(pts, edges, cx-2.5f, cy+1.5f, 0, 5.0f, 3.0f, wallH);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(255,100,50), 3);
    }

    // 2. Corner towers (red)
    {
        float offsets[][2] = {
            {cx-2.5f, cy+1.5f},
            {cx+1.5f, cy+1.5f},
            {cx+1.5f, cy-1.5f+tw},
            {cx-2.5f, cy-1.5f+tw}
        };
        for (auto &off : offsets) {
            std::vector<cv::Point3f> pts;
            std::vector<std::pair<int,int>> edges;
            addBox(pts, edges, off[0], off[1], 0, tw, tw, towerH);
            drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                          cv::Scalar(0,0,220), 3);
        }
    }

    // 3. Tower roofs (yellow)
    {
        float tTopZ = -towerH, tipZ = tTopZ-1.5f;
        float offsets[][2] = {
            {cx-1.9f, cy+0.9f}, {cx+2.1f, cy+0.9f},
            {cx+2.1f, cy-0.9f}, {cx-1.9f, cy-0.9f}
        };
        for (auto &off : offsets) {
            std::vector<cv::Point3f> pts;
            std::vector<std::pair<int,int>> edges;
            addPyramid(pts, edges, off[0], off[1], tTopZ, 0.6f, 0.6f, tipZ);
            drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                          cv::Scalar(0,220,255), 3);
        }
    }

    // 4. Central keep (green)
    {
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        addBox(pts, edges, cx-keepW/2, cy+keepH/2, keepBaseZ, keepW, keepH, keepWallH);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0,200,0), 3);
    }

    // 5. Keep roof (magenta)
    {
        float kTopZ = keepBaseZ-keepWallH, tipZ = kTopZ-2.0f;
        std::vector<cv::Point3f> pts;
        std::vector<std::pair<int,int>> edges;
        addPyramid(pts, edges, cx, cy, kTopZ, 1.1f, 1.1f, tipZ);
        drawWireframe(frame, pts, edges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(255,0,255), 3);
    }

    // 6. Flag pole + flag
    {
        float kTopZ  = keepBaseZ-keepWallH;
        float pBase  = kTopZ-2.0f;
        float pTop   = pBase-1.5f;
        std::vector<cv::Point3f> polePts = {{cx, cy, pBase}, {cx, cy, pTop}};
        std::vector<std::pair<int,int>> poleEdges = {{0,1}};
        drawWireframe(frame, polePts, poleEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(200,200,200), 2);
        std::vector<cv::Point3f> flagPts = {
            {cx,      cy, pTop},
            {cx+1.0f, cy, pTop+0.5f},
            {cx,      cy, pTop+1.0f}
        };
        std::vector<std::pair<int,int>> flagEdges = {{0,1},{1,2},{2,0}};
        drawWireframe(frame, flagPts, flagEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0,0,255), 3);
    }

    // 7. Gate archway (orange)
    {
        float gx = cx, gy = cy+1.5f;
        std::vector<cv::Point3f> gatePts = {
            {gx-0.4f, gy,  0.0f},
            {gx+0.4f, gy,  0.0f},
            {gx+0.4f, gy, -1.0f},
            {gx-0.4f, gy, -1.0f},
            {gx,      gy, -1.3f}
        };
        std::vector<std::pair<int,int>> gateEdges = {{0,1},{0,3},{1,2},{2,4},{3,4}};
        drawWireframe(frame, gatePts, gateEdges, cameraMatrix, distCoeffs, rvec, tvec,
                      cv::Scalar(0,140,255), 3);
    }
}

// ─── Extension: disguise the chessboard ────────────────────────────────────
// Projects the outer boundary of the chessboard and fills it with a
// semi-transparent green-screen colour so the board no longer looks like
// a calibration target.  Uses a tiled checkerboard of two colours for a
// more convincing replacement texture.
void drawTargetDisguise(cv::Mat &frame, const cv::Mat &cameraMatrix,
                        const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec,
                        const cv::Size &patternSize)
{
    const int cols = patternSize.width - 1;   // number of inner-corner columns
    const int rows = patternSize.height - 1;  // number of inner-corner rows

    // Project all inner corners + a one-square border
    // We'll fill each cell individually so we can alternate the disguise colour.
    // World coords: inner corners span (0,0) → (cols, -rows).
    // We extend by one half-square on each side for the outer squares.
    // For simplicity fill each board square with a projected quad.

    // Choose two disguise colours (a mosaic of warm amber squares)
    const cv::Scalar colA(0,  120, 255);   // orange
    const cv::Scalar colB(0,   60, 180);   // dark orange

    // Iterate over every square on the full board including the outer border.
    // Inner corners span c=0..cols-1, r=0..rows-1; extending by 1 on each side
    // covers the outer ring of squares that the printed board also has.
    for (int r = -1; r <= rows; r++) {
        for (int c = -1; c <= cols; c++) {
            // 4 world corners of this square
            std::vector<cv::Point3f> sq3d = {
                {(float)c,       (float)(-r),       0.0f},
                {(float)(c + 1), (float)(-r),       0.0f},
                {(float)(c + 1), (float)(-r - 1),   0.0f},
                {(float)c,       (float)(-r - 1),   0.0f}
            };
            std::vector<cv::Point2f> sq2d;
            cv::projectPoints(sq3d, rvec, tvec, cameraMatrix, distCoeffs, sq2d);

            std::vector<cv::Point> poly(4);
            for (int i = 0; i < 4; i++)
                poly[i] = cv::Point(cvRound(sq2d[i].x), cvRound(sq2d[i].y));

            // Use bitwise AND for correct alternating colour with negative indices
            cv::Mat overlay = frame.clone();
            cv::fillConvexPoly(overlay, poly, ((r + c) & 1) == 0 ? colA : colB);
            cv::addWeighted(overlay, 0.65, frame, 0.35, 0, frame);
        }
    }
}