#pragma once
#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

/* ── shared vision constants ──────────────────────────────────────── */
namespace vp {
constexpr int    IMG_W = 640, IMG_H = 240;
constexpr int    HALF_W = IMG_W / 2;
constexpr double IMG_CX = HALF_W / 2.0;
constexpr double IMG_CY = IMG_H  / 2.0;

constexpr int    THRESH    = 225;
constexpr int    ERODE_SZ  = 3, DILATE_SZ = 7;
constexpr double MIN_AREA  = 15.0 * 15.0;

constexpr double HFOV_DEG  = 45.0;
constexpr double CAMERA_H  = 85.0;          // camera height [cm]
constexpr double BASELINE  = 5.5;           // stereo baseline [cm]
constexpr double FOCAL_PX  = (HALF_W / 2.0) /
        std::tan((HFOV_DEG * M_PI / 180.0) / 2.0);

/* Physics & filter */
constexpr double G_CM       = 981.0;        // gravity [cm s⁻²]
constexpr double LPF_ALPHA  = 0.7;
constexpr double VEL_ALPHA  = 0.7;
constexpr double PRED_ALPHA = 0.6;
constexpr std::size_t BUF_MAX      = 4;
constexpr std::size_t BUF_MIN      = 2;
constexpr double      RESET_GAP_S  = 0.30;
} // namespace vp

/* ── centroid helper (optional: move to vision_utils.hpp) ─────────── */
inline bool centroid(const cv::Mat& roi,
                     double& cx, double& cy, double& area,
                     const cv::Mat& erodeK, const cv::Mat& dilateK)
{
  cv::Mat g, m;  cv::cvtColor(roi, g, cv::COLOR_BGR2GRAY);
  cv::threshold(g, m, vp::THRESH, 255, cv::THRESH_BINARY);
  cv::erode(m, m, erodeK);  cv::dilate(m, m, dilateK);

  std::vector<std::vector<cv::Point>> cont;
  cv::findContours(m, cont, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (cont.empty()) return false;

  auto it = std::max_element(cont.begin(), cont.end(),
            [](auto& a, auto& b){ return cv::contourArea(a) < cv::contourArea(b); });
  area = cv::contourArea(*it);
  if (area < vp::MIN_AREA) return false;

  cv::Moments mom = cv::moments(*it);
  cx = mom.m10 / mom.m00;
  cy = mom.m01 / mom.m00;
  return true;
}

