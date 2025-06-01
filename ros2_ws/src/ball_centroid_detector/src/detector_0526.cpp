// detector.cpp
#include <memory>
#include <vector>
#include <deque> // For easier management of fixed-size history
#include <algorithm>
#include <cmath> // For sqrt and tan
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <opencv2/core/types.hpp> // For cv::Point2d

#ifndef M_PI // Ensure M_PI is defined
#define M_PI 3.14159265358979323846
#endif

constexpr double MIN_BALL_AREA = 15.0 * 15.0; 

// Constants for AREA-based height estimation
constexpr double REFERENCE_AREA_AT_LOWEST_POINT = 340.0;
constexpr double BALL_HEIGHT_CALIBRATION_FACTOR = 100.0; 

// Constants for STEREO-based height estimation
constexpr double STEREO_BASELINE_CM = 6.0; 
constexpr double HORIZONTAL_FOV_DEGREES = 45.0; 

// Constants for perspective correction (shared)
constexpr double CAMERA_HEIGHT_CM = 85.0; 
constexpr double IMAGE_WIDTH_PIXELS = 320.0; 
constexpr double IMAGE_HEIGHT_PIXELS = 240.0; 
constexpr double IMAGE_CENTER_X_PIXELS = IMAGE_WIDTH_PIXELS / 2.0; 
constexpr double IMAGE_CENTER_Y_PIXELS = IMAGE_HEIGHT_PIXELS / 2.0;

// Constants for Prediction
constexpr double LOOKAHEAD_FRAMES = 0; // How many frames/iterations to predict ahead. Tune this.
const size_t PREDICTION_HISTORY_SIZE = 2; // Need 2 previous points for one velocity estimate


class BallCentroidDetector : public rclcpp::Node
{
public:
    BallCentroidDetector()
    : Node("ball_centroid_detector"), 
      FOCAL_LENGTH_PIXELS_((IMAGE_WIDTH_PIXELS / 2.0) / std::tan((HORIZONTAL_FOV_DEGREES * M_PI / 180.0) / 2.0))
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&BallCentroidDetector::image_cb, this, std::placeholders::_1));
        pub_ = create_publisher<geometry_msgs::msg::Point>("/ball_centroid", 10);
        RCLCPP_INFO(get_logger(), "BallCentroidDetector started. Cam H: %.1fcm. Img Ctr: (%.0f, %.0f). Focal: %.1f px. Lookahead: %.1f frames",
            CAMERA_HEIGHT_CM, IMAGE_CENTER_X_PIXELS, IMAGE_CENTER_Y_PIXELS, FOCAL_LENGTH_PIXELS_, LOOKAHEAD_FRAMES);
    }

private:
    bool find_centroid_in_roi(const cv::Mat& roi, double& cx, double& cy, double& area_out)
    {
        if (roi.empty()) return false;
        cv::Mat gray, mask;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, 247, 255, cv::THRESH_BINARY); 
        cv::Mat erodeK = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3});
        cv::Mat dilateK = cv::getStructuringElement(cv::MORPH_ELLIPSE, {7,7});
        cv::erode(mask, mask, erodeK, {}, 1);
        cv::dilate(mask, mask, dilateK, {}, 1);
        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double bestA = 0;
        cv::Point2d bestC;
        for (auto &c : cnts) {
            double area = cv::contourArea(c);
            if (area < MIN_BALL_AREA) continue;
            auto m = cv::moments(c);
            if (m.m00 == 0) continue;
            double _cx = m.m10/m.m00;
            double _cy = m.m01/m.m00;
            if (area > bestA) {
                bestA = area;
                bestC = {_cx, _cy};
            }
        }
        if (bestA > 0) {
            cx = bestC.x;
            cy = bestC.y;
            area_out = bestA;
            return true;
        }
        area_out = 0;
        return false;
    }

    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        if (img.empty()) return;

        if (img.cols != 640 || img.rows != 240) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Bad image size: %dx%d", img.cols, img.rows);
            return;
        }

        cv::Rect L_roi_rect(0,0,320,240), R_roi_rect(320,0,320,240);
        double lx, ly, rx, ry; 
        double areaL = 0.0, areaR = 0.0;
        bool fL = find_centroid_in_roi(img(L_roi_rect), lx, ly, areaL);
        bool fR = find_centroid_in_roi(img(R_roi_rect), rx, ry, areaR);

        geometry_msgs::msg::Point point_to_publish; 
        bool found_overall = false;
        double current_ball_area = 0.0;
        double perceived_px = 0.0, perceived_py = 0.0; 

        if (fL && fR) {
            perceived_px = (lx + rx)/2.0; 
            perceived_py = (ly + ry)/2.0; 
            current_ball_area = (areaL + areaR) / 2.0;
            found_overall = true;
        } else if (fL) {
            perceived_px = lx; perceived_py = ly;
            current_ball_area = areaL;
            found_overall = true;
        } else if (fR) {
            perceived_px = rx; perceived_py = ry;
            current_ball_area = areaR;
            found_overall = true;
        }

        if (found_overall) {
            double original_clamped_px = std::clamp(perceived_px, 0.0, IMAGE_WIDTH_PIXELS - 1.0);
            double original_clamped_py = std::clamp(perceived_py, 0.0, IMAGE_HEIGHT_PIXELS - 1.0);

            double h_cm_area = 0.0; 
            if (current_ball_area > 1.0) { 
                double ratio_for_height = REFERENCE_AREA_AT_LOWEST_POINT / current_ball_area;
                h_cm_area = BALL_HEIGHT_CALIBRATION_FACTOR * (std::sqrt(std::max(0.0, ratio_for_height)) - 1.0);
            } else {
                 RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,"Area too small (%.1f) for area-based height.", current_ball_area);
            }

            double h_cm_stereo = -1.0; 
            double distance_to_ball_stereo_cm = -1.0;
            if (fL && fR) { 
                double disparity_pixels = lx - rx; 
                RCLCPP_DEBUG(get_logger(), "Stereo: lx=%.1f, rx=%.1f, disparity=%.1f px", lx, rx, disparity_pixels);

                if (disparity_pixels > 0.5) { 
                    distance_to_ball_stereo_cm = (STEREO_BASELINE_CM * FOCAL_LENGTH_PIXELS_) / disparity_pixels;
                    h_cm_stereo = CAMERA_HEIGHT_CM - distance_to_ball_stereo_cm;
                    RCLCPP_INFO(get_logger(), "Stereo Est: Dist: %.1f cm, Height: %.1f cm", distance_to_ball_stereo_cm, h_cm_stereo);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Stereo: Disparity too small or non-positive (%.1f px). Cannot estimate depth.", disparity_pixels);
                }
            }
            
            RCLCPP_INFO(get_logger(), "Ball Raw Px: (%.1f, %.1f). Area: %.1f -> Est H_area: %.1f cm.", 
                        original_clamped_px, original_clamped_py, current_ball_area, h_cm_area);

            double h_cm_for_correction = h_cm_area; 
            if (fL && fR && h_cm_stereo > -CAMERA_HEIGHT_CM * 0.5 && distance_to_ball_stereo_cm > 0) { 
                if (std::abs(h_cm_stereo) < CAMERA_HEIGHT_CM * 1.5) { 
                    h_cm_for_correction = h_cm_stereo;
                    RCLCPP_INFO(get_logger(), "Using Stereo Height (%.1f cm) for perspective correction.", h_cm_stereo);
                } else {
                    RCLCPP_WARN(get_logger(), "Stereo height (%.1f cm) seems unreasonable, falling back to area height (%.1f cm).", h_cm_stereo, h_cm_area);
                }
            } else {
                 RCLCPP_INFO(get_logger(), "Using Area Height (%.1f cm) for perspective correction.", h_cm_area);
            }

            double dx_pixels = perceived_px - IMAGE_CENTER_X_PIXELS;
            double dy_pixels = perceived_py - IMAGE_CENTER_Y_PIXELS;
            double H_minus_h = CAMERA_HEIGHT_CM - h_cm_for_correction;
            double current_corrected_px = perceived_px; 
            double current_corrected_py = perceived_py;

            if (H_minus_h > 1.0 && CAMERA_HEIGHT_CM > 1.0) { 
                double correction_factor = H_minus_h / CAMERA_HEIGHT_CM;
                double corrected_dx_pixels = dx_pixels * correction_factor;
                double corrected_dy_pixels = dy_pixels * correction_factor;
                current_corrected_px = IMAGE_CENTER_X_PIXELS + corrected_dx_pixels;
                current_corrected_py = IMAGE_CENTER_Y_PIXELS + corrected_dy_pixels;
                RCLCPP_INFO(get_logger(), "Perspective Corrected Px: (%.1f, %.1f). Factor: %.3f",
                            current_corrected_px, current_corrected_py, correction_factor);
            } else {
                 RCLCPP_WARN(get_logger(), "Ball est. at/above camera (H-h = %.1f cm) or invalid H. No perspective correction.", H_minus_h);
            }
            
            // --- PREDICTION ---
            double predicted_px = current_corrected_px;
            double predicted_py = current_corrected_py;

            if (previous_ball_positions_.size() == PREDICTION_HISTORY_SIZE) {
                const cv::Point2d& pos_n_minus_1 = previous_ball_positions_[1]; // Most recent previous
                const cv::Point2d& pos_n_minus_2 = previous_ball_positions_[0]; // Oldest in history
                
                double velocity_x = pos_n_minus_1.x - pos_n_minus_2.x;
                double velocity_y = pos_n_minus_1.y - pos_n_minus_2.y;
                
                // Predict based on current corrected position and this velocity
                // (A more robust predictor might use velocity from pos_n_minus_1 and current_corrected_px)
                // Let's use current_corrected_px as the base for prediction
                predicted_px = current_corrected_px + velocity_x * LOOKAHEAD_FRAMES;
                predicted_py = current_corrected_py + velocity_y * LOOKAHEAD_FRAMES;
                
                RCLCPP_INFO(get_logger(), "Prediction: Vel (%.1f, %.1f) px/fr. Pred Px: (%.1f, %.1f)",
                            velocity_x, velocity_y, predicted_px, predicted_py);
            } else {
                RCLCPP_INFO(get_logger(), "Prediction: Not enough history (%zu/%zu), using current corrected position.",
                            previous_ball_positions_.size(), PREDICTION_HISTORY_SIZE);
            }

            // Update history with current corrected position
            previous_ball_positions_.push_back(cv::Point2d(current_corrected_px, current_corrected_py));
            if (previous_ball_positions_.size() > PREDICTION_HISTORY_SIZE) {
                previous_ball_positions_.pop_front(); // Keep history size fixed
            }

            point_to_publish.x = std::clamp(predicted_px, 0.0, IMAGE_WIDTH_PIXELS - 1.0);
            point_to_publish.y = std::clamp(predicted_py, 0.0, IMAGE_HEIGHT_PIXELS - 1.0);
            point_to_publish.z = h_cm_for_correction; 
            pub_->publish(point_to_publish);

        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ball not found in either view.");
            previous_ball_positions_.clear(); // Reset history if ball is lost
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    const double FOCAL_LENGTH_PIXELS_; 
    std::deque<cv::Point2d> previous_ball_positions_; // Using deque for efficient pop_front
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallCentroidDetector>());
    rclcpp::shutdown();
    return 0;
}
