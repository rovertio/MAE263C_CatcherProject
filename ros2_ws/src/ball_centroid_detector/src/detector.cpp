#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // For cv::Rect
#include "geometry_msgs/msg/point.hpp"

class BallCentroidDetector : public rclcpp::Node
{
public:
    BallCentroidDetector()
        : Node("ball_centroid_detector")
    {
        // Assuming the combined image comes from gscam on /camera/combined/image_raw
        // And your ball_image topic is a remapping of that or a different source
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", // Or "/ball_image" if you remap it
            10,
            std::bind(&BallCentroidDetector::image_cb, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/ball_centroid", 10);

        RCLCPP_INFO(this->get_logger(), "BallCentroidDetector node started. Waiting for images...");
    }

private:
    // Helper function to find centroid in a single image/ROI
    // Returns true if centroid found, false otherwise.
    // cx and cy are output parameters.
    bool find_centroid_in_roi(const cv::Mat& roi_bgr, double& cx_out, double& cy_out)
    {
        if (roi_bgr.empty()) {
            return false;
        }

        cv::Mat gray;
        cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);

        cv::Mat mask;
        // Adjust these parameters based on your ball's color and lighting
        // For a bright ball, a higher threshold (e.g., 200-240) is good.
        // For a dark ball on a light background, you might invert the threshold
        // or use a lower threshold with THRESH_BINARY_INV.
        cv::threshold(gray, mask, 240, 255, cv::THRESH_BINARY); // Threshold might need tuning

        // Optional: Denoising before morphology
        // cv::medianBlur(mask, mask, 3);


        // Morphology: Erode to remove small noise, Dilate to restore ball size
        // The kernel size and iterations might need tuning
        cv::Mat kernel_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)); // Larger dilation

        cv::erode(mask, mask, kernel_erode, cv::Point(-1,-1), 1);
        cv::dilate(mask, mask, kernel_dilate, cv::Point(-1,-1), 1); // Consider more dilation if ball is small or erosion is aggressive


        cv::Moments M = cv::moments(mask, /*binaryImage=*/true);

        if (M.m00 > 1e-2) { // Check if any white pixels (ball area) found
            cx_out = M.m10 / M.m00;
            cy_out = M.m01 / M.m00;
            return true;
        }
        return false;
    }

    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat bgr_full = cv_ptr->image;
        if (bgr_full.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty image.");
            return;
        }

        // Assuming the image is 640x240 (width x height)
        // Left camera ROI: x=0, y=0, width=320, height=240
        // Right camera ROI: x=320, y=0, width=320, height=240
        if (bgr_full.cols != 640 || bgr_full.rows != 240) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Received image with unexpected dimensions: %dx%d. Expected 640x240.",
                                 bgr_full.cols, bgr_full.rows);
            return;
        }

        cv::Rect left_roi_rect(0, 0, 320, 240);
        cv::Rect right_roi_rect(320, 0, 320, 240);

        cv::Mat left_bgr = bgr_full(left_roi_rect);
        cv::Mat right_bgr = bgr_full(right_roi_rect);

        double cx_left = -1.0, cy_left = -1.0;
        double cx_right_roi = -1.0, cy_right_roi = -1.0; // Relative to right ROI

        bool found_left = find_centroid_in_roi(left_bgr, cx_left, cy_left);
        bool found_right = find_centroid_in_roi(right_bgr, cx_right_roi, cy_right_roi);

        // Adjust right centroid to be in the 0-319 coordinate system (like the left)
        double cx_right_adjusted = -1.0, cy_right_adjusted = -1.0;
        if (found_right) {
            cx_right_adjusted = cx_right_roi; // Already in 0-319 relative to its ROI
            cy_right_adjusted = cy_right_roi;
        }

        geometry_msgs::msg::Point p_out;
        bool ball_detected_overall = false;

        if (found_left && found_right) {
            p_out.x = (cx_left + cx_right_adjusted) / 2.0;
            p_out.y = (cy_left + cy_right_adjusted) / 2.0;
            p_out.z = 0.0; // Or some confidence metric / depth if available
            ball_detected_overall = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 17, // Log every 1 sec
                                 "Ball found in L&R. Left(%.1f,%.1f) RightAdj(%.1f,%.1f) Avg(%.1f,%.1f)",
                                 cx_left, cy_left, cx_right_adjusted, cy_right_adjusted, p_out.x, p_out.y);
        } else if (found_left) {
            p_out.x = cx_left;
            p_out.y = cy_left;
            p_out.z = 0.0;
            ball_detected_overall = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 17,
                                 "Ball found in Left only. (%.1f,%.1f)", p_out.x, p_out.y);
        } else if (found_right) {
            p_out.x = cx_right_adjusted;
            p_out.y = cy_right_adjusted;
            p_out.z = 0.0;
            ball_detected_overall = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 17,
                                 "Ball found in Right only. (%.1f,%.1f)", p_out.x, p_out.y);
        }

        if (ball_detected_overall) {
            // Ensure coordinates are within the 320x240 space
            // (though averaging should keep them within if individual detections are)
            p_out.x = std::max(0.0, std::min(p_out.x, 319.0));
            p_out.y = std::max(0.0, std::min(p_out.y, 239.0));
            pub_->publish(p_out);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, // Log every 2 secs
                                 "Ball not found in either camera view.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallCentroidDetector>());
    rclcpp::shutdown();
    return 0;
}
