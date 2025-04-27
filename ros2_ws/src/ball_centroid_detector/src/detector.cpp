#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>

class BallCentroidDetector : public rclcpp::Node
{
public:
	BallCentroidDetector()
	: Node("ball_centroid_detector")
	{
		sub_ = this->create_subscription<sensor_msgs::msg::Image>(
				"/ball_image", 
				10,
				std::bind(&BallCentroidDetector::image_cb, this, std::placeholders::_1));
	}

private:
	void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
	{
		cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;

		cv::Mat gray;
		cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

		cv::Mat mask;
		cv::threshold(gray, mask, 200, 255, cv::THRESH_BINARY);

		cv::erode(mask, mask, cv::Mat(), {-1,-1}, 1);
		cv::dilate(mask, mask, cv::Mat(), {-1,-1}, 2);

		cv::Moments M = cv::moments(mask, /*binaryImage=*/true);

		if (M.m00 > 1e-2) {
			double cx = M.m10 / M.m00;
			double cy = M.m01 / M.m00;
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
						33,
						"Centroid pixel: (%.1f, %.1f)", cx, cy);
		} else {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
					2000, "Ball not found");
		}
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BallCentroidDetector>());
	rclcpp::shutdown();
	return 0;
}
