#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

class SyntheticBallPublisher : public rclcpp::Node
{
public:
	SyntheticBallPublisher()
	: Node("synthetic_ball_publisher")
	{
		image_pub_ = 
			image_transport::create_publisher(this, "ball_image");

		timer_ = this->create_wall_timer(
			33ms,
			std::bind(&SyntheticBallPublisher::timer_cb, this));

		width_  = 640;
		height_ = 480;
		radius_ = 60;
		center_ = cv::Point(width_/4,height_/4);
	}

private:
	void timer_cb()
	{
		cv::Mat img = cv::Mat::zeros(height_, width_, CV_8UC3);

		cv::circle(img, center_, radius_, cv::Scalar(255, 255, 255), cv::FILLED);

		std_msgs::msg::Header hdr;
		hdr.stamp = this->now();
		hdr.frame_id = "synthetic_camera";
		sensor_msgs::msg::Image::SharedPtr msg = 
			cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();

		image_pub_.publish(msg);
	}

	image_transport::Publisher image_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	int width_, height_, radius_;
	cv::Point center_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SyntheticBallPublisher>());
	rclcpp::shutdown();
	return 0;
}

