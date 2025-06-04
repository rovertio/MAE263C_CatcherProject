#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ball_detector/msg/set_xy.hpp"

using namespace std::chrono_literals;
using SetXY = ball_detector::msg::SetXY;

class PixelBridge : public rclcpp::Node
{
public:
	PixelBridge(): Node("pixel_to_xy_bridge")
	{
		using std::placeholders::_1;
		sub_ = create_subscription<geometry_msgs::msg::Point>(
				"/ball_centroid", 10, std::bind(&PixelBridge::cb, this, _1));

		pub_ = create_publisher<ball_detector::msg::SetXY>(
			"/set_xy", 10);

		// timer_=create_wall_timer(10ms,std::bind(&PixelBridge::cb,this));

	}

private:
	void cb(const geometry_msgs::msg::Point::SharedPtr p)
	{
		constexpr double SCALE = 0.22;
		constexpr double PIX_H = 240.0;
		constexpr double PIX_W = 320.0;

		double dx = (PIX_H / 2.0) - p->y;
		double dy = (PIX_W / 2.0) - p->x;

		auto m = SetXY();
		m.x = dx * SCALE;
		m.y = dy * SCALE;
		// message.header.stamp = this->get_clock()->now();
		// m.stamp = now();
		m.stamp = this->get_clock()->now();


		// Print raw detected values
        RCLCPP_INFO(get_logger(),
			"x loc=%.1f y loc=%.1f",
			m.x,m.y);

		pub_->publish(m);
	}
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
	rclcpp::Publisher<ball_detector::msg::SetXY>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PixelBridge>());
	rclcpp::shutdown();
	return 0;
}

