#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "inverse_kinematics_node/msg/set_xy.hpp"

class PixelBridge : public rclcpp::Node
{
public:
	PixelBridge(): Node("pixel_to_xy_bridge")
	{
		using std::placeholders::_1;
		sub_ = create_subscription<geometry_msgs::msg::Point>(
				"/ball_centroid", 10, std::bind(&PixelBridge::cb, this, _1));

	pub_ = create_publisher<inverse_kinematics_node::msg::SetXY>(
		"/set_xy", 10);

	}

private:
	void cb(const geometry_msgs::msg::Point::SharedPtr p)
	{
		constexpr double SCALE = 1.0/8.0;
		constexpr double PIX_H = 240.0;
		constexpr double PIX_W = 320.0;

		double dx = (PIX_H / 2.0) - p->y;
		double dy = (PIX_W / 2.0) - p->x;

		inverse_kinematics_node::msg::SetXY m;
		m.x = dx * SCALE;
		m.y = dy * SCALE;
		pub_->publish(m);
	}
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
	rclcpp::Publisher<inverse_kinematics_node::msg::SetXY>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PixelBridge>());
	rclcpp::shutdown();
	return 0;
}

