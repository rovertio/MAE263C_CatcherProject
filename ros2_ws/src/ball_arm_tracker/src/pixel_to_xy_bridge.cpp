#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "controller_msgs/msg/set_xy.hpp"

using SetXY = controller_msgs::msg::SetXY;      // alias once

class PixelBridge : public rclcpp::Node
{
public:
  PixelBridge() : Node("pixel_to_xy_bridge")
  {
    using std::placeholders::_1;
    sub_ = create_subscription<geometry_msgs::msg::Point>(
              "/ball_centroid", 10, std::bind(&PixelBridge::cb, this, _1));

    pub_ = create_publisher<SetXY>("/set_xy", 10);   // ★ changed
  }

private:
  void cb(const geometry_msgs::msg::Point::SharedPtr p)
  {
    constexpr double SCALE = 0.22;
    constexpr double PIX_H = 240.0;
    constexpr double PIX_W = 320.0;

    double dx = (PIX_H / 2.0) - p->y;
    double dy = (PIX_W / 2.0) - p->x;

    SetXY msg;                                       // ★ changed
    msg.x = dx * SCALE;
    msg.y = dy * SCALE;
    pub_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<SetXY>::SharedPtr                         pub_;  // ★ changed
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixelBridge>());
  rclcpp::shutdown();
  return 0;
}

