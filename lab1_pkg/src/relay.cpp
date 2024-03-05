#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class Relay : public rclcpp::Node
{
public:
  Relay()
  : Node("relay")
  {
    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10, std::bind(&Relay::topic_callback, this, _1));
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "driver_relay",10);
  }

private:
  void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const
  {
    double r = msg->drive.speed;
    double s = msg->drive.steering_angle;

    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    message.drive.speed = 3*r;
    message.drive.steering_angle = 3*s;
    publisher_->publish(message);
  }
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
