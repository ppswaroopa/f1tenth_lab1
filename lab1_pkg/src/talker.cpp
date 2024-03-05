#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker")
  {
    this->declare_parameter("speed",0.0);
    this->declare_parameter("steering_angle",0.0); 
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    timer_ = this->create_wall_timer(
      0ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double v,d;
    v = this->get_parameter("speed").as_double();
    d = this->get_parameter("steering_angle").as_double();

    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    message.drive.speed = v;
    message.drive.steering_angle = d;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
