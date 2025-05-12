#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ClockPublisher : public rclcpp::Node
{
public:
  ClockPublisher()
  : Node("sim_clock_publisher"), start_time_(std::chrono::steady_clock::now())
  {
    pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ClockPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - start_time_;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() % 1000000000;

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec = static_cast<int32_t>(seconds);
    clock_msg.clock.nanosec = static_cast<uint32_t>(nanoseconds);

    pub_->publish(clock_msg);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
