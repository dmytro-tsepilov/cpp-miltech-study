#include "qos_profiles_demo/qos_common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace demo = qos_profiles_demo;

class QosCounterPublisher final : public rclcpp::Node {
public:
  QosCounterPublisher()
    : Node("qos_counter_publisher")
  {
    topic_ = declare_parameter<std::string>("topic", "/qos_demo/counter");
    reliability_ = declare_parameter<std::string>("reliability", "best_effort");
    durability_ = declare_parameter<std::string>("durability", "volatile");
    depth_ = declare_parameter<int>("depth", 4);
    period_ms_ = declare_parameter<int>("period_ms", 100);
    stop_after_ = declare_parameter<int>("stop_after", 20);
    linger_ms_ = declare_parameter<int>("linger_ms", 0);

    const auto qos = demo::make_qos(reliability_, durability_, depth_);
    publisher_ = create_publisher<std_msgs::msg::UInt64>(topic_, qos);
    demo::print_profile("publisher", topic_, reliability_, durability_, depth_);

    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), [this]() { publish_next(); });
  }

private:
  void publish_next()
  {
    if (finished_) {
      return;
    }

    if (stop_after_ > 0 && published_ >= static_cast<std::uint64_t>(stop_after_)) {
      finished_ = true;
      std::cout << "SUMMARY role=publisher published=" << published_ << " linger_ms=" << linger_ms_ << '\n';
      timer_->cancel();
      if (linger_ms_ <= 0) {
        rclcpp::shutdown();
        return;
      }
      shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(linger_ms_), []() { rclcpp::shutdown(); });
      return;
    }

    std_msgs::msg::UInt64 message;
    message.data = published_;
    publisher_->publish(message);
    std::cout << "PUB seq=" << published_ << '\n';
    ++published_;
  }

  std::string topic_{"/qos_demo/counter"};
  std::string reliability_{"best_effort"};
  std::string durability_{"volatile"};
  int depth_{4};
  int period_ms_{100};
  int stop_after_{20};
  int linger_ms_{0};
  bool finished_{false};
  std::uint64_t published_{0};
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosCounterPublisher>());
  rclcpp::shutdown();
  return 0;
}
