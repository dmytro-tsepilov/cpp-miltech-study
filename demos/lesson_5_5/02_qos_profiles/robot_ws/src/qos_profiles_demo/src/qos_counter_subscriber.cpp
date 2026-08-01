#include "qos_profiles_demo/qos_common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

namespace demo = qos_profiles_demo;

class QosCounterSubscriber final : public rclcpp::Node {
public:
  QosCounterSubscriber()
    : Node("qos_counter_subscriber")
  {
    topic_ = declare_parameter<std::string>("topic", "/qos_demo/counter");
    reliability_ = declare_parameter<std::string>("reliability", "best_effort");
    durability_ = declare_parameter<std::string>("durability", "volatile");
    depth_ = declare_parameter<int>("depth", 4);
    expected_samples_ = declare_parameter<int>("expected_samples", 0);
    max_idle_ms_ = declare_parameter<int>("max_idle_ms", 3000);
    processing_ms_ = declare_parameter<int>("processing_ms", 0);
    last_sample_time_ = std::chrono::steady_clock::now();

    const auto qos = demo::make_qos(reliability_, durability_, depth_);
    subscription_ =
      create_subscription<std_msgs::msg::UInt64>(topic_, qos, [this](const std_msgs::msg::UInt64& message) { on_sample(message); });
    demo::print_profile("subscriber", topic_, reliability_, durability_, depth_);

    watchdog_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { on_watchdog(); });
  }

private:
  void on_sample(const std_msgs::msg::UInt64& message)
  {
    last_sample_time_ = std::chrono::steady_clock::now();
    if (processing_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(processing_ms_));
    }

    if (received_ == 0U) {
      first_seq_ = message.data;
    }
    else if (message.data > last_seq_ + 1U) {
      gaps_ += message.data - last_seq_ - 1U;
    }

    last_seq_ = message.data;
    ++received_;
    std::cout << "SUB seq=" << message.data << " received=" << received_ << " gaps=" << gaps_ << '\n';

    if (expected_samples_ > 0 && received_ >= static_cast<std::uint64_t>(expected_samples_)) {
      finish();
    }
  }

  void on_watchdog()
  {
    if (finished_ || max_idle_ms_ <= 0) {
      return;
    }

    const auto idle = std::chrono::steady_clock::now() - last_sample_time_;
    if (idle >= std::chrono::milliseconds(max_idle_ms_)) {
      finish();
    }
  }

  void finish()
  {
    if (finished_) {
      return;
    }

    finished_ = true;
    std::cout << "SUMMARY" << " role=subscriber" << " received=" << received_ << " first_seq=" << (received_ == 0U ? 0U : first_seq_)
              << " last_seq=" << (received_ == 0U ? 0U : last_seq_) << " gaps=" << gaps_ << '\n';
    rclcpp::shutdown();
  }

  std::string topic_{"/qos_demo/counter"};
  std::string reliability_{"best_effort"};
  std::string durability_{"volatile"};
  int depth_{4};
  int expected_samples_{0};
  int max_idle_ms_{3000};
  int processing_ms_{0};
  bool finished_{false};
  std::uint64_t received_{0};
  std::uint64_t first_seq_{0};
  std::uint64_t last_seq_{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t gaps_{0};
  std::chrono::steady_clock::time_point last_sample_time_{};
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosCounterSubscriber>());
  rclcpp::shutdown();
  return 0;
}
