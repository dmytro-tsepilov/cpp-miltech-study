#include <iceoryx_compare_demo/msg/payload_hundred_mb.hpp>
#include <iceoryx_compare_demo/msg/payload_one_mb.hpp>
#include <iceoryx_compare_demo/msg/payload_ten_mb.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {

using PayloadOneMb = iceoryx_compare_demo::msg::PayloadOneMb;
using PayloadTenMb = iceoryx_compare_demo::msg::PayloadTenMb;
using PayloadHundredMb = iceoryx_compare_demo::msg::PayloadHundredMb;

constexpr char kTopicOneMb[] = "/demo/loaned_payload_1mb";
constexpr char kTopicTenMb[] = "/demo/loaned_payload_10mb";
constexpr char kTopicHundredMb[] = "/demo/loaned_payload_100mb";

constexpr int kExpectedSamplesPerPayload = 4;
constexpr auto kIdleTimeout = std::chrono::seconds{15};
constexpr auto kWatchdogPeriod = std::chrono::milliseconds{250};

std::uint64_t steady_now_ns()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

double ns_to_ms(const std::uint64_t ns)
{
  return static_cast<double>(ns) / 1'000'000.0;
}

const char* env_or(const char* name, const char* fallback)
{
  const auto* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

const char* transport_label()
{
  return env_or("DEMO_TRANSPORT_LABEL", env_or("RMW_IMPLEMENTATION", "rmw_unset"));
}

struct TimingStats {
  int samples{0};
  std::size_t payload_bytes{0};
  double total_ms{0.0};
  double max_ms{0.0};

  double add(const std::uint64_t duration_ns, const std::size_t new_payload_bytes)
  {
    const auto duration_ms = ns_to_ms(duration_ns);
    ++samples;
    payload_bytes = new_payload_bytes;
    total_ms += duration_ms;
    if (duration_ms > max_ms) {
      max_ms = duration_ms;
    }
    return duration_ms;
  }

  double average_ms() const { return samples == 0 ? 0.0 : total_ms / static_cast<double>(samples); }
};

void print_callback_result(const char* payload_label, const TimingStats& stats)
{
  std::cout << std::fixed << std::setprecision(3) << "CALLBACK_RESULT" << " transport=" << transport_label() << " payload=" << payload_label
            << " samples=" << stats.samples << " payload_bytes=" << stats.payload_bytes << " avg_callback_age_ms=" << stats.average_ms()
            << " max_callback_age_ms=" << stats.max_ms << '\n';
}

}  // namespace

class Ros2LoanedPayloadSubscriber final : public rclcpp::Node {
public:
  Ros2LoanedPayloadSubscriber()
    : Node("ros2_payload_subscriber")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable().durability_volatile();

    subscription_1mb_ =
      create_subscription<PayloadOneMb>(kTopicOneMb, qos, [this](const PayloadOneMb::ConstSharedPtr message) { on_1mb(*message); });

    subscription_10mb_ =
      create_subscription<PayloadTenMb>(kTopicTenMb, qos, [this](const PayloadTenMb::ConstSharedPtr message) { on_10mb(*message); });

    subscription_100mb_ = create_subscription<PayloadHundredMb>(
      kTopicHundredMb, qos, [this](const PayloadHundredMb::ConstSharedPtr message) { on_100mb(*message); });

    watchdog_ = create_wall_timer(kWatchdogPeriod, [this]() { finish_if_idle(); });

    RCLCPP_INFO(get_logger(), "waiting for %d samples for each payload size: 1MB, 10MB, 100MB", kExpectedSamplesPerPayload);
  }

private:
  void on_1mb(const PayloadOneMb& message)
  {
    record_sample(
      "1MB", message.seq, message.published_steady_ns, message.payload_bytes, message.data.front(), message.data.back(), stats_1mb_);
  }

  void on_10mb(const PayloadTenMb& message)
  {
    record_sample(
      "10MB", message.seq, message.published_steady_ns, message.payload_bytes, message.data.front(), message.data.back(), stats_10mb_);
  }

  void on_100mb(const PayloadHundredMb& message)
  {
    record_sample(
      "100MB", message.seq, message.published_steady_ns, message.payload_bytes, message.data.front(), message.data.back(), stats_100mb_);
  }

  void record_sample(const char* payload_label,
                     const std::uint64_t sequence,
                     const std::uint64_t published_steady_ns,
                     const std::uint32_t payload_bytes,
                     const std::uint8_t first_byte,
                     const std::uint8_t last_byte,
                     TimingStats& stats)
  {
    const auto now = steady_now_ns();
    const auto callback_age_ns = now >= published_steady_ns ? now - published_steady_ns : 0U;
    const auto callback_age_ms = stats.add(callback_age_ns, payload_bytes);

    last_sample_steady_ns_ = now;
    payload_marker_sum_ += static_cast<std::uint64_t>(first_byte) + last_byte;

    std::cout << std::fixed << std::setprecision(3) << "SAMPLE" << " transport=" << transport_label() << " payload=" << payload_label
              << " seq=" << sequence << " payload_bytes=" << payload_bytes << " callback_age_ms=" << callback_age_ms << '\n';

    if (all_samples_received()) {
      finish();
    }
  }

  bool all_samples_received() const
  {
    return stats_1mb_.samples >= kExpectedSamplesPerPayload && stats_10mb_.samples >= kExpectedSamplesPerPayload &&
           stats_100mb_.samples >= kExpectedSamplesPerPayload;
  }

  bool any_samples_received() const { return stats_1mb_.samples > 0 || stats_10mb_.samples > 0 || stats_100mb_.samples > 0; }

  void finish_if_idle()
  {
    if (finished_ || !any_samples_received()) {
      return;
    }

    const auto now = steady_now_ns();
    const auto idle_ns = now >= last_sample_steady_ns_ ? now - last_sample_steady_ns_ : 0U;
    if (idle_ns >= static_cast<std::uint64_t>(kIdleTimeout.count()) * 1'000'000'000ULL) {
      finish();
    }
  }

  void finish()
  {
    if (finished_) {
      return;
    }

    finished_ = true;

    std::cout << '\n';
    print_callback_result("1MB", stats_1mb_);
    print_callback_result("10MB", stats_10mb_);
    print_callback_result("100MB", stats_100mb_);
    RCLCPP_INFO(get_logger(), "payload_marker_sum=%llu", static_cast<unsigned long long>(payload_marker_sum_));

    rclcpp::shutdown();
  }

  bool finished_{false};
  std::uint64_t last_sample_steady_ns_{0};
  std::uint64_t payload_marker_sum_{0};
  TimingStats stats_1mb_;
  TimingStats stats_10mb_;
  TimingStats stats_100mb_;
  rclcpp::Subscription<PayloadOneMb>::SharedPtr subscription_1mb_;
  rclcpp::Subscription<PayloadTenMb>::SharedPtr subscription_10mb_;
  rclcpp::Subscription<PayloadHundredMb>::SharedPtr subscription_100mb_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2LoanedPayloadSubscriber>());
  rclcpp::shutdown();
  return 0;
}
