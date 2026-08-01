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
#include <utility>

namespace {

using PayloadOneMb = iceoryx_compare_demo::msg::PayloadOneMb;
using PayloadTenMb = iceoryx_compare_demo::msg::PayloadTenMb;
using PayloadHundredMb = iceoryx_compare_demo::msg::PayloadHundredMb;

constexpr char kTopicOneMb[] = "/demo/loaned_payload_1mb";
constexpr char kTopicTenMb[] = "/demo/loaned_payload_10mb";
constexpr char kTopicHundredMb[] = "/demo/loaned_payload_100mb";

constexpr int kRounds = 4;
constexpr int kPayloadKinds = 3;
constexpr auto kPublishPeriod = std::chrono::milliseconds{700};
constexpr auto kAckTimeout = std::chrono::seconds{10};

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
  double total_ms{0.0};
  double max_ms{0.0};

  double add(const std::uint64_t duration_ns)
  {
    const auto duration_ms = ns_to_ms(duration_ns);
    ++samples;
    total_ms += duration_ms;
    if (duration_ms > max_ms) {
      max_ms = duration_ms;
    }
    return duration_ms;
  }

  double average_ms() const { return samples == 0 ? 0.0 : total_ms / static_cast<double>(samples); }
};

void print_publish_result(const char* payload_label, const std::uint64_t payload_bytes, const TimingStats& stats)
{
  std::cout << std::fixed << std::setprecision(3) << "PUBLISH_RESULT" << " transport=" << transport_label() << " payload=" << payload_label
            << " samples=" << stats.samples << " payload_bytes=" << payload_bytes << " avg_publish_call_ms=" << stats.average_ms()
            << " max_publish_call_ms=" << stats.max_ms << '\n';
}

}  // namespace

class Ros2LoanedPayloadPublisher final : public rclcpp::Node {
public:
  Ros2LoanedPayloadPublisher()
    : Node("ros2_payload_publisher")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable().durability_volatile();

    publisher_1mb_ = create_publisher<PayloadOneMb>(kTopicOneMb, qos);
    publisher_10mb_ = create_publisher<PayloadTenMb>(kTopicTenMb, qos);
    publisher_100mb_ = create_publisher<PayloadHundredMb>(kTopicHundredMb, qos);

    timer_ = create_wall_timer(kPublishPeriod, [this]() { publish_next(); });

    RCLCPP_INFO(get_logger(),
                "transport=%s rmw=%s cyclonedds_uri=%s loaned_1mb=%s loaned_10mb=%s loaned_100mb=%s rounds=%d",
                transport_label(),
                env_or("RMW_IMPLEMENTATION", "rmw_unset"),
                env_or("CYCLONEDDS_URI", "unset"),
                publisher_1mb_->can_loan_messages() ? "true" : "false",
                publisher_10mb_->can_loan_messages() ? "true" : "false",
                publisher_100mb_->can_loan_messages() ? "true" : "false",
                kRounds);
  }

private:
  bool subscribers_ready() const
  {
    return publisher_1mb_->get_subscription_count() > 0 && publisher_10mb_->get_subscription_count() > 0 &&
           publisher_100mb_->get_subscription_count() > 0;
  }

  void publish_next()
  {
    if (published_messages_ >= kRounds * kPayloadKinds) {
      finish();
      return;
    }

    if (!subscribers_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for subscribers on all payload topics");
      return;
    }

    const auto round = static_cast<std::uint64_t>(published_messages_ / kPayloadKinds);
    const auto payload_index = published_messages_ % kPayloadKinds;

    if (payload_index == 0) {
      publish_1mb(round);
    }
    else if (payload_index == 1) {
      publish_10mb(round);
    }
    else {
      publish_100mb(round);
    }

    ++published_messages_;
  }

  void publish_1mb(const std::uint64_t sequence)
  {
    auto loaned_message = publisher_1mb_->borrow_loaned_message();
    auto& message = loaned_message.get();
    fill_message(message, sequence);

    const auto before_publish_ns = steady_now_ns();
    publisher_1mb_->publish(std::move(loaned_message));
    const auto publish_call_ms = publish_call_1mb_.add(steady_now_ns() - before_publish_ns);

    log_publish("1MB", sequence, PayloadOneMb::PAYLOAD_BYTES, publish_call_ms);
  }

  void publish_10mb(const std::uint64_t sequence)
  {
    auto loaned_message = publisher_10mb_->borrow_loaned_message();
    auto& message = loaned_message.get();
    fill_message(message, sequence);

    const auto before_publish_ns = steady_now_ns();
    publisher_10mb_->publish(std::move(loaned_message));
    const auto publish_call_ms = publish_call_10mb_.add(steady_now_ns() - before_publish_ns);

    log_publish("10MB", sequence, PayloadTenMb::PAYLOAD_BYTES, publish_call_ms);
  }

  void publish_100mb(const std::uint64_t sequence)
  {
    auto loaned_message = publisher_100mb_->borrow_loaned_message();
    auto& message = loaned_message.get();
    fill_message(message, sequence);

    const auto before_publish_ns = steady_now_ns();
    publisher_100mb_->publish(std::move(loaned_message));
    const auto publish_call_ms = publish_call_100mb_.add(steady_now_ns() - before_publish_ns);

    log_publish("100MB", sequence, PayloadHundredMb::PAYLOAD_BYTES, publish_call_ms);
  }

  void fill_message(PayloadOneMb& message, const std::uint64_t sequence)
  {
    fill_common_fields(message.seq,
                       message.published_steady_ns,
                       message.payload_bytes,
                       static_cast<std::uint32_t>(message.data.size()),
                       message.data.front(),
                       message.data.back(),
                       sequence);
  }

  void fill_message(PayloadTenMb& message, const std::uint64_t sequence)
  {
    fill_common_fields(message.seq,
                       message.published_steady_ns,
                       message.payload_bytes,
                       static_cast<std::uint32_t>(message.data.size()),
                       message.data.front(),
                       message.data.back(),
                       sequence);
  }

  void fill_message(PayloadHundredMb& message, const std::uint64_t sequence)
  {
    fill_common_fields(message.seq,
                       message.published_steady_ns,
                       message.payload_bytes,
                       static_cast<std::uint32_t>(message.data.size()),
                       message.data.front(),
                       message.data.back(),
                       sequence);
  }

  void fill_common_fields(std::uint64_t& sequence_field,
                          std::uint64_t& published_steady_ns,
                          std::uint32_t& payload_bytes,
                          const std::uint32_t payload_size,
                          std::uint8_t& first_byte,
                          std::uint8_t& last_byte,
                          const std::uint64_t sequence)
  {
    sequence_field = sequence;
    published_steady_ns = steady_now_ns();
    payload_bytes = payload_size;

    // Touch a few bytes so the payload is real without making memset the demo bottleneck.
    const auto marker = static_cast<std::uint8_t>(sequence & 0xFFU);
    first_byte = marker;
    last_byte = marker;
  }

  void log_publish(const char* payload_label,
                   const std::uint64_t sequence,
                   const std::uint64_t payload_bytes,
                   const double publish_call_ms) const
  {
    RCLCPP_INFO(get_logger(),
                "PUBLISH payload=%s seq=%llu bytes=%llu publish_call_ms=%.3f",
                payload_label,
                static_cast<unsigned long long>(sequence),
                static_cast<unsigned long long>(payload_bytes),
                publish_call_ms);
  }

  void finish()
  {
    if (finished_) {
      return;
    }

    finished_ = true;
    RCLCPP_INFO(get_logger(), "finished publishing %d samples per payload size", kRounds);

    // Keep the process alive until reliable DDS delivery includes the final 100 MB sample.
    const auto acked_100mb = publisher_100mb_->wait_for_all_acked(kAckTimeout);
    const auto acked_10mb = publisher_10mb_->wait_for_all_acked(kAckTimeout);
    const auto acked_1mb = publisher_1mb_->wait_for_all_acked(kAckTimeout);

    RCLCPP_INFO(get_logger(),
                "reliable_delivery acked_1mb=%s acked_10mb=%s acked_100mb=%s timeout_ms=%lld",
                acked_1mb ? "true" : "false",
                acked_10mb ? "true" : "false",
                acked_100mb ? "true" : "false",
                static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(kAckTimeout).count()));

    std::cout << '\n';
    print_publish_result("1MB", PayloadOneMb::PAYLOAD_BYTES, publish_call_1mb_);
    print_publish_result("10MB", PayloadTenMb::PAYLOAD_BYTES, publish_call_10mb_);
    print_publish_result("100MB", PayloadHundredMb::PAYLOAD_BYTES, publish_call_100mb_);

    rclcpp::shutdown();
  }

  bool finished_{false};
  int published_messages_{0};
  TimingStats publish_call_1mb_;
  TimingStats publish_call_10mb_;
  TimingStats publish_call_100mb_;
  rclcpp::Publisher<PayloadOneMb>::SharedPtr publisher_1mb_;
  rclcpp::Publisher<PayloadTenMb>::SharedPtr publisher_10mb_;
  rclcpp::Publisher<PayloadHundredMb>::SharedPtr publisher_100mb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2LoanedPayloadPublisher>());
  rclcpp::shutdown();
  return 0;
}
