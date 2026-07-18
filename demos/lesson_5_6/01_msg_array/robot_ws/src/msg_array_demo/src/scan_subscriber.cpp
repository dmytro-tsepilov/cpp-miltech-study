#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "msg_array_demo/msg/local_scan_lite.hpp"

namespace {

constexpr auto kScanTopic = "/demo/local_scan";

using LocalScanLite = msg_array_demo::msg::LocalScanLite;

std::string contact_label(const LocalScanLite::_cells_type::value_type& cell)
{
  if (cell.contact_id == 0) {
    return "none";
  }

  return "contact_id=" + std::to_string(cell.contact_id);
}

}  // namespace

class ScanSubscriber final : public rclcpp::Node {
public:
  ScanSubscriber()
    : Node("scan_subscriber")
  {
    subscription_ = create_subscription<LocalScanLite>(
      kScanTopic,
      10,
      [this](const LocalScanLite& scan) { on_scan(scan); });
  }

private:
  void on_scan(const LocalScanLite& scan)
  {
    RCLCPP_INFO(
      get_logger(),
      "received local scan scenario=%s robot=(%d,%d) cells=%zu",
      scan.scenario_name.c_str(),
      scan.robot_x,
      scan.robot_y,
      scan.cells.size());

    for (const auto& cell : scan.cells) {
      RCLCPP_INFO(
        get_logger(),
        "  cell=(%d,%d) type=%s contact=%s",
        cell.x,
        cell.y,
        cell.cell_type.c_str(),
        contact_label(cell).c_str());
    }
  }

  rclcpp::Subscription<LocalScanLite>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}
