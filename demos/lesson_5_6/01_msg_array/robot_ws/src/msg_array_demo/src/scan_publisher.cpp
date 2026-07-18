#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "msg_array_demo/msg/cell_observation_lite.hpp"
#include "msg_array_demo/msg/local_scan_lite.hpp"

namespace {

constexpr auto kScanTopic = "/demo/local_scan";

using CellObservationLite = msg_array_demo::msg::CellObservationLite;
using LocalScanLite = msg_array_demo::msg::LocalScanLite;

CellObservationLite make_cell(
  const int x,
  const int y,
  const char* cell_type,
  const int contact_id = 0)
{
  CellObservationLite cell;
  cell.x = x;
  cell.y = y;
  cell.cell_type = cell_type;
  cell.contact_id = contact_id;
  return cell;
}

LocalScanLite make_scan()
{
  LocalScanLite scan;
  scan.scenario_name = "array_demo";
  scan.robot_x = 2;
  scan.robot_y = 1;

  scan.cells.reserve(9);
  scan.cells.push_back(make_cell(1, 0, "#"));
  scan.cells.push_back(make_cell(2, 0, "#"));
  scan.cells.push_back(make_cell(3, 0, "#"));
  scan.cells.push_back(make_cell(1, 1, "S"));
  scan.cells.push_back(make_cell(2, 1, "."));
  scan.cells.push_back(make_cell(3, 1, "C", 7));
  scan.cells.push_back(make_cell(1, 2, "#"));
  scan.cells.push_back(make_cell(2, 2, "."));
  scan.cells.push_back(make_cell(3, 2, "#"));

  return scan;
}

}  // namespace

class ScanPublisher final : public rclcpp::Node {
public:
  ScanPublisher()
    : Node("scan_publisher")
  {
    publisher_ = create_publisher<LocalScanLite>(kScanTopic, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds{1000}, [this]() { publish_scan(); });
  }

private:
  void publish_scan()
  {
    const auto scan = make_scan();
    publisher_->publish(scan);

    RCLCPP_INFO(
      get_logger(),
      "published local scan robot=(%d,%d) cells=%zu",
      scan.robot_x,
      scan.robot_y,
      scan.cells.size());

    for (const auto& cell : scan.cells) {
      if (cell.cell_type != "C") {
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "visible contact id=%d at cell=(%d,%d)",
        cell.contact_id,
        cell.x,
        cell.y);
    }
  }

  rclcpp::Publisher<LocalScanLite>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
