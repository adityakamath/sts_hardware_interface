/**
 * @file motor_diagnostics_node.cpp
 * @brief Real-time motor health monitoring for STS servo motor chains
 *
 * Monitors motor temperature, voltage, current, and detects internal stalls
 * by analyzing state interface feedback from the hardware interface (via state_interfaces_broadcaster).
 */

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <control_msgs/msg/float64_values.hpp>
#include <control_msgs/msg/keys.hpp>
#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cstdio>

/**
 * @class MotorDiagnosticsNode
 * @brief Node for monitoring motor health and detecting failure conditions
 *
 * Subscribes to <state_topic>/names and <state_topic>/values and publishes diagnostic messages
 * when motors exceed temperature, voltage, current, or stall thresholds.
 */
class MotorDiagnosticsNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor - initializes parameters and creates pub/sub
   */
  MotorDiagnosticsNode() : Node("motor_diagnostics_node") {
    // Declare parameters with defaults
    this->declare_parameter("effort_threshold", 0.5);
    this->declare_parameter("current_threshold", 1.5);
    this->declare_parameter("voltage_min", 6.0);
    this->declare_parameter("voltage_max", 14.0);
    this->declare_parameter("temp_warn", 60.0);
    this->declare_parameter("temp_error", 75.0);
    this->declare_parameter("current_max", 3.0);
    this->declare_parameter("state_topic", "/state_interfaces_broadcaster");

    // Get parameters
    effort_threshold_ = this->get_parameter("effort_threshold").as_double();
    current_threshold_ = this->get_parameter("current_threshold").as_double();
    voltage_min_ = this->get_parameter("voltage_min").as_double();
    voltage_max_ = this->get_parameter("voltage_max").as_double();
    temp_warn_ = this->get_parameter("temp_warn").as_double();
    temp_error_ = this->get_parameter("temp_error").as_double();
    current_max_ = this->get_parameter("current_max").as_double();
    const std::string state_topic = this->get_parameter("state_topic").as_string();

    // names is latched (transient local) and published once; values stream at the controller rate
    names_sub_ = this->create_subscription<control_msgs::msg::Keys>(
        state_topic + "/names", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&MotorDiagnosticsNode::names_callback, this, std::placeholders::_1));
    values_sub_ = this->create_subscription<control_msgs::msg::Float64Values>(
        state_topic + "/values", rclcpp::SensorDataQoS(),
        std::bind(&MotorDiagnosticsNode::values_callback, this, std::placeholders::_1));

    // Create publisher. Reliable, not SensorDataQoS - this carries stall/overheat/overcurrent
    // warnings, not a high-rate sensor stream, and a best-effort warning is a warning that can
    // silently vanish. Matches bno055_diagnostics' QoS for the same /diagnostics topic.
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(this->get_logger(), "Motor diagnostics node started");
    RCLCPP_INFO(this->get_logger(),
                "Stall thresholds - Effort: %.2fNm, Current: %.2fA",
                effort_threshold_, current_threshold_);
  }

private:
  double effort_threshold_; // Effort threshold for stall detection (N⋅m)
  double current_threshold_; // Current threshold for stall detection (A)
  double voltage_min_; // Minimum acceptable voltage (V)
  double voltage_max_; // Maximum acceptable voltage (V)
  double temp_warn_; // Temperature warning threshold (°C)
  double temp_error_; // Temperature error threshold (°C)
  double current_max_; // Maximum current limit (A)

  rclcpp::Subscription<control_msgs::msg::Keys>::SharedPtr names_sub_; // "<joint>/<interface>" keys
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr values_sub_; // Values, same order as keys
  std::vector<std::string> keys_; // Latest keys from the broadcaster
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_; // Publisher for diagnostic messages

  /**
   * @brief Store the key list published by state_interfaces_broadcaster
   * @param msg Keys of the form "<joint>/<interface>"
   */
  void names_callback(const control_msgs::msg::Keys::SharedPtr msg) {
    keys_ = msg->keys;
  }

  /**
   * @brief Process state interface values and publish diagnostics
   * @param msg Values in the same order as the latest keys
   */
  void values_callback(const control_msgs::msg::Float64Values::SharedPtr msg) {
    if (keys_.empty() || msg->values.size() != keys_.size()) return;

    // Group "<joint>/<interface>" into per-joint maps, keeping joint order of first appearance
    std::vector<std::string> joint_order;
    std::map<std::string, std::unordered_map<std::string, double>> joints;
    for (size_t i = 0; i < keys_.size(); ++i) {
      const auto slash = keys_[i].rfind('/');
      if (slash == std::string::npos) continue;
      const std::string joint = keys_[i].substr(0, slash);
      if (!joints.count(joint)) joint_order.push_back(joint);
      joints[joint][keys_[i].substr(slash + 1)] = msg->values[i];
    }

    auto diagnostics = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostics->header.stamp = this->now();
    diagnostics->header.frame_id = "base_link";

    for (const auto& joint_name : joint_order) {
      const auto& values_dict = joints[joint_name];
      auto status = diagnostic_msgs::msg::DiagnosticStatus();
      // name, not just hardware_id: diagnostic_aggregator's GenericAnalyzer matches
      // (startswith/contains/regex) only against DiagnosticStatus.name, never hardware_id -
      // an unset name can't be grouped by any analyzer config.
      status.name = "Motor: " + joint_name;
      status.hardware_id = joint_name;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

      status.values.reserve(values_dict.size());
      for (const auto& [key, value] : values_dict) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = std::to_string(value);
        status.values.push_back(std::move(kv));
      }

      // Run health checks
      check_temperature(status, values_dict);
      check_voltage(status, values_dict);
      check_overcurrent(status, values_dict);
      check_stalls(status, values_dict);

      diagnostics->status.push_back(std::move(status));
    }

    diagnostics_pub_->publish(*diagnostics);
  }

  /**
   * @brief Check motor temperature against warning and error thresholds
   * @param status Diagnostic status to update
   * @param values_dict Map of interface names to values
   */
  void check_temperature(diagnostic_msgs::msg::DiagnosticStatus& status,
                          const std::unordered_map<std::string, double>& values_dict) {
    auto temp_it = values_dict.find("temperature");
    if (temp_it == values_dict.end()) {
      return;
    }

    double temp = temp_it->second;
    if (temp > temp_error_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Critical temperature: " + std::to_string(temp) + "°C";
    } else if (temp > temp_warn_) {
      if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "High temperature: " + std::to_string(temp) + "°C";
      }
    }
  }

  /**
   * @brief Check motor voltage is within acceptable range
   * @param status Diagnostic status to update
   * @param values_dict Map of interface names to values
   */
  void check_voltage(diagnostic_msgs::msg::DiagnosticStatus& status,
                     const std::unordered_map<std::string, double>& values_dict) {
    auto voltage_it = values_dict.find("voltage");
    if (voltage_it == values_dict.end()) {
      return;
    }

    double voltage = voltage_it->second;
    if (voltage < voltage_min_) {
      if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Low voltage: " + std::to_string(voltage) + "V";
      }
    } else if (voltage > voltage_max_) {
      if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Voltage above acceptable limit: " + std::to_string(voltage) + "V";
      }
    }
  }

  /**
   * @brief Check for overcurrent conditions
   * @param status Diagnostic status to update
   * @param values_dict Map of interface names to values
   */
  void check_overcurrent(diagnostic_msgs::msg::DiagnosticStatus& status,
                         const std::unordered_map<std::string, double>& values_dict) {
    auto current_it = values_dict.find("current");
    if (current_it == values_dict.end()) {
      return;
    }

    double current = current_it->second;
    if (std::abs(current) > current_max_) {
      if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Overcurrent: " + std::to_string(current) + "A (max: " + std::to_string(current_max_) + "A)";
      }
    }
  }

  /**
   * @brief Detect internal motor stalls (motor stopped with high effort/current)
   * @param status Diagnostic status to update
   * @param values_dict Map of interface names to values
   */
  void check_stalls(diagnostic_msgs::msg::DiagnosticStatus& status,
                    const std::unordered_map<std::string, double>& values_dict) {
    // Get values
    auto is_moving_it = values_dict.find("is_moving");
    if (is_moving_it == values_dict.end()) {
      return;
    }

    double is_moving = is_moving_it->second;
    double effort = values_dict.count("effort") ? values_dict.at("effort") : 0.0;
    double current = values_dict.count("current") ? values_dict.at("current") : 0.0;

    // INTERNAL STALL: Motor stopped but high effort/current
    if (std::abs(is_moving) < 0.1) {
      bool high_effort = std::abs(effort) > effort_threshold_;
      bool high_current = std::abs(current) > current_threshold_;

      if (high_effort || high_current) {
        if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          if (high_effort && high_current) {
            status.message = "Internal stall: effort " + std::to_string(effort) + "Nm, current " + std::to_string(current) + "A but motor stopped";
          } else if (high_effort) {
            status.message = "Internal stall: effort " + std::to_string(effort) + "Nm but motor stopped";
          } else {
            status.message = "Internal stall: current " + std::to_string(current) + "A but motor stopped";
          }
        }
      }
    }
  }
};

/**
 * @brief Main entry point for motor diagnostics node
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}