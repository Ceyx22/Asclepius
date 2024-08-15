#ifndef DYNAMIXEL_ARM__AX12a_HPP_
#define DYNAMIXEL_ARM__AX12a_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using hardware_interface::return_type;

namespace ax_12a {

    struct JointValue {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
    };

    struct Joint {
        JointValue state{};
        JointValue command{};
        JointValue prev_command{};
    };

    enum class ControlMode {
        Position,
        Velocity,
        Torque,
    };

    class ax_12a : public hardware_interface::SystemInterface {
        public:
            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            return_type enable_torque(const bool enabled);

            return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

            return_type reset_command();

            CallbackReturn set_joint_positions();
            CallbackReturn set_joint_velocities();
            CallbackReturn set_joint_params();

            // DynamixelWorkbench dynamixel_workbench_;
            // std::map<const char * const, const ControlItem *> control_items_;
            std::vector<Joint> joints_;
            std::vector<uint8_t> joint_ids_;
            bool torque_enabled_{false};
            ControlMode control_mode_{ControlMode::Position};
            bool mode_changed_{false};
            bool use_dummy_{false};
};

}  // namespace ros2_control_demo_example_7

#endif  // DYNAMIXEL_ARM__AX12a_HPP_