#include <limits>
#include <vector>
#include <algorithm> // 包含 std::transform
#include <iterator>  // 包含 std::back_inserter

#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface {
    // 初始化硬件接口
    hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
            const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // 从hardware_info的joints[i].parameters中读取offset和multiplier
        for (size_t i = 0; i < info.joints.size(); i++) {
            const auto &joint = info.joints[i];

            auto it_offset = joint.parameters.find("offset");
            if (it_offset != joint.parameters.end()) {
                joint_offsets_[i] = std::stod(it_offset->second);
            }

            auto it_multiplier = joint.parameters.find("multiplier");
            if (it_multiplier != joint.parameters.end()) {
                joint_multipliers_[i] = std::stod(it_multiplier->second);
            }
        }

        // 初始化关节位置数组和速度数组
        const size_t total_joints = info_.joints.size();
        pos_commands_.resize(total_joints, std::numeric_limits<float>::quiet_NaN());
        vel_commands_.resize(total_joints, std::numeric_limits<float>::quiet_NaN());
        pos_states_.resize(total_joints, std::numeric_limits<float>::quiet_NaN());
        vel_states_.resize(total_joints, std::numeric_limits<float>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    // 初始化硬件连接
    hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        // 硬件实例化
        robot_ = std::make_unique<robot::Robot>(8080);

        if (robot_->init_robot()) {
            return CallbackReturn::SUCCESS;
        }

        return CallbackReturn::ERROR;
    }

    std::vector <hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces() {
        std::vector <hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
        }

        return state_interfaces;
    }

    std::vector <hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces() {
        std::vector <hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_commands_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]));
        }

        return command_interfaces;
    }

    // 使能硬件并向硬件发送读取命令
    hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        if (robot_->enable_robot()) {
            // 发送读取命令
            robot_->read_robot();
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::ERROR;
    }

    // 失能硬件
    hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        if (robot_->disable_robot()) {
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::ERROR
    }

    hardware_interface::return_type RobotHardwareInterface::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::vector <std::vector<float>> pos_;
//        std::vector <std::vector<double>> pos_states_double_;

        if (robot_->read(&pos_)) {
//            for (const auto &inner_vec_float: pos) {
//                std::vector<double> inner_vec_double;
//                // 对内层 vector 进行转换
//                std::transform(inner_vec_float.begin(), inner_vec_float.end(),
//                               std::back_inserter(inner_vec_double), // 插入到 inner_vec_double
//                               [](float val_f) { return static_cast<double>(val_f); }); // Lambda 转换
//                pos_states_double_.push_back(inner_vec_double);
//            }

            for (auto i = 0ul; i < pos_states_.size(); i++) {
                pos_states_[i] = joint_multipliers_[i] * pos_[i][1] + joint_offsets_[i];
            }

            printf("hardware_read\n");
            for (auto i = 0ul; i < pos_states_.size(); i++) {
                printf("%f ", pos_states_[i]);
                // printf("%f\n", vel_states_[i]);
            }
            printf("\n");

            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::ERROR;
    }

    hardware_interface::return_type RobotHardwareInterface::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::vector<float> pos_commands_om_;

        // 使用 std::transform 和一个 lambda 函数进行转换
//        std::transform(pos_commands_.begin(), pos_commands_.end(),
//                       std::back_inserter(pos_commands_float_), // 将结果插入到 vec_float 末尾
//                       [](double d) { return static_cast<float>(d); }); // 转换函数

        for (auto i = 0ul; i < pos_commands_.size(); i++) {
            // 将ROS中的命令转换为硬件命令：先减去偏移量，再乘以系数（注意，这里系数可能是-1，表示反转）
            pos_commands_om_[i] = joint_multipliers_[i] * (pos_commands_[i] - joint_offsets_[i]);
        }

        printf("hardware_write\n");
        for (auto i = 0ul; i < pos_commands_.size(); i++) {
            printf("%f ", pos_commands_om_[i]);
            // printf("%f\n", vel_commands_[i]);
        }
        printf("\n");

        if (robot_->write(pos_commands_om_)) {
            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::ERROR;
    }

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface
)
