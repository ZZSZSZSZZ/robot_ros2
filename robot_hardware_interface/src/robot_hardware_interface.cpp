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

        // 初始化关节位置数组和速度数组
        const size_t total_joints = info_.joints.size();
        pos_commands_.resize(total_joints, 0.0);
        vel_commands_.resize(total_joints, 0.0);
        pos_states_.resize(total_joints, 0.0);
        vel_states_.resize(total_joints, 0.0);

        joint_offsets_.resize(total_joints, 0.0);
        joint_multipliers_.resize(total_joints, 1);

        // 从hardware_info的joints[i].parameters中读取offset和multiplier
        for (size_t i = 0; i < total_joints; i++) {
            const auto parameters = info.joints[i].parameters;

            auto offset_it = parameters.find("offset");
            if (offset_it != parameters.end()) {
                joint_offsets_[i] = std::stof(offset_it->second);
            }

            auto multiplier_it = parameters.find("multiplier");
            if (multiplier_it != parameters.end()) {
                joint_multipliers_[i] = std::stof(multiplier_it->second);
            }
        }

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
        return CallbackReturn::ERROR;
    }

    hardware_interface::return_type RobotHardwareInterface::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::vector <std::vector<float>> pos_;

        if (robot_->read(&pos_)) {
            for (auto i = 0ul; i < pos_.size(); i++) {
                if (pos_[i][0] == 1553){
                    pos_states_[i] = static_cast<double>(joint_multipliers_[i] * pos_[i][1] + joint_offsets_[i])/2500;
                } else{
                    pos_states_[i] = static_cast<double>(joint_multipliers_[i] * pos_[i][1] + joint_offsets_[i]);
                }
                vel_states_[i] = static_cast<double>(pos_[i][2]);
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
        pos_commands_om_.resize(19, 0.0);

        for (auto i = 0ul; i < 19; i++) {
            // 将ROS中的命令转换为硬件命令：先减去偏移量，再乘以系数（注意，这里系数可能是-1，表示反转）
            pos_commands_om_[i] = static_cast<float>(joint_multipliers_[i] * (pos_commands_[i] - joint_offsets_[i]));
//            pos_commands_om_[i] = static_cast<float>(pos_commands_[i]);
        }

        std::vector<std::vector<float>> pos_commands_write;
//        pos_commands_write.resize(2, std::vector<float>(19, 0.0));
        pos_commands_write.push_back(pos_commands_om_);
        std::vector<float> floatVec(vel_commands_.begin(), vel_commands_.end());
        pos_commands_write.push_back(floatVec);

        printf("hardware_write\n");
        for (int i = 0; i < 2; ++i) {
            for (auto j = 0ul; j < pos_commands_.size(); j++) {
                printf("%f ", pos_commands_write[i][j]);
            }
            printf("\n");
        }

        if (robot_->write(pos_commands_write)) {
            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::ERROR;
    }

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface
)
