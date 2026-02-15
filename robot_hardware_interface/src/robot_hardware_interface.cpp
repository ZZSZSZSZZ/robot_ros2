#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <algorithm>

#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot/robot.hpp"
#include "robot/robot_device.hpp"

namespace robot_hardware_interface {

    hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
            const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        const size_t total_joints = info_.joints.size();

        // 预分配内存
        pos_commands_.resize(total_joints, 0.0);
        vel_commands_.resize(total_joints, 0.0);
        pos_states_.resize(total_joints, 0.0);
        vel_states_.resize(total_joints, 0.0);
        joint_offsets_.resize(total_joints, 0.0f);
        joint_multipliers_.resize(total_joints, 1);

        // 从参数读取偏移和系数
        for (size_t i = 0; i < total_joints; ++i) {
            const auto &parameters = info_.joints[i].parameters;

            auto offset_it = parameters.find("offset");
            if (offset_it != parameters.end()) {
                try {
                    joint_offsets_[i] = std::stof(offset_it->second);
                } catch (const std::exception &e) {
                    RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
                                "关节 %zu 偏移量解析失败: %s", i, e.what());
                    return CallbackReturn::ERROR;
                }
            }

            auto multiplier_it = parameters.find("multiplier");
            if (multiplier_it != parameters.end()) {
                try {
                    joint_multipliers_[i] = std::stoi(multiplier_it->second);
                } catch (const std::exception &e) {
                    RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
                                "关节 %zu 系数解析失败: %s", i, e.what());
                    return CallbackReturn::ERROR;
                }
            }
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        try {
            // 创建机器人对象
            robot_ = std::make_unique<robot::Robot>(8080);

            if (!robot_) {
                RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "创建机器人对象失败");
                return CallbackReturn::ERROR;
            }

            if (!robot_->init_robot(60000)) {  // 1分钟超时
                RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "机器人初始化失败");
                return CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "机器人初始化完成");
            return CallbackReturn::SUCCESS;

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "配置过程中发生异常: %s", e.what());
            return CallbackReturn::ERROR;
        }
    }

    std::vector <hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces() {
        std::vector <hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &pos_states_[i]
            );
            state_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY,
                    &vel_states_[i]
            );
        }
        return state_interfaces;
    }

    std::vector <hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces() {
        std::vector <hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &pos_commands_[i]
            );
            command_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY,
                    &vel_commands_[i]
            );
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        if (!robot_->enable_robot()) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "机器人激活失败");
            return CallbackReturn::ERROR;
        }

        if (!robot_->read_robot()) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "机器人发送读取命令失败");
            return CallbackReturn::ERROR;
        }

        // 激活后立即读取一次状态
//        std::vector <std::vector<float>> initial_state;
//        if (robot_->read(&initial_state)) {
//            RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "激活后成功读取初始状态");
//        } else {
//            RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "激活后读取初始状态失败");
//            return CallbackReturn::ERROR;
//        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "机器人已激活");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        if (robot_->is_connected()) {
            if (!robot_->disable_robot()) {
                RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "机器人失活过程中发生错误");
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "机器人已失活");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotHardwareInterface::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        std::vector <std::vector<float>> state_data;
//        const int timeout_ms = std::max(100, static_cast<int>(period.seconds() * 1000 * 0.5));
        const int timeout_ms = 1000;

        if (!robot_->read(&state_data, timeout_ms)) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RobotHardwareInterface"),
                                 *rclcpp::Clock().make_shared(),
                                 1000,
                                 "读取机器人状态失败 (超时: %dms)", timeout_ms);

            return hardware_interface::return_type::OK;  // 保持上一次状态
        }

        // 更新关节状态
        const size_t num_joints = std::min(info_.joints.size(), state_data.size());
        for (size_t i = 0; i < num_joints; ++i) {
            if (state_data[i].size() >= 2) {
                float raw_value = state_data[i][1];
                float transformed_value = joint_multipliers_[i] * raw_value + joint_offsets_[i];

                if (state_data[i][0] == 1553) {
                    pos_states_[i] = static_cast<double>(transformed_value) / 2500.0;
                } else {
                    pos_states_[i] = static_cast<double>(transformed_value);
                }

                vel_states_[i] = static_cast<double>(state_data[i][2]);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        // 准备写入数据
        const size_t num_joints = info_.joints.size();
        std::vector <std::vector<float>> write_data(3);
        write_data[0].resize(num_joints);
        write_data[1].resize(num_joints);
        write_data[2].resize(num_joints);

        for (size_t i = 0; i < num_joints; ++i) {
            write_data[0][i] = static_cast<float>(
                    joint_multipliers_[i] * (pos_commands_[i] - static_cast<double>(joint_offsets_[i])));
//            write_data[1][i] = static_cast<float>(vel_commands_[i]);

            float t = 20.0f * (write_data[0][i] - pos_states_[i]) + 2.75f * vel_states_[i];
            write_data[2][i] = t;
        }

        if (!robot_->write(write_data)) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RobotHardwareInterface"),
                                 *rclcpp::Clock().make_shared(),
                                 1000,
                                 "写入机器人命令失败");

            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::OK;
    }
}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface
)