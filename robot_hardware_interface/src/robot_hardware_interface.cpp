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
        total_joints_ = info_.joints.size();
        pos_commands_.resize(total_joints_, 0.0);
        vel_commands_.resize(total_joints_, 0.0);
        acc_commands_.resize(total_joints_, 0.0);

        pos_states_.resize(total_joints_, 0.0);
        vel_states_.resize(total_joints_, 0.0);

        joint_offsets_.resize(total_joints_, 0.0);
        joint_multipliers_.resize(total_joints_, 1);

        // 从hardware_info的joints[i].parameters中读取offset和multiplier
        for (size_t i = 0; i < total_joints_; i++) {
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

//        for (int i = 1; i <= 7; ++i) {
//            robot::motor::MotorConfig cfg;
//            cfg.id = i;
//            cfg.name = "Joint_" + std::to_string(i);
//            cfg.type = (i <= 4) ? "EYOU_PP11" : "EYOU_PP11L";  // 前4个用 PP11，后3个用 PP11L
//            cfg.tx_can_id = i;
//            cfg.rx_can_id = i;
//            cfg.interface_name = "can0";  // 多接口模式下可指定不同接口
//            cfg.useStandardFrame(8);
//
//            arm_motor_configs_.push_back(cfg);
//        }

        for (int i = 1; i <= 3; ++i) {
            robot::motor::MotorConfig cfg;
            cfg.id = i;
            cfg.name = "Joint_" + std::to_string(i);
            cfg.type = (i < 3) ? "EYOU_PP11" : "EYOU_PP11L"; // 前2个用 PP11，后1个用 PP11L
            cfg.tx_can_id = i;
            cfg.rx_can_id = i;
            cfg.interface_name = "can0";  // 多接口模式下可指定不同接口
            cfg.useStandardFrame(8);

            arm_motor_configs_.push_back(cfg);
        }

        return CallbackReturn::SUCCESS;
    }

    // 初始化硬件连接
    hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        robot::Robot::Options options;
        options.enable_can_fd = false; // 不使用 CAN FD
        options.receive_thread_priority = 50; // 接收线程优先级
        options.status_poll_interval_ms = 0; // 状态轮询间隔 10ms
        options.enable_background_thread = true; // 启用后台轮询
        options.skip_poll_if_disabled = true; // 未使能时跳过查询

        robot_ = std::make_unique<robot::Robot>("can0", options);

        // 硬件实例化
        auto result = robot_->initialize();
        if (result.isError()) return CallbackReturn::ERROR;

        arm_component_ = robot_->createArmComponent("LeftArm", arm_motor_configs_);
        if (!arm_component_) return CallbackReturn::ERROR;

        return CallbackReturn::SUCCESS;
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
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &acc_commands_[i]));
        }

        return command_interfaces;
    }

    // 使能硬件并向硬件发送读取命令
    hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        if (!robot_->enableAllMotors()) return CallbackReturn::ERROR;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return CallbackReturn::SUCCESS;
    }

    // 失能硬件
    hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        if (!robot_->disableAllMotors()) return CallbackReturn::ERROR;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        robot_->shutdown();

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotHardwareInterface::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        std::unordered_map <uint32_t, robot::motor::MotorState> states = robot_->getAllMotorState();

//        for (size_t i = 0; i < total_joints_; i++) {
//            pos_states_[i] = static_cast<double>(joint_multipliers_[i] * states[i].position + joint_offsets_[i]);
//            vel_states_[i] = static_cast<double>(states[i].velocity);
//        }

        for (int i = 0; i < states.size(); i++) {
            robot::motor::MotorState state = states.at(i + 1);
            pos_states_[i] = joint_multipliers_[i] * state.position + joint_offsets_[i];
            vel_states_[i] = state.velocity;
        }

        printf("hardware_read\n");
        for (size_t i = 0; i < pos_states_.size(); i++) {
            printf("%f ", pos_states_[i]);
        }
        printf("\n");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::vector <robot::PositionParam> position_param;
//        position_param.reserve(total_joints_);
        position_param.resize(3);

        for (int i = 0; i < 3; i++) { //TODO 注意循环次数
            // 将ROS中的命令转换为硬件命令：先减去偏移量，再乘以系数（注意，这里系数可能是-1，表示反转）
            double position = joint_multipliers_[i] * (pos_commands_[i] - joint_offsets_[i]);
//            position_param[i] = {position, vel_commands_[i], def_torque_[i], acc_commands_[i]};
            position_param[i] = {position, vel_commands_[i], 1, 0.5};
        }

        printf("position_param_position\n");
        for (auto i = 0ul; i < 3; i++) {
            printf("%f ", position_param[i].position);
        }
        printf("\n");

        printf("position_param_vel\n");
        for (auto i = 0ul; i < 3; i++) {
            printf("%f ", position_param[i].velocity);
        }
        printf("\n");

        arm_component_->setPositions(position_param);

        return hardware_interface::return_type::OK;
    }

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface
)
