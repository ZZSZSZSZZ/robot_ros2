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

        for (int i = 1; i <= 14; ++i) {
            robot::motor::MotorConfig cfg;
            cfg.id = i;
            cfg.name = "Joint_" + std::to_string(i);
            cfg.type = ((i >= 1 && i <= 4) || (i >= 8 && i <= 11)) ? "EYOU_PP11L"
                                                                   : "EYOU_PP11"; // 1-4, 8-11 用 PP11L，其余用 PP11
            cfg.tx_can_id = i;
            cfg.rx_can_id = i;
            cfg.interface_name = "can0";  // 多接口模式下可指定不同接口
            cfg.useStandardFrame(8);

            arm_motor_configs_.push_back(cfg);
        }

        body_motor_config_.id = 17; // 电机CAN ID
        body_motor_config_.name = "Joint_17"; // 电机名称
        body_motor_config_.type = "PUSHROD_YZ_AIM"; // 电机类型
        body_motor_config_.tx_can_id = body_motor_config_.id; // 发送CAN ID
        body_motor_config_.rx_can_id = body_motor_config_.id; // 接收CAN ID
        body_motor_config_.interface_name = "can0";
        body_motor_config_.useStandardFrame(8); // 顶杆电机使用标准帧

        left_wheel_motor_config_.id = 15;
        left_wheel_motor_config_.name = "Joint_15";
        left_wheel_motor_config_.type = "IWS_IWS45L_1D1_350_B_MCAFC";
        left_wheel_motor_config_.tx_can_id = left_wheel_motor_config_.id;
        left_wheel_motor_config_.rx_can_id = left_wheel_motor_config_.id;
        left_wheel_motor_config_.interface_name = "can0";
        left_wheel_motor_config_.useStandardFrame(8);

        right_wheel_motor_config_.id = 16;
        right_wheel_motor_config_.name = "Joint_16";
        right_wheel_motor_config_.type = "IWS_IWS45L_1D1_350_B_MCAFC";
        right_wheel_motor_config_.tx_can_id = right_wheel_motor_config_.id;
        right_wheel_motor_config_.rx_can_id = right_wheel_motor_config_.id;
        right_wheel_motor_config_.interface_name = "can0";
        right_wheel_motor_config_.useStandardFrame(8);

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

        robot::motor::iws::IwsFactory::registerMotorType();
        robot::motor::pushrod::PushrodFactory::registerMotorType();

        // 硬件实例化
        auto result = robot_->initialize();
        if (result.isError()) return CallbackReturn::ERROR;

        arm_component_ = robot_->createArmComponent("Arm", arm_motor_configs_);
        if (!arm_component_) return CallbackReturn::ERROR;

        pushrod_motor_ = robot::motor::pushrod::PushrodFactory::createMotor(body_motor_config_);
        iws_motor1_ = robot::motor::iws::IwsFactory::createMotor(left_wheel_motor_config_);
        iws_motor2_ = robot::motor::iws::IwsFactory::createMotor(right_wheel_motor_config_);

        auto manager = robot_->getMotorManager();

        manager->addMotor(pushrod_motor_);
        manager->addMotor(iws_motor1_);
        manager->addMotor(iws_motor2_);

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
//        robot_->enableAllMotors();

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

        for (size_t i = 0; i < 19; i++) {
            pos_states_[i] = static_cast<double>(joint_multipliers_[i] * states[i + 1].position + joint_offsets_[i]);
            vel_states_[i] = static_cast<double>(states[i + 1].velocity);
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
        position_param.resize(14);

        for (int i = 0; i < 14; i++) {
            // 将ROS中的命令转换为硬件命令：先减去偏移量，再乘以系数（注意，这里系数可能是-1，表示反转）
            double position = joint_multipliers_[i] * (pos_commands_[i] - joint_offsets_[i]);
//            position_param[i] = {position, vel_commands_[i], def_torque_[i], acc_commands_[i]};
            position_param[i] = {position, vel_commands_[i], 2, 0.1};
        }

        arm_component_->setPositions(position_param);

        for (int i = 15; i < 18; i++) {
            const std::string &joint_name = info_.joints[i].name;

            if (joint_name == "body_joint1") {
                pushrod_motor_->setPosition(pos_commands_[i], 1000, 1.0);
            } else if (joint_name == "body_left_wheel_joint") {
                iws_motor2_->setVelocity(vel_commands_[i], 1.0);
            } else if (joint_name == "body_right_wheel_joint") {
                iws_motor1_->setVelocity(-1 * vel_commands_[i], 1.0);
            }
        }

        printf("position_param_position\n");
        for (size_t i = 0; i < 19; i++) {
            printf("%f ", pos_commands_[i]);
        }
        printf("\n");

        printf("position_param_vel\n");
        for (size_t i = 0; i < 19; i++) {
            printf("%f ", vel_commands_[i]);
        }
        printf("\n");

        return hardware_interface::return_type::OK;
    }

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface
)
