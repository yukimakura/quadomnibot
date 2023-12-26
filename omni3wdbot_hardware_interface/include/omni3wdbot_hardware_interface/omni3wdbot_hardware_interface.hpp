#ifndef OMNI3WDBOT_HARDWARE_INTERFACE__OMNI3WDBOT_HARDWARE_INTERFACE_HPP_
#define OMNI3WDBOT_HARDWARE_INTERFACE__OMNI3WDBOT_HARDWARE_INTERFACE_HPP_

#include <omni3wdbot_hardware_interface/visibility_control.h>
#include <omni3wdbot_hardware_interface/omnibot_comm.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>

#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace omni3wdbot_hardware_interface
{
    struct JointValue
    {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
    };

    struct Joint
    {
        JointValue state{};
        JointValue command{};
        JointValue prev_command{};
    };

    struct JointExParam
    {
        int One_Rotate_Encoder_Pulse = 0;
    };

    enum class ControlMode
    {
        Position,
        Velocity,
        Torque,
        Currrent,
        ExtendedPosition,
        MultiTurn,
        CurrentBasedPosition,
        PWM,
    };

    class Omni3wdbotHardwareInterface
        : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Omni3wdbotHardwareInterface)

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        OMNI3WDBOT_HARDWARE_INTERFACE_PUBLIC
        return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        return_type set_control_mode(const ControlMode &mode, const bool force_set = false);

        return_type reset_command();

        CallbackReturn set_joint_velocities();
        CallbackReturn set_joint_params();
        void set_gain_param(GainStruct &gain, JointExParam &ex_param, hardware_interface::ComponentInfo &joint);

        std::shared_ptr<OmniBotComm> omnibot_comm;

        std::vector<double> hw_velocities;

        std::vector<Joint> joints;
        std::vector<std::string> joint_ids_;
        ControlMode control_mode_{ControlMode::Velocity};
        bool mode_changed_{false};
        bool use_dummy_{false};

        int stm32_control_rate_hz_;
        std::shared_ptr<rclcpp::Time> last_read_time_;

        GainStruct latest_f_motor_ = {0., 0., 0.};
        JointExParam latest_f_motor_ex_param_ = {0};
        GainStruct latest_rr_motor_ = {0., 0., 0.};
        JointExParam latest_rr_motor_ex_param_ = {0};
        GainStruct latest_rl_motor_ = {0., 0., 0.};
        JointExParam latest_rl_motor_ex_param_ = {0};
    };
}

#endif // OMNI3WDBOT_HARDWARE_INTERFACE__OMNI3WDBOT_HARDWARE_INTERFACE_HPP_