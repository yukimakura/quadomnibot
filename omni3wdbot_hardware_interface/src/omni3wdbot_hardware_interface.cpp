#include "omni3wdbot_hardware_interface/omni3wdbot_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

namespace omni3wdbot_hardware_interface
{
  constexpr const char *Omni3wdBotHardware = "Omni3wdBotHardware";
  constexpr const char *const kExtraJointParameters[] = {
      "velocity_P_gain",
      "velocity_I_gain",
      "velocity_D_gain",
      "one_rotate_encoder_pulse"};

  CallbackReturn Omni3wdbotHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    try
    {
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "configure");
      if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      {
        return CallbackReturn::ERROR;
      }

      joints.resize(info_.joints.size(), Joint());
      joint_ids_.resize(info_.joints.size(), std::string(""));

      for (uint i = 0; i < info_.joints.size(); i++)
      {
        joint_ids_[i] = info_.joints[i].parameters.at("id");
        joints[i].state.position = std::numeric_limits<double>::quiet_NaN();
        joints[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
        joints[i].state.effort = std::numeric_limits<double>::quiet_NaN();
        joints[i].command.position = std::numeric_limits<double>::quiet_NaN();
        joints[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
        joints[i].command.effort = std::numeric_limits<double>::quiet_NaN();
        joints[i].prev_command.position = joints[i].command.position;
        joints[i].prev_command.velocity = joints[i].command.velocity;
        joints[i].prev_command.effort = joints[i].command.effort;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(Omni3wdBotHardware), "joint_id " << i << ": " << joint_ids_[i] << " jointname : " << info_.joints[i].name);
      }

      auto usb_port = info_.hardware_parameters.at("usb_port");
      auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
      stm32_control_rate_hz_ = std::stoi(info_.hardware_parameters.at("stm32_control_rate_hz"));

      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "usb_port: %s", usb_port.c_str());
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "baud_rate: %d", baud_rate);
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "stm32_control_rate_hz: %d", stm32_control_rate_hz_);
      // open port
      omnibot_comm = std::make_shared<OmniBotComm>(usb_port, baud_rate);
      omnibot_comm->ReadStart();

      set_control_mode(ControlMode::Velocity, true);
      set_joint_params();

      this->last_read_time_ = std::make_shared<rclcpp::Time>(rclcpp::Clock().now());

      return CallbackReturn::SUCCESS;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "on_init Error: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  std::vector<hardware_interface::StateInterface> Omni3wdbotHardwareInterface::export_state_interfaces()
  {
    try
    {
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "export_state_interfaces");
      std::vector<hardware_interface::StateInterface> state_interfaces;
      for (uint i = 0; i < info_.joints.size(); i++)
      {
        RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "export_state_interfaces  : %s/%s", info_.joints[i].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints[i].state.velocity));
      }

      return state_interfaces;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "export_state_interfaces Error: %s", e.what());
      return std::vector<hardware_interface::StateInterface>();
    }
  }

  std::vector<hardware_interface::CommandInterface> Omni3wdbotHardwareInterface::export_command_interfaces()
  {
    try
    {
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "export_command_interfaces");
      std::vector<hardware_interface::CommandInterface> command_interfaces;
      for (uint i = 0; i < info_.joints.size(); i++)
      {
        RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "export_command_interfaces  : %s/%s", info_.joints[i].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints[i].command.velocity));
      }

      return command_interfaces;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "export_command_interfaces Error: %s", e.what());
      return std::vector<hardware_interface::CommandInterface>();
    }
  }

  CallbackReturn Omni3wdbotHardwareInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    try
    {

      RCLCPP_DEBUG(rclcpp::get_logger(Omni3wdBotHardware), "start");
      for (uint i = 0; i < joints.size(); i++)
      {
        if (use_dummy_ && std::isnan(joints[i].state.position))
        {
          joints[i].state.position = 0.0;
          joints[i].state.velocity = 0.0;
          joints[i].state.effort = 0.0;
        }
      }
      read(rclcpp::Time{}, rclcpp::Duration(0, 0));
      reset_command();
      write(rclcpp::Time{}, rclcpp::Duration(0, 0));

      return CallbackReturn::SUCCESS;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "on_activate Error: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn Omni3wdbotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    try
    {
      RCLCPP_DEBUG(rclcpp::get_logger(Omni3wdBotHardware), "stop");
      omnibot_comm->WriteTarget(0, 0, 0);
      return CallbackReturn::SUCCESS;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "on_deactivate Error: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  return_type Omni3wdbotHardwareInterface::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    try
    {
      if (use_dummy_)
      {
        return return_type::OK;
      }
      auto curtime = rclcpp::Clock().now();
      auto deltaMilliTime = (curtime - *this->last_read_time_).nanoseconds() / 1000000;
      this->last_read_time_ = std::make_shared<rclcpp::Time>(curtime);

      std::vector<std::string> ids(info_.joints.size(), std::string(""));
      std::vector<int32_t> positions(info_.joints.size(), 0);
      std::vector<int32_t> velocities(info_.joints.size(), 0);
      std::vector<int32_t> currents(info_.joints.size(), 0);

      std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

      auto fb = omnibot_comm->GetFeedbackMergeAllQueue();
      RCLCPP_INFO_STREAM(rclcpp::get_logger(Omni3wdBotHardware), "delta milli time : " << deltaMilliTime << "latest fb f: " << fb.F << " rl: " << fb.RL << " rr: " << fb.RR);
      for (uint i = 0; i < joints.size(); i++)
      {

        if (joint_ids_[i] == "F")
          joints[i].state.velocity = ((double)fb.F / (double)this->latest_f_motor_ex_param_.One_Rotate_Encoder_Pulse) * 2. * M_PI * (1000. / deltaMilliTime);
        else if (joint_ids_[i] == "RR")
          joints[i].state.velocity = ((double)fb.RR / (double)this->latest_rr_motor_ex_param_.One_Rotate_Encoder_Pulse) * 2. * M_PI * (1000. / deltaMilliTime);
        else if (joint_ids_[i] == "RL")
          joints[i].state.velocity = ((double)fb.RL / (double)this->latest_rl_motor_ex_param_.One_Rotate_Encoder_Pulse) * 2. * M_PI * (1000. / deltaMilliTime);

        RCLCPP_INFO_STREAM(rclcpp::get_logger(Omni3wdBotHardware), "latest vel: " << joints[i].state.velocity);
      }

      return return_type::OK;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "read Error: %s", e.what());
      return return_type::ERROR;
    }
  }

  return_type Omni3wdbotHardwareInterface::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "write start");
    try
    {

      if (use_dummy_)
      {
        for (auto &joint : joints)
        {
          joint.prev_command.position = joint.command.position;
          joint.state.position = joint.command.position;
        }
        return return_type::OK;
      }

      // Velocity control
      // TODO: Test PWM mode
      if (std::any_of(
              joints.cbegin(), joints.cend(), [](auto j)
              { return j.command.velocity != j.prev_command.velocity; }))
      {
        set_control_mode(ControlMode::Velocity);
        if (mode_changed_)
        {
          set_joint_params();
        }
        set_joint_velocities();
        return return_type::OK;
      }

      // if all command values are unchanged, then remain in existing control mode and set corresponding command values
      switch (control_mode_)
      {
      case ControlMode::Velocity:
        set_joint_velocities();
        return return_type::OK;
        break;
      default: // effort, etc
        RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "Control mode not implemented");
        return return_type::ERROR;
        break;
      }
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "write Error: %s", e.what());
      return return_type::ERROR;
    }
  }

  return_type Omni3wdbotHardwareInterface::set_control_mode(const ControlMode &mode, const bool force_set)
  {
    try
    {

      mode_changed_ = false;

      if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity))
      {

        RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "Velocity control");
        if (control_mode_ != ControlMode::Velocity)
        {
          mode_changed_ = true;

          control_mode_ = ControlMode::Velocity;
        }

        return return_type::OK;
      }

      if (control_mode_ != ControlMode::Velocity)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger(Omni3wdBotHardware), "Only velocity control are implemented");
        return return_type::ERROR;
      }

      return return_type::OK;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "set_control_mode Error: %s", e.what());
      return return_type::ERROR;
    }
  }

  return_type Omni3wdbotHardwareInterface::reset_command()
  {
    try
    {
      for (uint i = 0; i < joints.size(); i++)
      {
        joints[i].command.position = joints[i].state.position;
        joints[i].command.velocity = 0.0;
        joints[i].command.effort = 0.0;
        joints[i].prev_command.position = joints[i].command.position;
        joints[i].prev_command.velocity = joints[i].command.velocity;
        joints[i].prev_command.effort = joints[i].command.effort;
      }

      return return_type::OK;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "reset_command Error: %s", e.what());
      return return_type::ERROR;
    }
  }

  CallbackReturn Omni3wdbotHardwareInterface::set_joint_velocities()
  {
    try
    {
      std::vector<int32_t> commands(info_.joints.size(), 0);
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "set_joint_velocities");

      int f_motor_target = 0;
      int rr_motor_target = 0;
      int rl_motor_target = 0;
      for (uint i = 0; i < joint_ids_.size(); i++)
      {
        auto cur_enc_pulse = 0;
        if (joint_ids_[i] == "F")
          f_motor_target = (int)((joints[i].command.velocity * this->latest_f_motor_ex_param_.One_Rotate_Encoder_Pulse / (2. * M_PI))) / stm32_control_rate_hz_;
        else if (joint_ids_[i] == "RR")
          rr_motor_target = (int)((joints[i].command.velocity * this->latest_rr_motor_ex_param_.One_Rotate_Encoder_Pulse / (2. * M_PI))) / stm32_control_rate_hz_;
        else if (joint_ids_[i] == "RL")
          rl_motor_target = (int)((joints[i].command.velocity * this->latest_rl_motor_ex_param_.One_Rotate_Encoder_Pulse / (2. * M_PI))) / stm32_control_rate_hz_;

        joints[i].prev_command.velocity = joints[i].command.velocity;
      }
      omnibot_comm->WriteTarget(f_motor_target, rr_motor_target, rl_motor_target);
      return CallbackReturn::SUCCESS;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "set_joint_velocities Error: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn Omni3wdbotHardwareInterface::set_joint_params()
  {
    try
    {
      RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "set_joint_params");

      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].parameters.at("id") == "F")
          set_gain_param(this->latest_f_motor_, this->latest_f_motor_ex_param_, info_.joints[i]);
        else if (info_.joints[i].parameters.at("id") == "RR")
          set_gain_param(this->latest_rr_motor_, this->latest_rr_motor_ex_param_, info_.joints[i]);
        else if (info_.joints[i].parameters.at("id") == "RL")
          set_gain_param(this->latest_rl_motor_, this->latest_rl_motor_ex_param_, info_.joints[i]);
      }

      if (!omnibot_comm->WriteGain(latest_f_motor_, latest_rr_motor_, latest_rl_motor_))
        return CallbackReturn::FAILURE;

      // 安全用にTargetに0を書き込んどく(起動時にモーターを止める)
      omnibot_comm->WriteTarget(0, 0, 0);

      return CallbackReturn::SUCCESS;
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "set_joint_params Error: %s", e.what());
    }
  }

  void Omni3wdbotHardwareInterface::set_gain_param(GainStruct &gain, JointExParam &ex_param, hardware_interface::ComponentInfo &joint)
  {
    try
    {
      for (auto paramName : kExtraJointParameters)
      {
        RCLCPP_INFO(rclcpp::get_logger(Omni3wdBotHardware), "joint %s  param load  %s : %s", joint.parameters.at("id").c_str(), paramName, joint.parameters.at(paramName).c_str());
      }

      gain.KP = std::stod(joint.parameters.at("velocity_P_gain"));
      gain.KI = std::stod(joint.parameters.at("velocity_I_gain"));
      gain.KD = std::stod(joint.parameters.at("velocity_D_gain"));
      ex_param.One_Rotate_Encoder_Pulse = std::stoi(joint.parameters.at("one_rotate_encoder_pulse"));
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omni3wdBotHardware), "set_gain_param Error: %s", e.what());
    }
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omni3wdbot_hardware_interface::Omni3wdbotHardwareInterface, hardware_interface::SystemInterface)