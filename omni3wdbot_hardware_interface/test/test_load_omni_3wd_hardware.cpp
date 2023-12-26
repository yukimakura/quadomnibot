#include <gmock/gmock.h>
#include <memory>

#include <fstream>
#include <iostream>
#include <string>
#include <iterator>

#include <pluginlib/class_loader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

std::string getUrdf(std::string filePath)
{
  std::ifstream ifs(filePath);
  std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  return str;
}

TEST(TestLoadOmni3WDHardware, load_hardware_plugin)
{
  std::cout << "init loader" << std::endl;
  pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware_interface_loader("hardware_interface", "hardware_interface::SystemInterface");

  std::cout << "get plugin" << std::endl;
  auto hwif = hardware_interface_loader.createSharedInstance("omni3wdbot_hardware_interface/Omni3wdbotHardwareInterface");
  std::cout << "make hwinfo" << std::endl;
  auto hwinfo = hardware_interface::HardwareInfo();
  std::cout << "call pluginfunc" << std::endl;
  hwif->on_init(hwinfo);
  std::cout << "finish!" << std::endl;
}

TEST(TestLoadOmni3WDHardware, load_controller)
{
  try
  {

    std::cout << "start test" << std::endl;
    std::cout << "urdf path: " << ament_index_cpp::get_package_share_directory("omni3wdbot_hardware_interface") + "/test/omni3wdbot.urdf" << std::endl;

    std::string rawUrdf = getUrdf(ament_index_cpp::get_package_share_directory("omni3wdbot_hardware_interface") + "/test/omni3wdbot.urdf");
    rclcpp::init(0, nullptr);

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    controller_manager::ControllerManager cm(
        std::make_unique<hardware_interface::ResourceManager>(rawUrdf),
        executor, "test_controller_manager");

    ASSERT_NE(
        cm.load_controller("test_controller_manager", "omni_3wd_controller/Omni3WDController"),
        nullptr);

    sleep(5);
    std::cout << "update 1" << std::endl;
    cm.update(rclcpp::Time(), rclcpp::Duration(0, 0));
    sleep(1);
    std::cout << "update 2" << std::endl;
    cm.update(rclcpp::Time(), rclcpp::Duration(0, 0));
    sleep(1);
    std::cout << "update 3" << std::endl;
    cm.update(rclcpp::Time(), rclcpp::Duration(0, 0));

    sleep(5);

    ASSERT_FALSE(true);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  rclcpp::shutdown();
}
