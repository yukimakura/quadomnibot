#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class imu_quat_normalizer : public rclcpp::Node
{
public:
    imu_quat_normalizer()
        : Node("imu_quat_normalizer"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/omnibot/imu/normalized", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/omnibot/imu", 10, std::bind(&imu_quat_normalizer::topic_callback, this, _1));

        // timer_ = this->create_wall_timer(1000ms, std::bind(&imu_quat_normalizer::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w); 

        q.normalize();
        msg->orientation.x = q.getX();
        msg->orientation.y = q.getY();
        msg->orientation.z = q.getZ();
        msg->orientation.w = q.getW();
        publisher_->publish(*msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    size_t count_;

    // void timer_callback()
    // {
    //     std::string fromname = this->get_parameter("from").as_string();
    //     std::string toname = this->get_parameter("to").as_string();

    //     RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    //     std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    //     this->set_parameters(all_new_parameters);
    // }

    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_quat_normalizer>());
    rclcpp::shutdown();
    return 0;
}