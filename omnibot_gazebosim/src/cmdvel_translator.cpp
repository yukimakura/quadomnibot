#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class cmdvel_translator : public rclcpp::Node
{
public:
    cmdvel_translator()
        : Node("cmdvel_translator"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omnibot_base_controller/cmd_vel_unstamped", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&cmdvel_translator::topic_callback, this, _1));

        // timer_ = this->create_wall_timer(1000ms, std::bind(&cmdvel_translator::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        publisher_->publish(*msg);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
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
    rclcpp::spin(std::make_shared<cmdvel_translator>());
    rclcpp::shutdown();
    return 0;
}