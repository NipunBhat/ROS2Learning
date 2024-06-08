#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisher : public rclcpp::Node
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher")
    {
        _publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        _timer = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&HardwareStatusPublisher::_publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "Started hardware_status topic");
    }

    ~HardwareStatusPublisher()
    {
        //NOP
    }

protected:
//NOP

private:
    void _publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 130;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are too hot";
        _publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published hardware_status_information");
    }

    std::shared_ptr<rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}