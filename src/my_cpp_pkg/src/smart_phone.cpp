#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace rclcpp;

class SmartPhone : public Node
{
public:
    SmartPhone() : Node("smart_phone")
    {
        _subscriber = this->create_subscription<example_interfaces::msg::String>("robot_news", 10, 
        std::bind(&SmartPhone::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smart Phone has been started");
    }

protected:
    // NOP

private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr _subscriber;
};

int main(int argc, char **argv)
{
    init(argc, argv);   // Init Ros2
    auto node = std::make_shared<SmartPhone>();  // Create a node
    RCLCPP_INFO(node->get_logger(), "Hello Cpp Node"); // Made the node print
    spin(node);                                        // call back on the node
    shutdown();                                        // shutdown comm
    return 0;
}