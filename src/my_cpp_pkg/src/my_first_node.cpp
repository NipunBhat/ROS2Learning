#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;

class MyNode : public Node
{
public:
    MyNode() : Node("cpp_test"), _counter(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        _timer = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

protected:
    // NOP

private:
    void timerCallback()
    {
        _counter++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", _counter);
    }

    TimerBase::SharedPtr _timer;
    int _counter;
};

int main(int argc, char **argv)
{
    init(argc, argv);   // Init Ros2
    auto node = std::make_shared<MyNode>();  // Create a node
    RCLCPP_INFO(node->get_logger(), "Hello Cpp Node"); // Made the node print
    spin(node);                                        // call back on the node
    shutdown();                                        // shutdown comm
    return 0;
}