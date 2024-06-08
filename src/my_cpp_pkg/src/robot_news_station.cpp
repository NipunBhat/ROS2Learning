#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace rclcpp;

class RobotNewsStationNode : public Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), sRobotName("Nipun")
    {
        this->declare_parameter("robot_name", "Nipun");
        sRobotName = this->get_parameter("robot_name").as_string();
        
        _publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
         _timer = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station is Started ");
    }

protected:
    // NOP

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("This is") + sRobotName + std::string("from the Robot News Station");
        _publisher->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    std::string sRobotName; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   // Init Ros2
    auto node = std::make_shared<RobotNewsStationNode>();  // Create a node
    spin(node);                                        // call back on the node
    shutdown();                                        // shutdown comm
    return 0;
}