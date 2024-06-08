#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    { 
        pServer = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this,
            std::placeholders::_1, std::placeholders::_2) 
        );

        RCLCPP_INFO(this->get_logger(), "Service server has been started");
    }

    ~AddTwoIntsServerNode()
    {
        //NOP
    }

private:
    std::shared_ptr<rclcpp::Service<example_interfaces::srv::AddTwoInts>> pServer;

    void callbackAddTwoInts(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}