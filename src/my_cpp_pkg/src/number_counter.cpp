#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

/*
This node does subscribes to the number topic
Every time it finds something published to the number topic, it will increase the counter and publish to the number_counter topic
*/

class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter():Node("number_counter"), iCounter(0)
    {
        RCLCPP_INFO(this->get_logger(), "Started number_counter node");

        //Create subscription to number topic
        _subscriber = this->create_subscription<example_interfaces::msg::Int64>("number", 10, 
        std::bind(&NumberCounter::_getNumber, this, std::placeholders::_1));

        //initialize publisher
        _publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count" , 10);
        
        //start the reset counter server
        _server = this->create_service<std_srvs::srv::SetBool>("reset_counter",
        std::bind(&NumberCounter::_resetCounterVariable, this, std::placeholders::_1,
        std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Started the Reset Counter Server..");

        //a timer that performs call backs to the _publishCounterVal method every 500 milliseconds
        _timer = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&NumberCounter::_publisherCounterVal, this));
    }
protected:
//NOP
private:
    void _getNumber(const std::shared_ptr<example_interfaces::msg::Int64> number)
    {
        RCLCPP_INFO(this->get_logger(), "Got number %d from number topic", int(number->data));
        iCounter++;
    }

    void _publisherCounterVal()
    {
        auto counterVal = example_interfaces::msg::Int64();
        counterVal.data = iCounter;
        _publisher->publish(counterVal);
    }

    void _resetCounterVariable(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data == true)
        {
            int oldCounter = iCounter;
            iCounter = 0;
            response->success = true;
            response->message = "RESET COUNTER FROM " + std::to_string(oldCounter) + " TO " + std::to_string(iCounter);
            RCLCPP_INFO(this->get_logger(), "The counter is reset from %d to %ld", oldCounter, iCounter);
        }
        else
        {
            response->success = false;
            response->message = "The counter was not reset " + std::to_string(iCounter);
            RCLCPP_WARN(this->get_logger(), "Did not reset the counter, the value is %ld", iCounter);
        }
    }

    std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Int64>>_publisher;
    std::shared_ptr<rclcpp::Subscription<example_interfaces::msg::Int64>> _subscriber;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> _server;
    int64_t iCounter;
    std::shared_ptr<rclcpp::TimerBase> _timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
