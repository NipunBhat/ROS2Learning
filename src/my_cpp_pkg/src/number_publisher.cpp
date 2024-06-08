#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

// This is a cpp node that will always publish a constant number

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher():Node("number_publisher"),_iNumberToPublish(24),_iFrequencyToPublish(500)
    {
        this->declare_parameter("number_to_publish", 24); // The number to publish, if the param is not sent, default value ois still 24
        _iNumberToPublish = this->get_parameter("number_to_publish").as_int();

        this->declare_parameter("frequency_of_publishing", 500); // the frequency at which we will publish the number
        _iFrequencyToPublish = this->get_parameter("frequency_of_publishing").as_int(); 
        
        _publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10); // the topic name is number and the queue size of the topic is 10
        _timer = this->create_wall_timer(std::chrono::milliseconds(_iFrequencyToPublish), std::bind(&NumberPublisher::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Started node number_publisher");
    }

protected:
//NOP

private:
    void  publishNumber()
    {
        auto number = example_interfaces::msg::Int64();
        number.data = _iNumberToPublish;
        _publisher->publish(number);
        RCLCPP_INFO(this->get_logger(), "Published Number from number_publisher node to number topic");   
    }

    int64_t _iNumberToPublish;
    int _iFrequencyToPublish;
    bool _bDefaultOrNot;
    std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Int64>> _publisher; //create publisher shared pointer
    std::shared_ptr<rclcpp::TimerBase> _timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<NumberPublisher> node = std::make_shared<NumberPublisher>(); // The node instance is a shared pointer 
    rclcpp::spin(node);
    rclcpp::shutdown();
}