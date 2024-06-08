// This is a server that listens to the battery_client and publishes to a publisher
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/led_battery_status.hpp"
#include "my_robot_interfaces/msg/led_message_status.hpp"


class LedPanelServer : public rclcpp::Node
{
public:
    LedPanelServer(): Node("led_panel"), ledPanel({false, false, false})
    {
        this->declare_parameter("led_states", std::vector<bool>{false, false, false});
        ledPanel = this->get_parameter("led_states").as_bool_array();
        RCLCPP_INFO(this->get_logger(), "The values in the constructor are %d %d %d" , static_cast<int>(ledPanel[0]), 
        static_cast<int>(ledPanel[1]), static_cast<int>(ledPanel[2]));
        
        _server = this->create_service<my_robot_interfaces::srv::LedBatteryStatus>("set_led",
        std::bind(&LedPanelServer::_isLedOperationSuccess, this,
        std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Started the Led Server");

        _publisher = this->create_publisher<my_robot_interfaces::msg::LedMessageStatus>("led_panel_state", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LedPanelServer::_publishLedPanelStatus, this));

    }

    ~LedPanelServer()
    {
        //NOP
    }

protected:
//NOP
private:
    void _isLedOperationSuccess(std::shared_ptr<my_robot_interfaces::srv::LedBatteryStatus::Request> request,
                                std::shared_ptr<my_robot_interfaces::srv::LedBatteryStatus::Response> response)
    {
        ledPanel[request->lednumber - 1] = request->state; // Set the state of the led on that panel to on/off
        if(request->state == false) // When the led is turned off, battery is charged
        {
            RCLCPP_INFO(this->get_logger(), "THE BATTERY IS FULLY CHARGED");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "THE BATTERY IS NOT CHARGED");
        }
        response->success = true;
    }
    
    void _publishLedPanelStatus()
    {
        auto message = my_robot_interfaces::msg::LedMessageStatus();

        message.led[0] = ledPanel[0];
        message.led[1] = ledPanel[1];
        message.led[2] = ledPanel[2];

        _publisher->publish(message);

        RCLCPP_INFO(this->get_logger(), "%d %d %d" , static_cast<int>(ledPanel[0]), static_cast<int>(ledPanel[1]), static_cast<int>(ledPanel[2]));
    }

    std::shared_ptr<rclcpp::Service<my_robot_interfaces::srv::LedBatteryStatus>> _server; // The shared pointer that is the server.

    std::shared_ptr<rclcpp::Publisher<my_robot_interfaces::msg::LedMessageStatus>> _publisher; // Publisher
    std::shared_ptr<rclcpp::TimerBase> _timer;

    std::vector<bool> ledPanel; // Represents the LED Panel

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}