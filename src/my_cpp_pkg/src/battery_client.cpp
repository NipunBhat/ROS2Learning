#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/led_battery_status.hpp"

class BatteryClient : public rclcpp::Node
{
public:
    BatteryClient():Node("battery"), fullyCharged(true)
    {
        _thread1 = std::thread(std::bind(&BatteryClient::_SendBatteryStatus, this));
    }

    ~BatteryClient()
    {
        //NOP
    }


protected:
//NOP
private:
   void _SendBatteryStatus()
   {
        auto client = this->create_client<my_robot_interfaces::srv::LedBatteryStatus>("set_led");
        int sleepTime = 4;
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to be up ...");
        }
        RCLCPP_INFO(this->get_logger(), "Server is up");
        auto request = std::make_shared<my_robot_interfaces::srv::LedBatteryStatus::Request>();
        while(true)
        {
            if(fullyCharged == true)
            {
                fullyCharged = false;
                sleepTime = 4;
                request->lednumber = 3;
                request->state = false;

            }
            else
            {
                fullyCharged = true;
                sleepTime = 6;
                request->lednumber = 3;
                request->state = true;
            }
            sleep(sleepTime);
            auto future = client->async_send_request(request);
            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Service Call Success");
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
   } 

    std::thread _thread1;
    bool fullyCharged;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
