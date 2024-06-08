#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class ResetNumberCounter : public rclcpp::Node
{
public:
    ResetNumberCounter():Node("reset_number_count"), _reset(true)
    {
        _reset_thread = std::thread(std::bind(&ResetNumberCounter::resetCounter, this, true));
    }

    ~ResetNumberCounter()
    {
        _reset_thread.join();
    }

protected:
//NOP

private:
    //TODO -> make this menu driven instead of just run and resetting
    void resetCounter(bool data)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = data;

        _client = this->create_client<std_srvs::srv::SetBool>("reset_counter");

        while(_client->wait_for_service(std::chrono::seconds(1)) == false) // 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to start ...");
        }
        std::cout<<"Client is up"<<std::endl;
        request->data = true;
        auto future = _client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service did not fail");
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Service failed");
        }
    }

    std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> _client;
    std::thread _reset_thread;
    bool _reset;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::cout<<"After Init"<<std::endl;
    auto node = std::make_shared<ResetNumberCounter>();
    std::cout<<"After Node Declaration"<<std::endl;
    rclcpp::spin(node);
    std::cout<<"After Spin Node"<<std::endl;
    rclcpp::shutdown();
    std::cout<<"After shutdown"<<std::endl;
    return 0;
}