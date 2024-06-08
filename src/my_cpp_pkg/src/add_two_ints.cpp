#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode(): Node("add_two_ints")
    {
        thread1 = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));
    }

    ~AddTwoIntsClientNode()
    {
        //NOP
    }

    void callAddTwoIntsService(int a, int b)
    {
        //Create a client
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to be up ...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        
        try{
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b, response->sum);
        }catch(...){
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

protected:
//NOP
private:
    std::thread thread1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
