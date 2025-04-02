#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_long.hpp>

class GimbalControl : public rclcpp::Node
{
public:
    GimbalControl() : Node("GimbalControl")
    {
        client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "service");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = 205;
        request->param1 = pitch_;
        request->param2 = roll_;
        request->param3 = yaw_;
        request->param4 = 0;
        request->param5 = 0;
        request->param6 = 0;
        request->param7 = 2;

        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Gimbal ok");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Fail :(");
        }
    }

private:
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
    const double pitch_ = 0.0;
    const double roll_ = 0.0; 
    const double yaw_ = 90.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalControl>());
    rclcpp::shutdown();
    return 0;
}














