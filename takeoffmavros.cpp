//takeoff

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using namespace std::chrono_literals;

class TakeoffDrone : public rclcpp::Node
{
public:
    TakeoffDrone() : Node("takeoff_drone")
    {
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");


        timer_ = this->create_wall_timer(1s, std::bind(&TakeoffDrone::controlLoop, this));
    }

private:
    void controlLoop()
    {

        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->altitude = 10.0;
        auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

        if (takeoff_future.wait_for(10s) == std::future_status::ready)
        {
            auto takeoff_result = takeoff_future.get();
            if (takeoff_result->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff oldu");

            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Takeoff fail");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout");
        }
    }

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TakeoffDrone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

