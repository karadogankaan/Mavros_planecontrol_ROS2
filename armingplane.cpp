
//ARM 

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std::chrono_literals;

class ArmDrone : public rclcpp::Node
{
public:
    ArmDrone() : Node("mymavros")
    {
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        timer_ = this->create_wall_timer(1s, std::bind(&ArmDrone::controlLoop, this));
    }

private:
    void controlLoop()
    {

        auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_request->value = true;
        auto arm_future = arming_client_->async_send_request(arm_request);

        if (arm_future.wait_for(5s) == std::future_status::ready)
        {
            auto arm_result = arm_future.get();
            if (arm_result->success)
            {
                RCLCPP_INFO(this->get_logger(), "Drone arm oldu");

            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "arm olmadi");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout");
        }
    }

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmDrone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

