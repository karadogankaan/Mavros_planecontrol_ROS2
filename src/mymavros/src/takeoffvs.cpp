#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("drone_control")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, std::placeholders::_1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(1s, std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (!current_state_.armed)
        {
       
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_future = arming_client_->async_send_request(arm_request);

            if (arm_future.wait_for(5s) == std::future_status::ready)
            {
                auto arm_result = arm_future.get();
                if (arm_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "e arm");


                    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    takeoff_request->altitude = 10.0;
                    auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

                    if (takeoff_future.wait_for(10s) == std::future_status::ready)
                    {
                        auto takeoff_result = takeoff_future.get();
                        if (takeoff_result->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Takeof.");
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "f failed.");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Timeout");
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed ");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




