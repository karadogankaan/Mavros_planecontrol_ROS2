#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>

using namespace std::chrono_literals;

class MAVLinkFlight : public rclcpp::Node
{
public:
    MAVLinkFlight() : Node("uc")
    {
        local_pos_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/global", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MAVLinkFlight::flyToPosition, this));
    }

private:
    void flyToPosition()
    {
        mavros_msgs::msg::PositionTarget target;
        target.header.stamp = this->get_clock()->now();
        target.header.frame_id = "map";

        target.coordinate_frame = 3;

        target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                           mavros_msgs::msg::PositionTarget::IGNORE_PY |
                           mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                           mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

       
        target.position.x = 47.397742;
        target.position.y = 8.545594;
        target.position.z = 0.5;

        local_pos_pub_->publish(target);

        RCLCPP_INFO(this->get_logger(), " x=47.397742, y=8.545594, z=0.5");
    }

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MAVLinkFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
