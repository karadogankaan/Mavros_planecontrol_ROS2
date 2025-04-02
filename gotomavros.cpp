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



























/*
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
        target.header.frame_id = "map"; // Use appropriate frame_id for your setup

        target.coordinate_frame = 3; // MAV_FRAME_GLOBAL_INT

        target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                           mavros_msgs::msg::PositionTarget::IGNORE_PY |
                           mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                           mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                           mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        target.position.x = 47.397742; // Example latitude
        target.position.y = 8.545594;  // Example longitude
        target.position.z = 10.0;      // Altitude
        
        local_pos_pub_->publish(target);

        RCLCPP_INFO(this->get_logger(), "Sent global position target: x=47.397742, y=8.545594, z=10.0");
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

*/





/*
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("uc")
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleFlight::flyToPosition, this));
    }

private:
    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map"; // Use the appropriate frame_id based on your setup
        pose.pose.position.x = 100.0; // Move 100 meters forward
        pose.pose.position.y = 100.0; // Move 100 meters right
        pose.pose.position.z = 10.0;  // Maintain altitude

        local_pos_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Sent target position: x=100, y=100, z=10");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/

/*
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("simple_flight"), position_sent_(false)
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&SimpleFlight::stateCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleFlight::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD")
        {
            RCLCPP_WARN(this->get_logger(), "Please switch to OFFBOARD mode.");
            return;
        }

        if (!position_sent_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map"; // or "local_origin"
            pose.pose.position.x = 100.0; // Move 100 meters forward in x
            pose.pose.position.y = 100.0; // Move 100 meters right in y
            pose.pose.position.z = 2.0;  // Maintain altitude

            local_pos_pub_->publish(pose);
            position_sent_ = true;

            RCLCPP_INFO(this->get_logger(), "Sent target position: x=100, y=100, z=2");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    bool position_sent_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node =#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("simple_flight")
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleFlight::flyToPosition, this));
    }

private:
    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map"; // Use the appropriate frame_id based on your setup
        pose.pose.position.x = 100.0; // Move 100 meters forward
        pose.pose.position.y = 100.0; // Move 100 meters right
        pose.pose.position.z = 10.0;  // Maintain altitude

        local_pos_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Sent target position: x=100, y=100, z=10");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/




/*
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("uc")
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleFlight::flyToPosition, this));
    }

private:
    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x += 100.0; // Move 100 meters forward
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 10.0; // Maintain current altitude

        local_pos_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Flying 100 meters forward.");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/


/*
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("uc")
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&SimpleFlight::flyToPosition, this));
    }

private:
    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 20.0; // Example target position
        pose.pose.position.y = 20.0;
        pose.pose.position.z = 10.0;

        local_pos_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Flying to position.");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



*/


/*

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("uc")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&SimpleFlight::stateCallback, this, std::placeholders::_1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(1s, std::bind(&SimpleFlight::controlLoop, this));
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
            if (arm_future.wait_for(5s) == std::future_status::ready && arm_future.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Drone armed.");
                requestTakeoff();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed.");
            }
        }
    }

    void requestTakeoff()
    {
        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->altitude = 10.0; // 10 meters altitude
        auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

        if (takeoff_future.wait_for(10s) == std::future_status::ready && takeoff_future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
            flyToPosition();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
        }
    }

    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 20.0; // Example target position
        pose.pose.position.y = 20.0;
        pose.pose.position.z = 10.0;

        local_pos_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Flying to position.");
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/



/*
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class SimpleFlight : public rclcpp::Node
{
public:
    SimpleFlight() : Node("uc")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&SimpleFlight::stateCallback, this, std::placeholders::_1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(1s, std::bind(&SimpleFlight::controlLoop, this));
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
            if (arm_future.wait_for(5s) == std::future_status::ready && arm_future.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Drone armed.");
                requestTakeoff();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed.");
            }
        }
    }

    void requestTakeoff()
    {
        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->altitude = 10.0; // 10 meters altitude
        auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

        if (takeoff_future.wait_for(10s) == std::future_status::ready && takeoff_future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
            flyToPosition();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
        }
    }

    void flyToPosition()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 10.0; // Example target position
        pose.pose.position.y = 10.0;
        pose.pose.position.z = 10.0;

        for (int i = 0; i < 100; ++i) // Send position for a few seconds
        {
            local_pos_pub_->publish(pose);
            rclcpp::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "Flight command sent.");
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleFlight>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/


/*
//ucma

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class TakeoffAndFly : public rclcpp::Node
{
public:
    TakeoffAndFly() : Node("takeoff_and_fly") //cmakelliste yazdıgında duzelt
    {

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TakeoffAndFly::stateCallback, this, std::placeholders::_1));
        
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        mission_client_ = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command_long");


        timer_ = this->create_wall_timer(1s, std::bind(&TakeoffAndFly::controlLoop, this));
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
                    RCLCPP_INFO(this->get_logger(), "Drone arm");

          
                    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    takeoff_request->altitude = 10.0;
                    auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

                    if (takeoff_future.wait_for(10s) == std::future_status::ready)
                    {
                        auto takeoff_result = takeoff_future.get();
                        if (takeoff_result->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Takeoff oldu");

                 
                            setOffboardMode();
                            setTargetPosition();
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "failed");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "timeout ");
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Fail");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout");
            }
        }
    }

    void setOffboardMode()
    {
        if (current_state_.mode != "OFFBOARD")
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_future = set_mode_client_->async_send_request(mode_request);

            if (mode_future.wait_for(5s) == std::future_status::ready)
            {
                auto mode_result = mode_future.get();
                if (mode_result->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "offboard mode");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout");
            }
        }
    }

    void setTargetPosition()
    {

        auto mission_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        mission_request->command = 16;  // MAV_CMD_NAV_WAYPOINT
        mission_request->param1 = 0;   // Acceptance radius (in meters)
        mission_request->param2 = 0;   // 0: Use the default altitude for the waypoint
        mission_request->param3 = 0;   // 0: No specific yaw angle
        mission_request->param4 = 0;   // 0: No specific heading angle
        mission_request->param5 = 47.397742; // Target latitude (example value)
        mission_request->param6 = 8.545594;  // Target longitude (example value)
        mission_request->param7 = 20;        // Target altitude (example value in meters)

        auto mission_future = mission_client_->async_send_request(mission_request);

        if (mission_future.wait_for(10s) == std::future_status::ready)
        {
            auto mission_result = mission_future.get();
            if (mission_result->success)
            {
                RCLCPP_INFO(this->get_logger(), "Target pos oldu");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Fail");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout");
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr mission_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TakeoffAndFly>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/

