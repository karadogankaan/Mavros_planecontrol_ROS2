/*#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/debug_value.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("debug_value_publisher");

    auto publisher = node->create_publisher<mavros_msgs::msg::DebugValue>(
        "/mavros/debug_value/send", rclcpp::QoS(10));

    mavros_msgs::msg::DebugValue msg;
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    msg.header.frame_id = "frame";
    msg.name = "oguzhan";
    msg.value_int = 99;
    msg.value_float = 10.0;

    while (true)
    {
        publisher->publish(msg);
        sleep(1);
    }
    

    rclcpp::shutdown();

    return 0;
}



#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
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
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(mode_request);
        }

        if (!current_state_.armed)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            arming_client_->async_send_request(arm_request);
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
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





#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(mode_request);
            offboard_requested_ = true;
        }

        if (!current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            arming_client_->async_send_request(arm_request);
            arm_requested_ = true;
        }

        if (current_state_.armed && !takeoff_requested_)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Set desired takeoff altitude
            takeoff_client_->async_send_request(takeoff_request);
            takeoff_requested_ = true;
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}









#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(mode_request);
            offboard_requested_ = true;
        }

        if (!current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            arming_client_->async_send_request(arm_request);
            arm_requested_ = true;
            last_arm_time_ = this->now();
        }

        if (current_state_.armed && !takeoff_requested_)
        {
            
            if ((this->now() - last_arm_time_).seconds() > 3.0)
            {
                auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                takeoff_request->altitude = 10.0;
                takeoff_client_->async_send_request(takeoff_request);
                takeoff_requested_ = true;
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (!current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            arming_client_->async_send_request(arm_request);
            arm_requested_ = true;
            last_arm_time_ = this->now();
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Set desired takeoff altitude
            takeoff_client_->async_send_request(takeoff_request);
            takeoff_requested_ = true;
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
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
            RCLCPP_WARN(this->get_logger(), "Not in OFFBOARD mode, please switch to OFFBOARD.");
            return;
        }

        if (!current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto result = arming_client_->async_send_request(arm_request);

            if (result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Arming successful.");
                arm_requested_ = true;
                last_arm_time_ = this->now();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed.");
            }
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Adjust based on current altitude
            auto result = takeoff_client_->async_send_request(takeoff_request);

            if (result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                takeoff_requested_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_result = set_mode_client_->async_send_request(mode_request);

            if (mode_result.get()->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
                offboard_requested_ = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode.");
                return; // Return early to avoid further actions if mode isn't set
            }
        }

        if (current_state_.mode == "OFFBOARD" && !current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_result = arming_client_->async_send_request(arm_request);

            if (arm_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Arming successful.");
                arm_requested_ = true;
                last_arm_time_ = this->now();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed.");
                return; // Return early if arming fails
            }
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Adjust based on current altitude
            auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);

            if (takeoff_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                takeoff_requested_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_result = set_mode_client_->async_send_request(mode_request);

            if (mode_result.get()->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
                offboard_requested_ = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode.");
                return;
            }
        }

        if (current_state_.mode == "OFFBOARD" && !current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_result = arming_client_->async_send_request(arm_request);

            if (arm_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Arming successful.");
                arm_requested_ = true;
                last_arm_time_ = this->now();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed.");
                return;
            }
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Adjust based on current altitude
            auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);

            if (takeoff_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                takeoff_requested_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_future = set_mode_client_->async_send_request(mode_request);

            if (mode_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            {
                auto mode_result = mode_future.get();
                if (mode_result->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
                    offboard_requested_ = true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode.");
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for OFFBOARD mode response.");
                return;
            }
        }

        if (current_state_.mode == "OFFBOARD" && !current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_future = arming_client_->async_send_request(arm_request);

            if (arm_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            {
                auto arm_result = arm_future.get();
                if (arm_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Arming successful.");
                    arm_requested_ = true;
                    last_arm_time_ = this->now();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Arming failed.");
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for arming response.");
                return;
            }
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Adjust based on current altitude
            auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

            if (takeoff_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            {
                auto takeoff_result = takeoff_future.get();
                if (takeoff_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                    takeoff_requested_ = true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff response.");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using std::placeholders::_1;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControl::stateCallback, this, _1));

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DroneControl::controlLoop, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void controlLoop()
    {
        if (current_state_.mode != "OFFBOARD" && !offboard_requested_)
        {
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_future = set_mode_client_->async_send_request(mode_request);

            if (mode_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
            {
                auto mode_result = mode_future.get();
                if (mode_result->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
                    offboard_requested_ = true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for OFFBOARD mode response.");
                // Optionally retry or take additional actions
            }
        }

        if (current_state_.mode == "OFFBOARD" && !current_state_.armed && !arm_requested_)
        {
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_future = arming_client_->async_send_request(arm_request);

            if (arm_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
            {
                auto arm_result = arm_future.get();
                if (arm_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Arming successful.");
                    arm_requested_ = true;
                    last_arm_time_ = this->now();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Arming failed.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for arming response.");
                // Optionally retry or take additional actions
            }
        }

        if (arm_requested_ && !takeoff_requested_ &&
            (this->now() - last_arm_time_).seconds() > 3.0)
        {
            auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 10.0;  // Adjust based on current altitude
            auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

            if (takeoff_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
            {
                auto takeoff_result = takeoff_future.get();
                if (takeoff_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                    takeoff_requested_ = true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff response.");
                // Optionally retry or take additional actions
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_arm_time_;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    bool takeoff_requested_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
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
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        timer_ = this->create_wall_timer(1s, std::bind(&DroneControl::controlLoop, this));
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
            RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
            auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_request->custom_mode = "OFFBOARD";
            auto mode_future = set_mode_client_->async_send_request(mode_request);

            if (mode_future.wait_for(5s) == std::future_status::ready)
            {
                auto mode_result = mode_future.get();
                if (mode_result->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for OFFBOARD mode response.");
            }
        }
        else if (!current_state_.armed)
        {
            RCLCPP_INFO(this->get_logger(), "Arming...");
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_future = arming_client_->async_send_request(arm_request);

            if (arm_future.wait_for(5s) == std::future_status::ready)
            {
                auto arm_result = arm_future.get();
                if (arm_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Arming successful.");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to arm.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for arming response.");
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
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

*/


/*

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("mymavros")
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
                    RCLCPP_INFO(this->get_logger(), "Arm ");

                    // Request takeoff after arming
                    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    takeoff_request->altitude = 10.0;  // 10 meters altitude
                    auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

                    if (takeoff_future.wait_for(10s) == std::future_status::ready)
                    {
                        auto takeoff_result = takeoff_future.get();
                        if (takeoff_result->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Takeoff");
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "fail");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "o");
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Fail");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout yedi");
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



#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>


using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ( "mavros_msgs/OverrideRCIn", 1, true);


    ros::Rate rate(20.0);

   
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

  
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    mavros_msgs::OverrideRCIn rc_msg;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        for(int i = 0; i < 8; i++){
            rc_msg.channels[i] = 0;
        }
        rc_msg.channels[2] = 1300;

        rc_pub.publish(rc_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}```
*/



/* iyi iyi iyi iyi
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
        // Check if the drone is armed
        if (!current_state_.armed)
        {
            // Send the arm request
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_future = arming_client_->async_send_request(arm_request);

            if (arm_future.wait_for(5s) == std::future_status::ready)
            {
                auto arm_result = arm_future.get();
                if (arm_result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");

                    // Request takeoff after arming
                    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    takeoff_request->altitude = 10.0;  // 10 meters altitude
                    auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

                    if (takeoff_future.wait_for(10s) == std::future_status::ready)
                    {
                        auto takeoff_result = takeoff_future.get();
                        if (takeoff_result->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
                            // Optionally stop the loop or handle further operations
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Takeoff failed.");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff response.");
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for arming response.");
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
*/

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




