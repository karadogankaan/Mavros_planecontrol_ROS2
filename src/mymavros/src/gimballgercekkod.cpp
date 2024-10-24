
/*
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/MountControl.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::Publisher gimbal_pub = nh.advertise<mavros_msgs::MountControl>
            ("mavros/mount_control/command", 10);

    ros::Rate rate(20.0);

    // Wait for connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 10.0;

    // Arm the drone
    while (!current_state.armed) {
        arming_client.call(arm_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    // Take off
    takeoff_client.call(takeoff_cmd);
    ros::Duration(10.0).sleep();

    // Control the gimbal camera
    mavros_msgs::MountControl gimbal_cmd;
    gimbal_cmd.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;
    gimbal_cmd.pitch = 30.0;
    gimbal_cmd.yaw = 45.0;
    gimbal_pub.publish(gimbal_cmd);

    ROS_INFO("Drone is hovering and gimbal is set.");

    ros::spin();
    return 0;
}







*/






/*

//#include <ros/ros.h>   //gerek kalmayabilir
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/MountControl.h>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv, "gimbalcontrol");
    rclcpp::NodeHandle nh;  //nodehandle NH

    rclcpp::Publisher gimbal_pub = nh.advertise<mavros_msgs::MountControl>
            ("mavros/mount_control/command", 10);

    rclcpp::Rate rate(10.0);

    while (rclcpp::ok()) {
        mavros_msgs::MountControl gimbal_cmd;
        gimbal_cmd.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;
        gimbal_cmd.pitch = 30.0;  // Adjust pitch angle
        gimbal_cmd.yaw = 45.0;    // Adjust yaw angle

        gimbal_pub.publish(gimbal_cmd);

        ROS_INFO("Gimbal command sent: Pitch %f, Yaw %f", gimbal_cmd.pitch, gimbal_cmd.yaw);

        rate.sleep();
    }

    return 0;
}

*/


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














