#include "ros/ros.h"
#include "std_msgs/String.h"

#include <diffbot_msgs/AngularVelocitiesStamped.h>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

ros::Publisher pub_received_vel_for_plot;

void measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg_joint_states) {
    // declare vel
    double vel;
    vel = msg_joint_states->velocity[0];
    // publish vel 
    std_msgs::Float32 vel_msg;
    vel_msg.data = vel;
    pub_received_vel_for_plot.publish(vel_msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_pid");

    ros::NodeHandle n;

    double vel;
    n.getParam("/vel", vel);

    ros::Publisher pub_wheel_cmd_velocities = n.advertise<diffbot_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);
    ros::Publisher pub_cmd_vel_for_plot = n.advertise<std_msgs::Float32>("cmd_vel_plot", 10);
    pub_received_vel_for_plot = n.advertise<std_msgs::Float32>("received_vel_plot", 10);

    auto sub_measured_joint_states = n.subscribe("measured_joint_states", 10, measuredJointStatesCallback);

    ros::Rate loop_rate(10);

    double cmd_vel = 0;

    double step = 0.4;

    while (ros::ok()) {

        diffbot_msgs::WheelsCmdStamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();


        // command wheel velocities between -vel and vel
        
        wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(cmd_vel);
        wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(cmd_vel);


        cmd_vel += step;
        if (cmd_vel > vel || cmd_vel < -vel) {
            step = -step;
        }

        pub_wheel_cmd_velocities.publish(wheel_cmd_msg);

        std_msgs::Float32 vel_msg;
        vel_msg.data = cmd_vel;
        pub_cmd_vel_for_plot.publish(vel_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}