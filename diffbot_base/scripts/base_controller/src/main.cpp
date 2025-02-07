#include <ros.h>

#include "base_controller.h"
#include "diffbot_base_config.h"
#include "h_bridge_controller/h_bridge_controller.h"

ros::NodeHandle nh;

using namespace diffbot;

HBridgeController motor_controller_right = HBridgeController(4, 5, 6);
HBridgeController motor_controller_left = HBridgeController(9, 8, 7);

BaseController<HBridgeController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);

void setup()
{
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}


void loop()
{
    static bool imu_is_initialized;

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
    if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh.logwarn("Emergency STOP");
        base_controller.eStop();
    }

    // This block publishes the IMU data based on a defined imu rate
    ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    {
        base_controller.read_and_publish_imu();
        base_controller.lastUpdateTime().imu = nh.now();
    }

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            //base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }
    // Call all the callbacks waiting to be called
    nh.spinOnce();
}