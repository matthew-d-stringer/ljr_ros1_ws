/**
 * servo.cpp
 *
 * @author    Ethan Foss
 * @date       09/26/2023
 */


#include <string>
#include <iostream>
#include <cstdlib>

//rclib
#ifdef __cplusplus
extern "C"
{
#endif
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#ifdef __cplusplus
}
#endif

#include "ros/ros.h" // ROS

//Msgs
//#include <std_msgs/Int32.h>
#include <lander_junior_ros/ActuatorCommands.h>

//Services
#include <std_srvs/Trigger.h>


class Servo
{
public:

    // Define Statements
    const double SERVO_MAX_ANGLE = 20.0; // Max Servo Angle [deg]
    const double SERVO_SPEED = 250.0; // Servo Speed [deg/s]
    const int SERVO_FREQUENCY = 100; // Servo Frequency [Hz]
    const double SERVO_MAX_COMMAND_DIFF = 5.0; // Servo Max Command Difference [deg]
    const double SERVO_DEG_TO_VAL = 0.011111111; // Servo Degrees to Command Value [deg]
    //const int RC_SERVO_CH_MIN = 1; // Channel Limits
    //const int RC_SERVO_CH_MAX = 8; // Channel Limits

    Servo(ros::NodeHandle *nh)
    {
        // Initialize Servo Position
        servoPos1 = 0;
        servoPos2 = 0;

        // Set as disabled
        enabled = false;

        // Private Node Handle
        ros::NodeHandle private_nh("~");

        // Set Servo Channel
        if (!nh->getParam("servo_channel_1",servoChannel1))
        {
            std::cout << "Failed to get Servo Channel 1" << std::endl;
            exit(1);
        }
        if (!nh->getParam("servo_channel_2",servoChannel2))
        {
            std::cout << "Failed to get Servo Channel 2" << std::endl;
            exit(1);
        }

        // Create Subscriber
        std::string servo_sub_name;
        if (!private_nh.getParam("topics/actuator_commands",servo_sub_name))
        {
            std::cout << "Failed to get Servo Sub Name" << std::endl;
            exit(1);
        }
        actuator_cmd_sub_ = nh->subscribe(servo_sub_name,10,&Servo::subscriberCallback,this);

        // Create Service Listener
        std::string servo_enable_service_name;
        if (!private_nh.getParam("services/EnableServo",servo_enable_service_name))
        {
            std::cout << "Failed to get Servo Service Name" << std::endl;
            exit(1);
        }
        enable_servo_service = nh->advertiseService(servo_enable_service_name,&Servo::handleEnableService,this);

        // Create Service Listener
        std::string servo_disable_service_name;
        if (!private_nh.getParam("services/DisableServo",servo_disable_service_name))
        {
            std::cout << "Failed to get Servo Service Name" << std::endl;
            exit(1);
        }
        disable_servo_service = nh->advertiseService(servo_disable_service_name,&Servo::handleDisableService,this);


        // Create Timer for Servo
        timer_ = nh->createTimer(ros::Duration(1.0/SERVO_FREQUENCY),&Servo::timerCallback,this);

    }

    bool handleEnableService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = enableServos();
        res.message = "Servos Successfully Enabled";
        return res.success;
        
    }

    bool handleDisableService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = disableServos();
        res.message = "Servos Successfully Disabled";
        return res.success;
        
    }

private:

    int servoChannel1;
    double servoPos1;
    double servoCommand1;
    int servoChannel2;
    double servoPos2;
    double servoCommand2;
    bool enabled;

    ros::Timer timer_;
    ros::Subscriber actuator_cmd_sub_;
    ros::ServiceServer enable_servo_service;
    ros::ServiceServer disable_servo_service;

    bool enableServos()
    {
        if(enabled)
        {
            std::cout << "Servos already enabled!" << std::endl;
            return false;
        }

        // Check Battery Power
        if(rc_adc_init())
        {
            std::cout << "Failed to Initialize ADC for Servos" << std::endl;
            exit(1);
        }
        if(rc_adc_batt()<6.0)
        {
            std::cout << "Battery Power Insufficient to Drive Servos" << std::endl;
            exit(1);
        }
        rc_adc_cleanup();

        //initialize PRU
        if(rc_servo_init())
        {
            std::cout << "Failed to Intialize Servo" << std::endl;
            exit(1);
        }

        // Power Rail
        std::cout << "Powering Servo Rail" << std::endl;
        rc_servo_power_rail_en(0);

        // Sleep 2 Seconds
        ros::Duration(2).sleep();
        
        // Enable:
        enabled = true;
        return true;
    }

    bool disableServos()
    {
        if(!enabled)
        {
            std::cout << "Servos already disabled!" << std::endl;
            return false;
        }

        enabled = false;

        rc_servo_power_rail_en(0);
        rc_servo_cleanup();
        return true;

    }

    void timerCallback(const ros::TimerEvent& event)
    {
        if(enabled)
        {
            // // Servo 1
            // if(abs(servoPos1-servoCommand1) < SERVO_MAX_COMMAND_DIFF*SERVO_DEG_TO_VAL)
            // {
            //     servoPos1 = servoCommand1;
            // }
            // else if(servoPos1 > servoCommand1)
            // {
            //     servoPos1-=SERVO_SPEED/SERVO_FREQUENCY;
            // }
            // else
            // {
            //     servoPos1+=SERVO_SPEED/SERVO_FREQUENCY;
            // }

            servoPos1 = servoCommand1;
            if(servoPos1>SERVO_MAX_ANGLE)
            {
                servoPos1 = SERVO_MAX_ANGLE;
            }
            if(servoPos1<-SERVO_MAX_ANGLE)
            {
                servoPos1 = -SERVO_MAX_ANGLE;
            }

            // // Servo 2
            // if(abs(servoPos2-servoCommand2) < SERVO_MAX_COMMAND_DIFF*SERVO_DEG_TO_VAL)
            // {
            //     servoPos2 = servoCommand2;
            // }
            // else if(servoPos2 > servoCommand2)
            // {
            //     servoPos2-=SERVO_SPEED/SERVO_FREQUENCY;
            // }
            // else
            // {
            //     servoPos2+=SERVO_SPEED/SERVO_FREQUENCY;
            // }

            servoPos2 = servoCommand2;
            if(servoPos2>SERVO_MAX_ANGLE)
            {
                servoPos2 = SERVO_MAX_ANGLE;
            }
            if(servoPos2<-SERVO_MAX_ANGLE)
            {
                servoPos2 = -SERVO_MAX_ANGLE;
            }

            if(rc_servo_send_pulse_normalized(servoChannel1,servoPos1)==-1)
            {
                std::cout << "Failed to set Servo Position" << std::endl;
                exit(1);
            }

            if(rc_servo_send_pulse_normalized(servoChannel2,servoPos2)==-1)
            {
                std::cout << "Failed to set Servo Position" << std::endl;
                exit(1);
            }
        }
    }

    void subscriberCallback(const lander_junior_ros::ActuatorCommands& actuator_commands_msg)
    {
        servoCommand1 = actuator_commands_msg.alpha;
        servoCommand2 = actuator_commands_msg.beta;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"servo_node");
    ros::NodeHandle nh;
    Servo servo = Servo(&nh);
    ros::spin();
    return 0;
}