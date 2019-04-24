#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <std_msgs/Float64.h>
#include <string>
#include "topic_tools/MuxSelect.h"

#define J_ESC_POS 1	//index for joy.axes
#define J_SERVO_POS 2
static double max_speed = 0;    //for velocity in m/s
static double max_angle = 0;	//for steering angle in radians

ros::Publisher escPub;
ros::Subscriber joySub;
ros::Publisher servoPub;
std_msgs::Float64 escMsg;
//ros::ServiceClient client;
int i = 0;

void JoyCallback(const sensor_msgs::Joy joy) {
    
    std_msgs::Float64 servoMsg;

    escMsg.data = (max_speed*joy.axes[J_ESC_POS]);
    servoMsg.data = (-max_angle*joy.axes[J_SERVO_POS]);
    /*    topic_tools::MuxSelect srv;
        if (joy.buttons[5] == 1){
        if (i == 0){
            srv.request.topic = "/cmd/set_motor_vel";
            }else{
            srv.request.topic = "/joy/set_motor_vel";
        }
        client.call(srv);
        i = (i + 1) % 2;
        }
    */  
    escPub.publish(escMsg);
    servoPub.publish(servoMsg);
}

void LoadParams() {
    if (!ros::param::get("TraxxasParameters/max_forward_vel", max_speed))
    {
	ROS_WARN("Could not load parameter: max_forward_vel\n using default value: %f", max_speed);
    }
    if (!ros::param::get("TraxxasParameters/max_right_steering_angle", max_angle))
    {
	ROS_WARN("Could not load parameter: max_right_steering_angle\n using default value: %f", max_angle);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_traxxas");
    //ros::NodeHandle ph;	private handle?
    ros::NodeHandle nh;
  //  client = nh.serviceClient<topic_tools::MuxSelect>("/topic_tools/MuxSelect");

    LoadParams();

    joySub = nh.subscribe("joy", 10, JoyCallback);
    escPub = nh.advertise<std_msgs::Float64>("/joy/set_motor_vel", 10);
    servoPub = nh.advertise<std_msgs::Float64>("/joy/set_servo_pos", 10);

    while(ros::ok()) {
        ros::Rate responseRate(60);
        ros::spinOnce();
        responseRate.sleep();
       // escPub.publish(escMsg);		//auto reitterate of esc command is not safe, the vehicle will keep driving if the controller signal is lost!
    }
}
