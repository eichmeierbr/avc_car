/*********************************************
This code was originally created for replacing
the VESC controller with the original Traxxas
ESC. This later version incorperates feedback
from the servo's potentiometer, and relays
that feedback to the system for accurate
odometry sensing

Author: Derek Workman
Email: derek.workman@aggiemail.usu.edu

This code was not working quite well so
there have been huge changes made

Editor: Sebastian Helzer
Email: sebastian+bots@helzer.eu
*********************************************/
/**[important notes]*************************
The relationship between the servo motor, and
steering angle is assumed to be near-linear,
and is calculated to be linear in this program.
Adjustments to this code may be necessary for
vehicles that do not have a linear servo-to-
steering angle relationship

!!!When writing motor_vel commands to this node,
make sure to take into account that a sudden reverse
in direction will cause incorrect velocity feedback

 As a safety feature, this node will cut the throttle
 if motor commands are not sent within a second
*********************************************/

#include <fcntl.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

using namespace std;

string serial_port_name = "ttyACM0";

int fd, c, res;
struct termios tio, oldTio;
int tty_fd;

static const uint8_t NEUTRAL = 90;

static double servo_angle_cmd = 0.0;
#define FULL_THROTTLE 135

// Functions
bool PublishMotorVel(ros::Publisher, std_msgs::Float64);
bool PublishServoPos(ros::Publisher, std_msgs::Float64);

template <class T>
T BoundCheck(T val, T ep1, T ep2);

static double motor_vel_cmd = 0.0;  // replaces val, and is used in PID_Loop()

static double servo_command_mul = FULL_THROTTLE / M_PI;  // default values
static double servo_command_offset = NEUTRAL;

static double forward_vel = 0.0;  // saves current forward velocity of vehicle

////[loaded parameters]////
static double diff_gear_ratio = 2.85;  // initialized with default values
static double wheel_radius =
    0.05;  // these are loaded with yaml file parameters on startup
static double max_right_steering_angle = -0.5235987756;
static double max_left_steering_angle = 0.5235987756;
static bool steering_angle_is_in_radians = true;
static double max_forward_vel = 1.0;
static double min_forward_vel = 0.0;
static double max_reverse_vel = -1.0;
static double min_reverse_vel = -0.0;

static int max_right_servo_feedback_val = 694;
static int max_left_servo_feedback_val = 309;

static double max_capable_forward_speed = 17.8816;
static double max_capable_reverse_speed = 17.8816;
static double max_accel = 1.1;  // not sure yet

static const uint8_t MOTOR_ID = 0;
static const uint8_t SERVO_ID = 1;

void writeMotor(uint8_t byte) {
  size_t size = 4;
  uint8_t buffer[size];
  buffer[0] = 'G';
  buffer[1] = 'O';
  buffer[2] = MOTOR_ID;
  buffer[3] = byte;
  //tio.c_cc[VMIN] = 4;
  try {
    write(fd, buffer, size);
  } catch (...) {
    ROS_WARN("in motor error block");
  }
}

void MotorCallback(const std_msgs::Float64 speed) {
  motor_vel_cmd = speed.data;
}

void writeServo(double val) {
  val = BoundCheck(val, max_left_steering_angle,
                   max_right_steering_angle);  // if val is out of bounds, then
                                               // it is set to the end point
  uint8_t byte = val * servo_command_mul + servo_command_offset;
  size_t size = 4;
  uint8_t buffer[size];
  buffer[0] = 'G';  // PID
  buffer[1] = 'O';
  buffer[2] = SERVO_ID;  // address
  buffer[3] = byte;      // data byte
  try {
    write(fd, buffer, size);
  } catch (...) {
    ROS_WARN("in servo error block");
  }
}

void ServoCallback(const std_msgs::Float64 servo) {
  ROS_DEBUG("In ServoCallback");
  servo_angle_cmd = -servo.data;
}

void LoadParams() {
  if (!ros::param::get("TraxxasParameters/diff_gear_ratio", diff_gear_ratio)) {
    ROS_WARN(
        "Could not load parameter: diff_gear_ratio\n using default value: %f",
        diff_gear_ratio);
  }
  if (!ros::param::get("TraxxasParameters/wheel_radius", wheel_radius)) {
    ROS_WARN("Could not load parameter: wheel_radius\n using default value: %f",
             wheel_radius);
  }
  if (!ros::param::get("TraxxasParameters/max_right_steering_angle",
                       max_right_steering_angle)) {
    ROS_WARN(
        "Could not load parameter: max_right_steering_angle\n using default "
        "value: %f",
        max_right_steering_angle);
  }
  if (!ros::param::get("TraxxasParameters/max_left_steering_angle",
                       max_left_steering_angle)) {
    ROS_WARN(
        "Could not load parameter: max_left_steering_angle\n using default "
        "value: %f",
        max_left_steering_angle);
  }
  if (!ros::param::get("TraxxasParameters/steering_angle_is_in_radians",
                       steering_angle_is_in_radians)) {
    ROS_WARN(
        "Could not load parameter: steering_angle_is_in_radians\n using "
        "default value: %s",
        steering_angle_is_in_radians ? "true" : "false");
  }
  if (!ros::param::get("TraxxasParameters/max_forward_vel", max_forward_vel)) {
    ROS_WARN(
        "Could not load parameter: max_forward_vel\n using default value: %f",
        max_forward_vel);
  }
  if (!ros::param::get("TraxxasParameters/min_forward_vel", min_forward_vel)) {
    ROS_WARN(
        "Could not load parameter: min_forward_vel\n using default value: %f",
        min_forward_vel);
  }
  if (!ros::param::get("TraxxasParameters/max_reverse_vel", max_reverse_vel)) {
    ROS_WARN(
        "Could not load parameter: max_reverse_vel\n using default value: %f",
        max_reverse_vel);
  }
  if (!ros::param::get("TraxxasParameters/min_reverse_vel", min_reverse_vel)) {
    ROS_WARN(
        "Could not load parameter: min_reverse_vel\n using default value: %f",
        min_reverse_vel);
  }
  if (!ros::param::get("TraxxasParameters/max_right_servo_feedback_val",
                       max_right_servo_feedback_val)) {
    ROS_WARN(
        "Could not load parameter: max_right_servo_feedback_val\n using "
        "default value: %d",
        max_right_servo_feedback_val);
  }
  if (!ros::param::get("TraxxasParameters/max_left_servo_feedback_val",
                       max_left_servo_feedback_val)) {
    ROS_WARN(
        "Could not load parameter: max_left_servo_feedback_val\n using default "
        "value: %d",
        max_left_servo_feedback_val);
  }
  if (!ros::param::get("TraxxasParameters/max_capable_forward_speed",
                       max_capable_forward_speed)) {
    ROS_WARN(
        "Could not load parameter: max_capable_forward_speed\n using default "
        "value: %f",
        max_capable_forward_speed);
  }
  if (!ros::param::get("TraxxasParameters/max_capable_reverse_speed",
                       max_capable_reverse_speed)) {
    ROS_WARN(
        "Could not load parameter: max_capable_reverse_speed\n using default "
        "value: %f",
        max_capable_reverse_speed);
  }
  if (!ros::param::get("TraxxasParameters/max_accel", max_accel)) {
    ROS_WARN("Could not load parameter: max_accel\n using default value: %f",
             max_accel);
  }
  if (!ros::param::get("PortConfig/traxxas_driver_port", serial_port_name)) {
    ROS_WARN(
        "Could not load parameter: traxxas_driver_port\n using default "
        "value: ");
  }
}

bool connectToArduino() {
  string csPortName = ("/dev/" + serial_port_name);
  const char *port = csPortName.c_str();

  try {
    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
      perror(port);
      exit(-1);
    }
    tcgetattr(fd, &oldTio);

    bzero(&tio, sizeof(tio));
    tio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;      // input mode is non-cononical, no echo . . .
    tio.c_cc[VTIME] = 1;  // inter-character time t = 0.05 * 0.1 seconds ?
    tio.c_cc[VMIN] =
        1;  
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &tio);
  } catch (...) {
    ROS_ERROR("Failed to connect to the Arduino UNO");
    ros::shutdown();
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "servo_esc_driver");
  ROS_WARN("Start to load params");
  LoadParams();
  ROS_WARN("Params loaded");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  if (!connectToArduino()) {
    return 0;
  }

  ROS_INFO("Done waiting for the Arduino");

  ros::Subscriber motor_sub = nh.subscribe("set_motor_vel", 10, MotorCallback);
  ros::Subscriber servo_sub = nh.subscribe("set_servo_pos", 10, ServoCallback);
  ros::Publisher motor_pub =
      nh.advertise<std_msgs::Float64>("get_motor_vel", 10);  // 50);
  ros::Publisher servo_pub =
      nh.advertise<std_msgs::Float64>("get_servo_pos", 10);  // 50);
  
  ros::Rate servo_pub_rate(30);

  while (ros::ok()) {
    uint8_t cmd = 90 + 4 * motor_vel_cmd;
    writeMotor(cmd);
    writeServo(servo_angle_cmd);
    ros::spinOnce();
    servo_pub_rate.sleep();
  }

  close(fd);
  tcsetattr(fd, TCSANOW, &oldTio);
  return 0;
}

template <class T>
T BoundCheck(T val, T ep1, T ep2) {
  T upper_bound;
  T lower_bound;

  if (ep1 > ep2)  // find upper and lower bounds
  {
    upper_bound = ep1;
    lower_bound = ep2;
  } else {
    upper_bound = ep2;
    lower_bound = ep1;
  }

  if (val > upper_bound)  // compare with radian end points
  {
    val = upper_bound;  // set to max, and not past that point
  } else if (val < lower_bound) {
    val = lower_bound;
  }
  return val;
}
