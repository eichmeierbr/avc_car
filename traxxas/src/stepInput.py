#!/usr/bin/env python

# Author: Braden Eichmeier

import rospy, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
 

def cmd_callback(data):
  global wheelbase
  #global ackermann_cmd_topic
  global esc_topic
  global steering_topic 
  global frame_id
  #global pub
  global esc_pub
  global steering_pub 

  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
 

  esc_msg = Float64()
  esc_msg.data = v
  steering_msg = Float64()
  steering_msg.data = steering
  
  esc_pub.publish(esc_msg)
  steering_pub.publish(steering_msg)

if __name__ == '__main__': 
  try:
     
    rospy.init_node('step_input')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    #ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    esc_topic = rospy.get_param('~esc_topic', '/cmd/set_motor_vel')
    steering_topic = rospy.get_param('~steering_topic', '/cmd/set_servo_pos')
    wheelbase = rospy.get_param('~wheelbase', 0.33)
    frame_id = rospy.get_param('~frame_id', 'odom')
   
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    #pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    esc_pub = rospy.Publisher(esc_topic, Float64, queue_size=1)
    steering_pub = rospy.Publisher(steering_topic, Float64, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

