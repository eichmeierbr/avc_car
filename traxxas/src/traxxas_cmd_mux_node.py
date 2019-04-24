#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

#freq_p1 = rospy.Rate(0.5)	#required frequency
T_p1 = 2	#2 seconds
time_p1 = 0.0
#last_time_p1 = 0.0
#flag_p1 = False

def set_time_p1(time):
    global time_p1
    time_p1 = time

def p1_servo_callback(msg):
    global servo_pub
    set_time_p1(rospy.Time.now().to_sec())
    servo_pub.publish(msg)

def p1_vel_callback(msg):
    global vel_pub
    set_time_p1(rospy.Time.now().to_sec())
    vel_pub.publish(msg)


def p2_servo_callback(msg):
    #global servo_pub
    if (rospy.Time.now().to_sec() - time_p1) > T_p1:
        msg.data*=7.5
        servo_pub.publish(msg)

def p2_vel_callback(msg):
    #global vel_pub
    if (rospy.Time.now().to_sec() - time_p1) > T_p1:
        msg.data*=7.5
        vel_pub.publish(msg)

def joyCallback(msg):
    global motor_topic, servo_topic, current_topic_motor, current_topic_servo
    # This is a control that if A is held, command is from joy, let go and then back to 
    # cmd.
    # if(msg.buttons[1]):
    #     motor_topic.unregister()
    #     servo_topic.unregister()
    #     motor_topic = rospy.Subscriber('/joy/set_servo_pos', Float64, p1_servo_callback)	#p1
    #     servo_topic = rospy.Subscriber('/joy/set_motor_vel', Float64, p1_vel_callback)
    # else:
    #     motor_topic.unregister()
    #     servo_topic.unregister()
    #     motor_topic = rospy.Subscriber('/cmd/set_servo_pos', Float64, p2_servo_callback)
    #     servo_topic = rospy.Subscriber('/cmd/set_motor_vel', Float64, p2_vel_callback)
    
    # This changes the controller for velocity whenever A is pressed.
    if(msg.buttons[1]):
        if(current_topic_motor!='/joy/set_servo_pos'):
            current_topic_motor = '/joy/set_servo_pos'
            current_topic_servo = '/joy/set_motor_vel'
        else:
            current_topic_motor = '/cmd/set_servo_pos'
            current_topic_servo = '/cmd/set_motor_vel'
        motor_topic.unregister()
        servo_topic.unregister()
        motor_topic = rospy.Subscriber(current_topic_motor, Float64, p1_servo_callback)	#p1
        servo_topic = rospy.Subscriber(current_topic_servo, Float64, p1_vel_callback)
        

if __name__=='__main__':
    try:
        rospy.init_node('traxxas_cmd_mux')
        current_topic_motor = '/cmd/set_servo_pos'
        current_topic_servo = '/cmd/set_motor_vel'
        motor_topic = rospy.Subscriber(current_topic_motor, Float64, p2_servo_callback)
        servo_topic = rospy.Subscriber(current_topic_servo, Float64, p2_vel_callback)
        rospy.Subscriber('/joy', Joy, joyCallback)
        servo_pub = rospy.Publisher('/set_servo_pos', Float64, queue_size=1)
        vel_pub = rospy.Publisher('/set_motor_vel', Float64, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
