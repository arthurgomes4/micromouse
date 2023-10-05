#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class vex:
    def callback(self,msg):
        scaler = 1
        x_msg = Twist()
        x_msg.linear.x = msg.linear.x*scaler
        x_msg.angular.z = msg.angular.z
        y_msg = Twist()
        y_msg.linear.x = msg.linear.y*scaler
        y_msg.angular.z = msg.angular.z
        self.x_vel_pub.publish(x_msg)
        self.y_vel_pub.publish(y_msg)

    def __init__(self, x_vel_topic,y_vel_topic,combined_topic):
        rospy.init_node('vex')
        vel_sub = rospy.Subscriber(combined_topic, Twist, self.callback)
        self.x_vel_pub = rospy.Publisher(x_vel_topic,Twist,queue_size=1)
        self.y_vel_pub = rospy.Publisher(y_vel_topic,Twist,queue_size=1)
        rospy.loginfo('waiting for '+ combined_topic)
        rospy.spin()

if __name__ == '__main__':
    vex("/cmd_vel_x","/cmd_vel_y","/cmd_vel")