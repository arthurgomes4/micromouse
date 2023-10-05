#! /usr/bin/env python

import rospy
import numpy as np
import math as m
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pkg_tf_micromouse.srv import move, moveResponse, moveTraj, moveTrajResponse
from tf.transformations import euler_from_quaternion

class walker():

    def __init__(self, name, vel_topic, pos_topic):

        # set some params
        self.bot_velocity = 0.4
        self.position_tolerance = 0.01
        self.position = []
        self.yaw = None
        self.linear_gain = 3.5
        self.angular_gain = 1

        # maze parameters
        wall_width = 0.012
        cell_number = 16
        corridor_width = 0.168

        # make divisor
        self.divisor = (wall_width + corridor_width)/2

        # transfrom from {c} to {o}
        disp = (self.divisor + cell_number*(corridor_width + wall_width))/2
        self.T_oc = np.array([[ 1,  0, -disp],
                              [ 0, -1, disp],
                              [ 0,  0,    1]])
        
        # init node stuff
        rospy.init_node(name)
        rospy.Subscriber(pos_topic, Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.controller_rate = rospy.Rate(30)
        self.service_container = rospy.Service('move_to_goal', move, self.handle_move_to_goal)
        self.traj_sc = rospy.Service('move_traj', moveTraj, self.handle_move_traj)

        # wait for odom to become available
        while len(self.position) == 0:
            pass   
        
        rospy.loginfo('node initialized. waiting for requests')
        rospy.spin()
        rospy.loginfo('node shutting down')

    def unit_vec(self,curr,next):
        vec = (next[0] - curr[0], next[1] - curr[1])
        mag = m.sqrt((vec[0]+1)**2 + (vec[1]+1)**2)
        return (vec[0]//mag, vec[1]//mag)

    def handle_move_traj(self, req):
        path = []
        for a in range(len(req.i)):
            path.append((req.i[a], req.j[a]))

        waypoints = []
        waypoints.append(path[0])
        dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

        for i in range(1,len(path)- 1):
            new_dir = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            if new_dir != dir:
                waypoints.append((path[i][0] - dir[0], path[i][1] - dir[1]))
                waypoints.append((path[i][0] + new_dir[0], path[i][1] + new_dir[1]))
                dir = new_dir[:]
        waypoints.append(path[-1])

        for i in range(len(waypoints)):
            # find the corresponding global goal
            x,y = self.mat2point(waypoints[i][0],waypoints[i][1])

            rospy.loginfo('moving to ' + str(x) + ',' + str(y))
            Ex,Ey = self.linear_error(x,y)
            vel_msg = Twist()

            while abs(Ex) > self.position_tolerance or abs(Ey) > self.position_tolerance:

                # calculate and publish velocity
                Vx = self.linear_gain*Ex
                Vy = self.linear_gain*Ey

                vel_msg.linear.x, vel_msg.linear.y = self.cap_velocity(Vx,Vy)
                vel_msg.angular.z = self.correct_orientation()
                self.vel_pub.publish(vel_msg)

                Ex,Ey = self.linear_error(x,y)

                

                self.controller_rate.sleep()

            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
            rospy.loginfo('reached!')
        
        resp = moveTrajResponse()
        resp.ack = True
        return resp

    def handle_move_to_goal(self, req):
        ret = self.move_to_goal(req.i, req.j)
        resp = moveResponse()
        resp.ack = ret
        return resp
    
    def mat2point(self,i,j):
        c_point = np.array([[j*self.divisor + self.divisor/2],[i*self.divisor + self.divisor/2],[1]]) 
        o_point = np.matmul(self.T_oc, c_point)
        return float(o_point[0]), float(o_point[1])

    def move_to_goal(self, i,j):

        # find the corresponding global goal
        x,y = self.mat2point(i,j)

        rospy.loginfo('moving to ' + str(x) + ',' + str(y))
        Ex,Ey = self.linear_error(x,y)
        vel_msg = Twist()
        # print(Ex,Ey)

        while abs(Ex) > self.position_tolerance or abs(Ey) > self.position_tolerance:

            # calculate and publish velocity
            Vx = self.linear_gain*Ex
            Vy = self.linear_gain*Ey

            vel_msg.linear.x, vel_msg.linear.y = self.cap_velocity(Vx,Vy)
            vel_msg.angular.z = self.correct_orientation()
            self.vel_pub.publish(vel_msg)

            Ex,Ey = self.linear_error(x,y)

            self.controller_rate.sleep()

        stop_msg = Twist()
        for i in range(400):
            self.vel_pub.publish(stop_msg)
        
        rospy.loginfo('reached!')
        return True
   
    def correct_orientation(self):
        return -self.angular_gain*self.yaw

    def cap_velocity(self,Vx,Vy):
        v_mag = m.sqrt(Vx**2 + Vy**2)
        if v_mag > self.bot_velocity:
            Vx = self.bot_velocity*Vx/v_mag
            Vy = self.bot_velocity*Vy/v_mag
        return Vx,Vy

    def linear_error(self, x,y):
        return x - self.position[0], y - self.position[1]

    def odom_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        (roll, pitch, self.yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

o = walker('walker', '/cmd_vel', '/odom')
