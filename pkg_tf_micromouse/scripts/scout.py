#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion
import heapq as hp
import math as m
from pkg_tf_micromouse.srv import move, moveRequest, moveTrajRequest, moveTraj
import time

class scout:

    def __init__(self, laser_topic, pos_topic):

        self.pos_topic = pos_topic
        # load maze paraeters
        wall_width = 0.012
        self.cell_number = 16
        corridor_width = 0.168

        # divisor for cell identification
        self.divisor = (wall_width + corridor_width)/2

        # transfrom from {c} to {o}
        disp = (self.divisor + self.cell_number*(corridor_width + wall_width))/2
        self.T_co = np.array([[ 1,  0, disp],
                              [ 0, -1, disp],
                              [ 0,  0,    1]])

        # create occupancy matrix
        self.occ_matrix = np.ones((self.cell_number*2+1,self.cell_number*2+1), dtype=np.int_)
        self.occ_matrix[0:self.cell_number*2+1,                      0] = 0
        self.occ_matrix[                     0, 0:self.cell_number*2+1] = 0
        self.occ_matrix[0:self.cell_number*2+1,     self.cell_number*2] = 0
        self.occ_matrix[    self.cell_number*2, 0:self.cell_number*2+1] = 0

        self.MAZE_RESET = False

        # occupancy matrix update
        self.occ_matrix_update = 0
        self.occ_matrix_update_threshold = 3
        self.maze_update_status = False
        
        # laser_callback lock
        self.laser_callback_lock = False

        # initialize node
        rospy.init_node('scout')
        rospy.Subscriber(laser_topic,LaserScan,self.laser_callback2)
        self.tf_lis = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.service_container = rospy.ServiceProxy('move_to_goal', move)
        self.service_container_2 = rospy.ServiceProxy('move_traj', moveTraj)
        rospy.loginfo('node setup complete')

        # amount of time to delay before starting run (seconds)
        time.sleep(2)
        
        start = time.time()
        self.search_maze()

        while self.MAZE_RESET:
            self.MAZE_RESET = False
            self.search_maze()
        
        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, self.bot_pos, (16,16))
        while len(path) == 0:
            self.reset_occ_matrix()
            while self.MAZE_RESET:
                self.MAZE_RESET = False
                self.search_maze()
            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, (16,16))

        search_time = time.time() - start
        start = time.time()

        rospy.loginfo('final run')
        self.request_traj(path)

        run_time = time.time() - start
        rospy.loginfo('runtime = '+str(run_time)+' search time = '+str(search_time)+' total = '+str(run_time + search_time/30))

    def search_maze(self):
        #====================================================================
        rospy.loginfo('moving to goal')

        self.bot_pos = self.locate_bot(self.pos_topic)
    
        self.wait_for_occ_mat_update()

        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, self.bot_pos, (16,16))

        while len(path) == 0:
            self.reset_occ_matrix()

            self.occ_matrix_update = 0
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, (16,16))
            
        node,path = self.find_dir_change(path)
        self.request_move(node[0],node[1])
        #====================================================================

        while not rospy.is_shutdown():
            
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)

            if self.check_path(maze, path):
                path = self.find_path(maze, node, (16,16))
            
            if len(path) == 0:
                temp_bot_pos = self.locate_bot(self.pos_topic)
                if temp_bot_pos != (16,16):
                    while len(path) == 0:
                        self.reset_occ_matrix()

                        self.occ_matrix_update = 0
                        self.wait_for_occ_mat_update()

                        maze = np.copy(self.occ_matrix)
                        path = self.find_path(maze, temp_bot_pos, (16,16))

                else:
                    break

            node,path = self.find_dir_change(path)
            self.request_move(node[0],node[1])

        #===================================================================
        rospy.loginfo('moving back home')

        self.wait_for_occ_mat_update()

        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, (16,16), self.bot_pos)

        while len(path)==0:
            self.reset_occ_matrix()

            self.occ_matrix_update = 0
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, (16,16), self.bot_pos)
            
        node,path = self.find_dir_change(path)
        self.request_move(node[0],node[1])
        #====================================================================

        while not rospy.is_shutdown():
            
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)

            if self.check_path(maze, path):
                path = self.find_path(maze, node, self.bot_pos)
            
            if len(path)==0:
                temp_bot_pos = self.locate_bot(self.pos_topic)
                if temp_bot_pos != self.bot_pos:
                    while len(path)==0:
                        self.reset_occ_matrix()

                        self.occ_matrix_update = 0
                        self.wait_for_occ_mat_update()

                        maze = np.copy(self.occ_matrix)
                        path = self.find_path(maze, temp_bot_pos, self.bot_pos)
                else:
                    break

            node,path = self.find_dir_change(path)
            self.request_move(node[0],node[1])

    def reset_occ_matrix(self):

        self.occ_matrix = np.ones((self.cell_number*2+1,self.cell_number*2+1), dtype=np.int_)
        self.occ_matrix[0:self.cell_number*2+1,                 0] = 0
        self.occ_matrix[                0, 0:self.cell_number*2+1] = 0
        self.occ_matrix[0:self.cell_number*2+1,     self.cell_number*2] = 0
        self.occ_matrix[    self.cell_number*2, 0:self.cell_number*2+1] = 0

        rospy.logwarn('MAZE RESET')
        #set warning flag
        self.MAZE_RESET = True

    def wait_for_occ_mat_update(self):
        while self.occ_matrix_update < self.occ_matrix_update_threshold:
            pass
        return

    def locate_bot(self, pos_topic):
        odom_msg = rospy.wait_for_message(pos_topic, Odometry)
        o_point = [[odom_msg.pose.pose.position.x], [odom_msg.pose.pose.position.y],[1]]
        c_point = np.matmul(self.T_co, o_point)
        element = c_point//self.divisor
        return (int(element[1]), int(element[0]))

    def request_move(self,i,j):
        rospy.wait_for_service('move_to_goal')
        try:
            self.laser_callback_lock = True
            self.occ_matrix_update = 0
            while self.maze_update_status:
                pass
            req = moveRequest()
            req.i = i
            req.j = j
            resp = self.service_container(req)
            self.laser_callback_lock = False
            
            return resp.ack
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def request_traj(self,path):
        rospy.wait_for_service('move_traj')
        try:
            req = moveTrajRequest()
            for node in path:
                req.i.append(node[0])
                req.j.append(node[1])
            resp = self.service_container_2(req)
            return resp.ack
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def find_dir_change(self,path):
        dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

        for i in range(1,len(path)-1):
            if (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) != dir:
                return path[i], path[i:]

        return path[-1], list()

    def check_path(self, maze, path):
        # path is a list of tuples
        for node in path:
            if not maze[node[0],node[1]]:
                return 1
        return 0

    def find_path(self, maze, start, goal):
        maze_dims = (maze.shape[0], maze.shape[1])

        open_nodes = []
        closed_nodes = np.zeros(maze_dims, dtype=np.bool)
        past_cost_matrix = np.full(maze_dims, np.Inf)
        parent_matrix = np.full((maze_dims[0],maze_dims[1],2), None)

        # initializing OPEN with start
        # heuristic added cost, node_pos
        hp.heappush(open_nodes, (0,start))
        past_cost_matrix[start[0], start[1]] = 0

        while len(open_nodes) > 0:

            current = hp.heappop(open_nodes)[1]
            closed_nodes[current[0], current[1]] = True

            if current == goal:
                path = []
                while current != (None,None):
                    path.append(current)
                    current = (parent_matrix[current[0], current[1], 0], parent_matrix[current[0], current[1], 1])

                # to display path ============
                # for node in path:
                #     maze[node[0],node[1]] = 2
                # for row in maze:
                #     print(row)
                # ============================

                
                return path[::-1]

            # find neighbours of CURRENT
            # create a neighbours list of each row having current_ctg, node_pos

            directions = ((-1,  0), # north
                        ( 0, -1), # west
                        ( 1,  0), # south
                        ( 0,  1)) # east

            for dir in directions:
                neighbour = (current[0] + dir[0], current[1] + dir[1])
                if closed_nodes[neighbour[0], neighbour[1]] or not maze[neighbour[0], neighbour[1]]:
                    continue
                else:
                    # temp = past cost of CURRENT + Heuristic of neighbour + 1 (step size)
                    temp = past_cost_matrix[current[0], current[1]] + 1
                    
                    if temp < past_cost_matrix[neighbour[0], neighbour[1]]:

                        past_cost_matrix[neighbour[0], neighbour[1]] = temp
                        parent_matrix[neighbour[0], neighbour[1]] = current

                        # we have to change the past cost in OPEN as well
                        k=0
                        for i in range(len(open_nodes)):
                            if open_nodes[i][1] == neighbour:
                                try:
                                    open_nodes[i][1] = temp + m.sqrt((goal[0]-neighbour[0])**2 + (goal[1]-neighbour[1])**2)
                                except TypeError:
                                    return list()
                                hp.heapify(open_nodes)
                                k = 1
                                break
                        if not k:        
                            hp.heappush(open_nodes, (temp + m.sqrt((goal[0]-neighbour[0])**2 + (goal[1]-neighbour[1])**2), neighbour))
        return list()

    def lasermsg_2_xy(self, msg):
        distances = np.array(msg.ranges)
        theta = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment*0.5, msg.angle_increment)
        X = distances*np.cos(theta)
        Y = distances*np.sin(theta)
        l_points = np.stack((X,Y,np.ones((len(X)))))
        return l_points

    def laser_callback1(self, laser_msg):

        if self.laser_callback_lock:
            return
        
        self.maze_update_status = True
        # calculate points in lidar frame
        l_points = self.lasermsg_2_xy(laser_msg)

        # calculate points in corner frame
        try:
            t = self.tf_lis.lookupTransform('/odom', '/micromouse_frame', rospy.Time(0))
            (x,y) = t[0][0:2]
            (roll, pitch, yaw) = euler_from_quaternion(t[1])
            yaw *= -1
            T_ol = np.array([[np.cos(yaw), np.sin(yaw), x],[-np.sin(yaw), np.cos(yaw), y],[0,0,1]])
            T_cl = np.matmul(self.T_co, T_ol)

            # only for display ===================
            # v_points = np.matmul(T_ol, l_points)
            # self.display_points(v_points)
            # ====================================

            c_points = np.matmul(T_cl, l_points)
            c_points = c_points//self.divisor
            elements = c_points[0:2, :]

            for point in np.transpose(elements):
                try:
                    self.occ_matrix[int(point[1]),int(point[0])] = 0
                except IndexError:
                    pass

            self.occ_matrix_update += 1
            self.maze_update_status = False

        except tf.LookupException:
            pass

    def laser_callback2(self, laser_msg):

        if self.laser_callback_lock:
            return
        
        self.maze_update_status = True
        # calculate points in lidar frame
        l_points = self.lasermsg_2_xy(laser_msg)

        while True and not rospy.is_shutdown():
            try:
                t = self.tf_lis.lookupTransform('/odom', '/micromouse_frame', laser_msg.header.stamp)
                break
            except (tf.LookupException, tf.ExtrapolationException) as e:
                if 'past' in str(e):
                    return
                
        (x,y) = t[0][0:2]
        (roll, pitch, yaw) = euler_from_quaternion(t[1])
        yaw *= -1
        T_ol = np.array([[np.cos(yaw), np.sin(yaw), x],[-np.sin(yaw), np.cos(yaw), y],[0,0,1]])
        T_cl = np.matmul(self.T_co, T_ol)

        c_points = np.matmul(T_cl, l_points)
        c_points = c_points//self.divisor
        elements = c_points[0:2, :]

        for point in np.transpose(elements):
            try:
                self.occ_matrix[int(point[1]),int(point[0])] = 0
            except IndexError:
                pass
   
        self.occ_matrix_update += 1
        self.maze_update_status = False

if __name__ == '__main__':
    o = scout('/laser/scan','/odom')
