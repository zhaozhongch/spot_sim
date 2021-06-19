#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib

roslib.load_manifest('teleop_legged_robots')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import DeleteModel
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from request_info.srv import *
from std_msgs.msg import Bool

from rs_command.srv import *

import sys, select, termios, tty

import tf_conversions

import numpy as np
import math

import matplotlib.pyplot as plt

show_animation = True

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, unit):
        """
        resolution: grid resolution [unit]
        ox: x position list of Obstacles [unit]
        oy: y position list of Obstacles [unit]
        resolution: grid resolution [unit]
        rr: robot radius[unit]
        unit: [m]
        """
        #self.model_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)
        #self.link_state_subscriber = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_state_callback)
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.unit = unit
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        print("start node index, position ", start_node.x, start_node.y, sx, sy)
        print("goal node index, position ", goal_node.x, goal_node.y, gx, gy)

        open_set, closed_set = dict(), dict()
        print("index ",self.calc_grid_index(start_node))
        open_set[self.calc_grid_index(start_node)] = start_node
        print("openset size ", len(open_set))

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            # In the graph, we show the real_position/unitm. However, in the computation, we need to move all the nodes's x and y to 0 based because the obstable_map[id_x][id_y] id_x and id_y must be positive.
            #So when we initialize the node, we use calc_xy_index(sx, self.min_x) to get node id to 0 based
            # but when we need to plot, we need to use add the original index back to get the real position on the plot
            if show_animation:  # pragma: no cover
                plt.plot(current.x + self.min_x,
                         current.y + self.min_y, "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                #print("expand grid 0")

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                #print("expand grid 1")

                if n_id in closed_set:
                    continue

                #print("expand grid 2")

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_index):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = (index  + min_index) * self.unit
        return pos

    def calc_xy_index(self, position, min_index):
        #in python 2, int() will turn, for example, min_index == -43, self.unit = 0.2, position == 20, I get 23.0 from (position - min_index * self.unit)/self.unit)
        # but int() turns 23.0 to 22.... maybe 23.0 is 22.999999..., so I firstly ceil it, the ceil in python2 also returns float.. WTF... Then int again
        #try to use int - int if possible instead of float - int * float.
        #return int(math.ceil((position - min_index * self.unit)/self.unit))
        return int(position/self.unit) - min_index
        

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        #print("node position", px,py)

        #print("boundary ", self.min_x * self.unit, self.min_y * self.unit, self.max_x * self.unit, self.max_y * self.unit)

        if px < (self.min_x * self.unit):
            return False
        elif py < (self.min_y * self.unit):
            return False
        elif px >= (self.max_x * self.unit):
            return False
        elif py >= (self.max_y * self.unit):
            return False

        #print("out of boundary ? ", px,py)
        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        #print("collision ? ", px,py)
        return True

    def calc_obstacle_map(self, ox, oy):
        print("unit length is ", self.unit)
        self.min_x = int(round(min(ox)))
        self.min_y = int(round(min(oy)))
        self.max_x = int(round(max(ox)))
        self.max_y = int(round(max(oy)))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = int(round((self.max_x - self.min_x)))
        self.y_width = int(round((self.max_y - self.min_y)))
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width+1)]
                             for _ in range(self.x_width+1)]

        for iox, ioy in zip(ox, oy):
            iox = iox - self.min_x
            ioy = ioy - self.min_y
            #print(iox,ioy)
            self.obstacle_map[int(iox)][int(ioy)] = True

        # for every point in the grid, check its distance in the initial obstacle map, if the distance is smaller than robot radius, then save it as obstacle.
        # The efficiency is low.
        # for ix in range(self.x_width):
        #     x = self.calc_grid_position(ix, self.min_x)
        #     for iy in range(self.y_width):
        #         y = self.calc_grid_position(iy, self.min_y)
        #         for iox, ioy in zip(ox, oy):
        #             d = math.hypot(iox - x, ioy - y)
        #             if d <= self.rr:
        #                 self.obstacle_map[ix][iy] = True
        #                 break
        print("get the obstacle map ........")

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

    def model_state_callback(self, data):
        """ Get all the model's position, add them to the obstacle map"""
        print("model numbers ",len(data.name))

        for nm in data.name:
            print("model name ",nm)
            model_index = data.name.index(nm)
            x = data.pose[model_index].position.x
            y = data.pose[model_index].position.y
            z = data.pose[model_index].position.z
            print("model positon, reference_frame ",x,y,z, data.pose[model_index].reference_name)
        
        rospy.signal_shutdown("We are done here!")

    def link_state_callback(self, data):
        """ Get all the links' position, add them to the obstacle map"""
        for nm in data.name:
            print("link name ",nm)
            model_index = data.name.index(nm)

        rospy.signal_shutdown("We are done here!")
    

def main():
    rospy.init_node("A star")

    goal_x = rospy.get_param("~/goal_x", 5.5)
    goal_y = rospy.get_param("~/goal_y", 8.6)

    print("wait for the service from the gazebo plugin")
    rospy.wait_for_service('/model_info')
    print("get the service")

    model_info = rospy.ServiceProxy('/model_info', GetModelInfo)
    '''
    service returns

    std_msgs/String names
    std_msgs/Float64MultiArray locations
    std_msgs/Float64MultiArray orientations
    std_msgs/Float64MultiArray dimensions
    std_msgs/Int64 num
    '''

    #false means we get all the links info of the model
    req_type = Bool(False)
    resp = model_info(req_type)

    # print('model links size ',resp.num.data)
    # #if resp.num.data == 1:
    # print('model name ', resp.names.data)
    # print('model bounding box ', resp.dimensions.data)
    # print('model location ', resp.locations.data)
    # print('model direction ', resp.orientations.data)

    '''
    #For the wall, we only have parallel to the x axis or perpendicular to the axis to make out life easier. Each wall is a cuboid, 2D is a rectangle.
    #We then cut the world by a lot of 10cm by 10cm grids, if the wall's (x,y) is within that grids, then is we fill that grid as an obstacle.
    #For the wall that can only be parallel/perpendicular to the axis. We simply calculate its min_x,y, max_x,y, project min_x,y, max_x,y to the world grid, find the correponding id and fill that grid
    # 
    # However, we should know how to solve a wall that is not perpendicular/parallel to the axis.
    # We firstly should get the min_x,y, max_x,y of that wall. Then for all the grids in that wall, we find their center, we need to calculate if the center is within that rectangle. A lot of ways, see https://www.cnblogs.com/carekee/articles/3731713.html .If the center is within the rectangel ,we fill that grid
    '''
    #set obstacle position
    ox,oy = [],[]

    # start and goal position
    sx = rospy.get_param("~/start_x", 10)  # [m]
    sy = rospy.get_param("~/start_y", 10) # [m]
    gx = rospy.get_param("~/goal_x", -10)  # [m]
    gy = rospy.get_param("~/goal_y", -10)  # [m]
    unit = 20 #[cm]
    unitm = unit/100.0 #[m]
    grid_size = 1  # [unit]
    robot_radius = 10  # [unit]

    grid_x = 0.1
    grid_y = 0.1
    for i in range(resp.num.data):
        yaw = resp.orientations.data[3*i+2]
        center_x = resp.locations.data[3*i]
        center_y = resp.locations.data[3*i+1]
        bbox_x   = resp.dimensions.data[3*i]
        bbox_y   = resp.dimensions.data[3*i+1]

        min_x = center_x - bbox_x/2
        max_x = min_x + bbox_x
        min_y = center_y - bbox_y/2
        max_y = min_y + bbox_y

        #decide the id in the grid
        id_min_x = int(min_x/unitm)
        id_min_y = int(min_y/unitm)
        id_max_x = int(max_x/unitm)
        id_max_y = int(max_y/unitm)

        #we make the obstacle "thicker" to count for the robot's own length, width...
        #for m in range(id_min_x, id_max_x+1):
        #    for n in range(id_min_y, id_max_y+1):
        for m in range(id_min_x-1, id_max_x+2):
            for n in range(id_min_y-1, id_max_y+2):
                oy.append(n)
                ox.append(m)
    
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx/unitm, sy/unitm, "og")
        plt.plot(gx/unitm, gy/unitm, "xb")
        plt.grid(True)
        plt.axis("equal")
        #plt.show()

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, unitm)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    rx_plot = [x / unitm for x in rx]
    ry_plot = [y / unitm for y in ry]
    
    #send the way points to the client
    way_points = []
    for x,y in zip(rx,ry):
        print("x,y",x,y)
        way_points.append(Point(x,y,0))

    rospy.wait_for_service('/get_way_points')
    send_way_points = rospy.ServiceProxy('/get_way_points', ReqWayPoints)
    sent = send_way_points(way_points)
    print("message send ", sent.success.data)

    if show_animation:  # pragma: no cover
        plt.plot(rx_plot, ry_plot, "-r")
        plt.pause(0.001)
        plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()