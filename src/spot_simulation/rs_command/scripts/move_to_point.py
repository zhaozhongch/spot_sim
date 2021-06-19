#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib

roslib.load_manifest('teleop_legged_robots')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys, select, termios, tty

import tf_conversions

import numpy as np
import math


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        robot_name = rospy.get_param("~/robot_name")
        twist_publisher_name = rospy.get_param("~/twist_publisher_name", "/" + robot_name + "/cmd_vel")
        pose_publisher_name = rospy.get_param("~/pose_publisher_name", "/" + robot_name + "/body_pose")
        self.twist_publisher = rospy.Publisher(twist_publisher_name, Twist, queue_size=1)
        self.pose_publisher = rospy.Publisher(pose_publisher_name, Pose, queue_size=1)
        self.robot_pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_model)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_roll = 0.0
        self.pose_pitch = 0.0
        self.pose_yaw = 0.0
        self.pose_speed = 0.0
        self.pose_turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.delay_wait_print = 4

        self.search_index = -1 #-1 means the last element in the list
        self.robot_name = robot_name#no `/`. Used for searching its index in the callback_model
        self.show_time = 0
        self.x_inst     = 0
        self.y_inst     = 0
        self.z_inst     = 0
        self.roll_inst  = 0
        self.pitch_inst = 0
        self.yaw_inst   = 0

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and (self.twist_publisher.get_num_connections() == 0 or
                                           self.pose_publisher.get_num_connections() == 0):
            if i == self.delay_wait_print:
                if self.twist_publisher.get_num_connections() == 0:
                    rospy.loginfo("Waiting for subscriber to connect to {}".format(self.twist_publisher.name))
                if self.pose_publisher.get_num_connections() == 0:
                    rospy.loginfo("Waiting for subscriber to connect to {}".format(self.pose_publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % (self.delay_wait_print + 1)
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw, pose_speed,
               pose_turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.pose_z = pose_z
        self.pose_roll = pose_roll
        self.pose_pitch = pose_pitch
        self.pose_yaw = pose_yaw
        self.pose_speed = pose_speed
        self.pose_turn = pose_turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, self.pose_x, self.pose_y, self.pose_z, self.pose_roll, self.pose_pitch,
                    self.pose_yaw, self.pose_speed, self.pose_turn)
        self.join()

    def run(self):
        twist = Twist()
        pose = Pose()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            # Copy state into pose message.
            pose.position.x = self.pose_x
            pose.position.y = self.pose_y
            pose.position.z = self.pose_z
            pose_roll_euler = self.pose_roll
            pose_pitch_euler = self.pose_pitch
            pose_yaw_euler = self.pose_yaw

            quaternion = tf_conversions.transformations.quaternion_from_euler(pose_roll_euler, pose_pitch_euler, pose_yaw_euler)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            self.condition.release()

            # Publish.
            self.twist_publisher.publish(twist)
            self.pose_publisher.publish(pose)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pose.position.x = self.pose_x
        pose.position.y = self.pose_y
        pose.position.z = self.pose_z
        pose_roll_euler = self.pose_roll
        pose_pitch_euler = self.pose_pitch
        pose_yaw_euler = self.pose_yaw

        quaternion = tf_conversions.transformations.quaternion_from_euler(pose_roll_euler, pose_pitch_euler, pose_yaw_euler)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        self.twist_publisher.publish(twist)
        self.pose_publisher.publish(pose)

    def callback_model(self, data):
        """ Read the data of Spot` positions an orientations"""
        if self.search_index == -1:
           self.search_index = data.name.index(self.robot_name)
        #print("name of all models ",data.name)
        #exit(0)
        self.x_inst = data.pose[self.search_index].position.x
        self.y_inst = data.pose[self.search_index].position.y
        self.z_inst = data.pose[self.search_index].position.z
        orientation_q = data.pose[self.search_index].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll_inst, self.pitch_inst, self.yaw_inst) = euler_from_quaternion(orientation_list)
        current_time = rospy.get_time()
        if current_time - self.show_time > 0.5:
            print("robot position ",self.x_inst,self.y_inst,self.z_inst)
            self.show_time = current_time


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def pose_vel(pose_speed, pose_turn):
    return "currently:\tpose_speed %s\tpose_turn %s " % (pose_speed, pose_turn)


def pose_print(pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw):
    return "currently:\tx %s\ty %s\tz %s\troll %s\tpitch %s\tyaw %s " % (
        pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw)


# def check_status_msg(status_msg, msg_max):
#     if status_msg == msg_max:
#         rospy.loginfo(msg)
#     return (status_msg + 1) % (msg_max + 1)

if __name__ == "__main__":

    rospy.init_node("send_velocity_command")

    speed = rospy.get_param("~/speed", 1.2)#original 0.5 when Tswing = 0.17. I reset Tswing to 0.1 and default speed to 1.2
    turn = rospy.get_param("~/turn", 1.0)
    pose_speed = rospy.get_param("~/pose_speed", 0.01)
    pose_turn = rospy.get_param("~/pose_turn", 0.1)
    repeat = rospy.get_param("~/repeat_rate", 0.0)
    msg_max = rospy.get_param("~/msg_max", 14)

    goal_x = rospy.get_param("~/goal_x", 5.5)
    goal_y = rospy.get_param("~/goal_y", 8.6)

    goal_marker_name = rospy.get_param("~/goal_marker_name","marker")

    print("goal x and goal y", goal_x, goal_y)


    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    pose_x = 0
    pose_y = 0
    pose_z = 0
    pose_roll = 0
    pose_pitch = 0
    pose_yaw = 0
    status_msg = 0  # number of printed additional messages

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw, pose_speed,
                          pose_turn)

        #rospy.loginfo(msg)
        rospy.loginfo(vels(speed, turn))

        rate = rospy.Rate(10)
        ln = 0
        alpha = np.pi

        arrive = False

        #A simple experiment, move to a point
        while not arrive and not rospy.is_shutdown():
            while abs(alpha)>0.1 and not rospy.is_shutdown():
                ln+=1

                dy = pub_thread.y_inst - goal_y
                dx = pub_thread.x_inst - goal_x
                alpha = np.arctan2(dy, dx) - pub_thread.yaw_inst;
                print("dy and dx is ",dy,dx)
                print("arctan2, yaw_inst",np.arctan2(dy, dx), pub_thread.yaw_inst)
                if alpha > np.pi:
                    alpha = 2*np.pi - alpha
                elif alpha < -np.pi:
                    alpha = 2*np.pi + alpha

                #turn
                if alpha>0:
                    turn = 1
                elif alpha < 0:
                    turn = -1
                else:
                    turn = 0

                print("turn is ", turn)
                pub_thread.update(x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                                pose_speed, pose_turn)
                #print("update message ",ln)
                print("angle to the goal", alpha)
                #print("all values ", x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                #                  pose_speed, pose_turn)
                rate.sleep()
                x = 0
                y = 0
                z = 0
                th = 1
                if abs(alpha) < 0.3:
                    th = 0.3

            turn = 0
            dy = pub_thread.y_inst - goal_y
            dx = pub_thread.x_inst - goal_x
            dist = math.sqrt(dx*dx + dy*dy)
            rate = rospy.Rate(10)

            while dist>0.2 and not rospy.is_shutdown():
                dy = pub_thread.y_inst - goal_y
                dx = pub_thread.x_inst - goal_x

                alpha = np.arctan2(dy, dx) - pub_thread.yaw_inst;
                print("dy and dx is ",dy,dx)
                print("arctan2, yaw_inst",np.arctan2(dy, dx), pub_thread.yaw_inst)
                if alpha > np.pi:
                    alpha = 2*np.pi - alpha
                elif alpha < -np.pi:
                    alpha = 2*np.pi + alpha

                if abs(alpha)>0.1:
                    x = 0
                    y = 0
                    z = 0
                    break;

                dist = math.sqrt(dx*dx + dy*dy)

                x = -1
                y = 0
                z = 0
                th = 1
                

                pub_thread.update(x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                                pose_speed, pose_turn)
                #print("update message ",ln)
                print("distance to the goal", dist)
                #print("all values ", x, y, z, th, speed, turn, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                #                  pose_speed, pose_turn)
                rate.sleep()
            
            if dist<0.2:
                arrive = True
        if arrive or rospy.is_shutdown():
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(goal_marker_name)
        print("angle to the is ",alpha, " exit")

    except Exception as e:
        rospy.logerr(e)

    finally:
        pub_thread.stop()