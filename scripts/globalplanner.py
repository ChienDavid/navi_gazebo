#!/usr/bin/env python3
"""
Created on Wed Mar 8 10:15:04 2023
@author: Chien Van Dang

Description: main function running global path planners for Indoor navigation
"""
import math
import numpy as np
import matplotlib.pyplot as plt

import rospy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from slam import SLAM
from astar import Astar
from navi_gazebo.msg import Pathmsgs
from gridmap import OccupancyGridMap
from utils import testmapAct2Sim, convert_path_Sim2Act, angle_quaternion2euler



OBSTACLE = 255
UNOCCUPIED = 0

###################################################################################################
def plot_scenario_1plot(path, local_obs, robot, goal=(3., 4.5)):
    plt.cla()
    plt.title("Navigation: from {} to {} \n Local goal {}".format((round(robot[0],2), round(robot[1],2), round(robot[2]*180/math.pi,2)), (path.subTarget_x_global, path.subTarget_y_global), (round(path.subTarget_x_local,3), round(path.subTarget_y_local,3), round(path.subTarget_theta_local,3))))
    plt.plot(path.globalCoordXs, path.globalCoordYs, ls='--', c='b')
    plt.plot(path.subTarget_x_global, path.subTarget_y_global, 'xg')
    #if path.subTarget_x_local != None and path.subTarget_y_local != None:
    #    plt.plot(-path.subTarget_y_local, path.subTarget_x_local, 'xk')
    plt.plot(local_obs[:,0], local_obs[:,1], '.b')
    #plot_arrow(robot[0], robot[1], robot[2])
    plt.plot(robot[0], robot[1], 'or')
    plt.plot(goal[0], goal[1], 'xr')
    plt.axis('equal')
    plt.grid()
    plt.show()
    #plt.pause(0.01)

def plot_arrow(x, y, yaw, length=0.25, width=0.1):
    plt.arrow(x, y, length*math.cos(yaw), length*math.sin(yaw), head_length=width, head_width=width)

def plot_initial_map(amap):
    plt.cla()
    plt.title("Initial map")
    plt.plot(np.array(amap.inflation)[:,0], np.array(amap.inflation)[:,1], ".g", label="inflation")
    plt.plot(amap.obsx, amap.obsy, ".k", label="lethal")
    plt.legend()
    plt.axis([10, 75, 125, 180])
    #plt.axis('equal')
    plt.grid()
    plt.show()

###################################################################################################
# Global planner
###################################################################################################
class GlobalPlanner:
    def __init__(self, amap, method, resolution=1):
        self.amap = amap
        self.method = method
        self.resolution = resolution

        self.pub_path = rospy.Publisher("/globalpath", Pathmsgs, queue_size=10)
        self.pub_pathrviz = rospy.Publisher("/globalpath_rviz", Path, queue_size=10)

        rospy.Subscriber("/tb3_0/odom", Odometry, self.callback_robot)
        rospy.Subscriber("/tb3_0/scan", LaserScan, self.callback_obstacle)
        rospy.Subscriber("/tb3_0/move_base_simple/goal", PoseStamped, self.callback_goal)
        self.check_connection("/tb3_0/odom", Odometry)
        self.check_connection("/tb3_0/scan", LaserScan)
        self.check_connection("/tb3_0/move_base_simple/goal", PoseStamped)

        self.last_robot = self.robot_sim
        self.last_goal = self.goal_sim
        self.aSlam = SLAM(amap=amap, resolution=self.resolution)

        rospy.loginfo("Initializing {} method".format(self.method))
        self.global_planner = Astar(amap=amap, method=self.method, resolution=self.resolution)

        rospy.loginfo("{} plans from {} to {}".format(self.method, [round(val,3) for val in self.last_robot], [round(val,3) for val in self.last_goal]))
        self.path, t0, length = self.global_planner.plan(robot_pos=self.last_robot, goal_pos=self.last_goal, newmap=amap)

    def check_connection(self, topic, msg_type):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(topic, msg_type, timeout=1)
            except:
                rospy.loginfo("Waiting for topic {}".format(topic))

    def callback_goal(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = angle_quaternion2euler(msg.pose.orientation)
        self.goal_act = np.array([x, y, theta])
        self.goal_sim = testmapAct2Sim(x, y, resolution=self.resolution)

    def callback_obstacle(self, msg):
        self.lidar_msg = msg
        self.view_range = int(msg.range_max * 20 / self.resolution)

    def callback_robot(self, msg):
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        theta = round(angle_quaternion2euler(msg.pose.pose.orientation), 3)
        linear = round(msg.twist.twist.linear.x, 3)
        angular = round(msg.twist.twist.angular.z, 3)
        self.robot_act = np.array([x, y, theta, linear, angular])
        self.robot_sim = testmapAct2Sim(x, y, resolution=self.resolution)

###################################################################################################
    def check_goal(self, robot, goal, goal_radius=3):
        x1, y1 = robot[0], robot[1]
        x2, y2 = goal[0], goal[1]
        dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        if dist <= goal_radius:
            return True
        return False

    def arrange_path(self, pathdata, resolution=1):
        path_act = convert_path_Sim2Act(pathdata, resolution=resolution)
        # for path tracking
        path_tracking = Pathmsgs()
        path_tracking.globalCoordXs = path_act[:,0]
        path_tracking.globalCoordYs = path_act[:,1]
        
        # for RViz
        path_rviz = Path()
        path_rviz.header.stamp = rospy.Time.now()
        path_rviz.header.frame_id = 'map'
        for (x, y) in path_act:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_rviz.poses.append(pose)
        return path_tracking, path_rviz

    def execute(self):
        self.pre_path = []
        while not rospy.is_shutdown():
            # check goal
            if self.check_goal(self.robot_sim, self.goal_sim):
                rospy.loginfo("Robot has arrived the goal!")
                continue

            # check invalid start point
            if self.amap.occupancy_grid_map[self.robot_sim[0], self.robot_sim[1]] == OBSTACLE:
                self.robot_sim = self.last_robot
                
            # update new route
            if self.robot_sim != self.last_robot or self.goal_sim != self.last_goal:
                new_mission = True if self.goal_sim != self.last_goal else False
                self.last_robot, self.last_goal = self.robot_sim, self.goal_sim
                rospy.loginfo("{} plans from {} to {}".format(self.method, [round(val,3) for val in self.last_robot], [round(val,3) for val in self.last_goal]))
                # replan
                slam_map = self.aSlam.rescan(self.robot_sim, self.view_range, self.lidar_msg, self.robot_act, new_mission)
                self.path, t1, length = self.global_planner.plan(robot_pos=self.last_robot, goal_pos=self.last_goal, newmap=slam_map)
            if len(self.path) < 2:
                self.path = self.pre_path
            else:
                self.pre_path = self.path
                
            # check goal
            if self.check_goal(self.robot_sim, self.goal_sim):
                rospy.loginfo("Robot has arrived the goal!")
                continue

            # publish planned route
            path_tracking, path_rviz = self.arrange_path(self.path, resolution=self.resolution)
            self.pub_path.publish(path_tracking)
            self.pub_pathrviz.publish(path_rviz)
            


def main():
    # Create a ROS node
    rospy.init_node("Global_Planner")
    
    # Define a log file
    PLANNERS = ["AstarOnline", "AstarOffline"]
    PLANNER_ID = 1
    ROBOT_MODEL = "Turtlebot3"
    
    # Initialize
    rospy.loginfo("Initializing...")
    resolution = 1.0
    amap = OccupancyGridMap(resolution=resolution)
    global_planner = GlobalPlanner(amap, PLANNERS[PLANNER_ID], resolution=resolution)

    # Execute global path planner
    rospy.loginfo("Start executing...")
    global_planner.execute()
    rospy.loginfo("Mission completed!")

if __name__ == '__main__':
    main()





