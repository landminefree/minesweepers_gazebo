#!/usr/bin/python2
""" Simple script to test the jury plugin. Publish the MineMapSolution msg to the scoring system """

import rospy
from time import sleep
from minesweepers_gazebo.msg import *

if __name__ == '__main__':
    rospy.init_node("mine_solution_publisher")
    pose_pub = rospy.Publisher("/minesweepers_gazebo/mine_map_solution", MineMapSolution, queue_size=5)
    sleep(2)
    solution = MineMapSolution()
    detection_0 = MineDetection()
    detection_0.x = 2
    detection_0.y = 2
    solution.surface_mines.append(detection_0)
    detection_1 = MineDetection()
    detection_1.x = 8
    detection_1.y = 13
    solution.surface_mines.append(detection_1)
    detection_2 = MineDetection()
    detection_2.x = 4
    detection_2.y = 10
    solution.surface_mines.append(detection_2)
    detection_3 = MineDetection()
    detection_3.x = 12
    detection_3.y = 8
    solution.surface_mines.append(detection_3)
    solution.header.stamp = rospy.Time.now()
    solution.header.frame_id = "map"
    pose_pub.publish(solution)
    sleep(2)
