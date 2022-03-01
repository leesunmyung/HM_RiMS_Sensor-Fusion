#!/usr/bin/env python3
import rospy
import sys,os
import os
import cantools
from mobileye_msgs.msg import LaneInfo_N, Mobileye_N, MyLane_N, ObstacleInfo_N
from mobileye_msgs.msg import Mobileye_Lane, Mobileye_Obstacle
from std_msgs.msg import String, Bool
import time


class Mobileye_Filter:

    def __init__(self):

        #Msg 객체 선언
        self.mobileye_msg = Mobileye_N()
        self.mldata = Mobileye_Lane()
        self.modata = Mobileye_Obstacle() 
        #Subscriber
        rospy.Subscriber("/mobileye", Mobileye_N, self.mobileye_callback)
        #Publisher : 2개로 분리
        self.mobileye_lane_pub = rospy.Publisher("/mobileye_lane", Mobileye_Lane, queue_size=1)
        self.mobileye_obstacle_pub = rospy.Publisher("/mobileye_obstacle", Mobileye_Obstacle, queue_size=1)

    def mobileye_callback(self, mobileye_data):
        self.mobileye_msg = Mobileye_N()
        self.mobileye_msg = mobileye_data
        cnt = len(self.mobileye_msg.obstacle)

        msg_idx = 0
        for i in range(0, cnt):
            if (self.mobileye_msg.obstacle[i].obstacle_type == 'Vehicle' and self.mobileye_msg.obstacle[i].obstacle_lane == 'Ego_Lane'):
                self.modata.obstacle[msg_idx] = self.mobileye_msg.obstacle[i]
                msg_idx = msg_idx + 1
        self.modata.max_obstacles = msg_idx 
        self.mldata.left_lane = self.mobileye_msg.left_lane
        self.mldata.right_lane = self.mobileye_msg.right_lane
        self.mldata.mylane = self.mobileye_msg.mylane




def main():
    mf = Mobileye_Filter()
    rospy.init_node('Mobileye_Filter_Node', anonymous=False)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        mf.mobileye_lane_pub.publish(mf.mldata)
        mf.mobileye_obstacle_pub.publish(mf.modata)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
                
