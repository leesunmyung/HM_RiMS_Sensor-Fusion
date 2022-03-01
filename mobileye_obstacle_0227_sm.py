#!/usr/bin/env python3
import rospy
import os
import cantools
#from retina_view.msg import MsgRadarPoint, Point, Track
from mobileye_msgs.msg import Mobileye_N, LaneInfo_N, MyLane_N, ObstacleInfo_N
#from chassis_msgs.msg import MsgCentroid, Centroid
from std_msgs.msg import String, Bool
import time

class Mobileye_Obstacle:

    def __init__(self):
        self.mobileye_msg = Mobileye_N()
        rospy.Subscriber("/mobileye_data", Mobileye_N, self.mobileye_callback)
        self.mobileye_pub = rospy.Publisher("/mobileye_obstacle", Mobileye_N, queue_size=1)
        self.mdata = Mobileye_N()


    def mobileye_callback(self, mobileye_data):
        self.mobileye_msg = Mobileye_N()
        self.mobileye_msg = mobileye_data

        cnt = mobileye_data.max_obstacles # 14

        for i in range(0, cnt) :
            if (((mobileye_data.obstacle[i].obstacle_type).startswith('V')) and ((mobileye_data.obstacle[i].obstacle_lane).startswith('E'))) :
                self.mdata.obstacle[i] = self.mobileye_msg.obstacle[i]

        self.mdata.max_obstacles = mobileye_data.max_obstacles
        self.mdata.left_lane = mobileye_data.left_lane
        self.mdata.right_lane = mobileye_data.right_lane
        self.mdata.mylane = mobileye_data.mylane

def main() :
    mo = Mobileye_Obstacle()
    rospy.init_node('Mobileye_Obstacle_Node', anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() :
        mo.mobileye_pub.publish(mo.mdata)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__' :
    main()
