#!/usr/bin/env python3
import rospy
import os
import cantools
from can_msgs.msg import Frame
from mobileye_msgs.msg import ObstacleInfo_N, MyLane_N, LaneInfo_N, Mobileye_N
from chassis_msgs.msg import MsgCentroid, Centroid
from retina_view.msg import MsgRadarPoint, Point, Track
from std_msgs.msg import String, Bool
import time


class Adjacent_Vehicle:

    def __init__(self):

        self.mobileye_msg = Mobileye_N()
        self.radar_msg = MsgCentroid()

        self.mdata = Mobileye_N()
        #Subscriber
        rospy.Subscriber("/mobileye", Mobileye_N, self.mobileye_callback)
        rospy.Subscriber("/radar_centroid", MsgCentroid, self.radar_callback)

        #Publisher : Vehicle Type의 Msg 작성 필요
        self.av_pub = rospy.Publisher("/adjacent_vehicle", Vehicle, queue_size=1)
        self.avdata = Vehicle()

        
        
    def mobileye_callback(self, mobileye_data):
        self.mobileye_msg = Mobileye()
        self.mobileye_msg = mobileye_data
        cnt = len(self.mobileye_msg.obstacle)
        obstacle_index = []
        final_obstacle_list = []

        #13번 돌기?
        for i in range(0, cnt): 
            #obstacle이 자동차이고, 자차선에 있다면
            if (self.mobileye_msg.obstacle[i].obstacle_type == 'vehicle' and self.mobileye_msg.obstacle[i].obstacle_lane == 'Ego_Lane'):
                obstacle_index.append(i)
        
        for i in range(0, len(obstacle_index)):
            final_obstacle_list[i] = self.mobileye_msg


    
    def centroid_callback(self, cent_data):
        self.radar_msg = MsgCentroid()
        self.radar_msg = cent_data
        cnt = len(self.radar_msg.centroid)



def main():
	av = Adjacent_Vehicle()
	rospy.init_node('Adjacent_Vehicle_Detection_Node', anonymous=False)
	rate = rospy.Rate(10)
    
	while not rospy.is_shutdown():
		av.av_pub.publish(av.avdata)
		rate.sleep()
	rospy.spin()
	
if __name__ == '__main__':
	main()
	
			

