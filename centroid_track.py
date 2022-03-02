#!/usr/bin/env python3
import rospy
import sys,os
import os
import cantools
from retina_view.msg import MsgRadarPoint, Point, Track
from chassis_msgs.msg import MsgCentroid, Centroid
from std_msgs.msg import String, Bool
import time


class Centroid_Track:

    def __init__(self):
        
        self.radar_msg = MsgRadarPoint()
        rospy.Subscriber("/radar_data", MsgRadarPoint, self.radar_callback)
        self.centroid_pub = rospy.Publisher("/radar_centroid", MsgCentroid, queue_size=1)
        self.cdata = MsgCentroid()

    # cnt : track의 개수
    # track_cnt : publish할 centroid에 대한 인덱스 값
    # points_cnt : track 당 point의 개수

    def radar_callback(self, radar_data):
        #Radar에서 해야 하는 일 track별 Centroid값 구해서 publish하기
        self.radar_msg = MsgRadarPoint()
        self.radar_msg = radar_data
        cnt = self.radar_msg.track_count
        track_cnt = -1
        points_cnt = 0
        # track_cnt가 한 트랙에 포함된 점의 개수라면, 왜 -1부터 시작하나요?(선)
        # track_cnt는 publish할 때 인덱스
        # track이 0번부터 15번까지 잡힌다는 가정하에, Vehicle이 15번 인덱스라면
        # for 문 인덱스 i를 그대로 썼을 때(15번), 0번부터 15번까지 총 16개의 객체가 생길 수 있으므로
        # track_cnt라는 값을 centroid의 인덱스로 사용하기 위해 선언 
        # publish할 메시지에 값을 넣어주는 것보다 centroid sum하는 부분이 더 위에 있어서
        # track_cnt = -1로 선언하고, for문 안에서 0부터 시작하는 로직
        for i in range(0, cnt):
            points_cnt = 0
            if (self.radar_msg.track_info[i].track_type == 1):
                track_id = self.radar_msg.track_info[i].track_id
                track_cnt = track_cnt + 1
                for i in self.radar_msg.points:
                    if (self.radar_msg.points.track == track_id):
                        #points_cnt : track 당 points들이 몇개 있는지
                        points_cnt = points_cnt + 1
                        centroid_x = centroid_x + self.radar_msg.points.x
                        centroid_y = centroid_y + self.radar_msg.points.y
                        centroid_z = centroid_z + self.radar_msg.points.z
                        centroid_doppler = centroid_doppler + self.radar_msg.points.doppler
                        centroid_power = centroid_power + self.radar_msg.points.power

                #평균값 구하고 메시지에 값 넣어주기
                self.cdata.centroid[track_cnt].x = centroid_x / points_cnt
                self.cdata.centroid[track_cnt].y = centroid_y / points_cnt
                self.cdata.centroid[track_cnt].z = centroid_z / points_cnt
                self.cdata.centroid[track_cnt].doppler = centroid_doppler / points_cnt
                self.cdata.centroid[track_cnt].power = centroid_power / points_cnt
                self.cdata.centroid[track_cnt].track_id = track_id
                self.cdata.centroid[track_cnt].track_type = self.radar_msg.track_info[i].track_type
                self.cdata.centroid[track_cnt].track_range = self.radar_msg.track_info[i].track_range
                self.cdata.centroid[track_cnt].point_count = points_cnt


def main():
    ct = Centroid_Track()
    rospy.init_node('Centroid_Track_Node', anonymous=False)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        ct.centroid_pub.publish(ct.cdata)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
                
