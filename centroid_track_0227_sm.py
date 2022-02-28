#!/usr/bin/env python3
import rospy
import os
import cantools
from retina_view.msg import MsgRadarPoint, Point, Track
from chassis_msgs.msg import MsgCentroid, Centroid
from std_msgs.msg import String, Bool
import time

# temp_list : [track_id, track_type, track_range, 해당 track내 point cloud 개수, x, y, z, doppler, power]
# temp_list : track_id, track_type, track_range 외 다른 요소들은 0으로 초기화하고 시작
# result_list : temp_list 배열을 요소로 가지는 이차원 배열

class Centroid_Track:

    def __init__(self):
        self.radar_msg = MsgRadarPoint()
        rospy.Subscriber("/radar_data", MsgRadarPoint, self.radar_callback)
        self.centroid_pub = rospy.Publisher("/radar_centroid", MsgCentroid, queue_size=1)
        self.cdata = MsgCentroid()


    def radar_callback(self, radar_data):
        self.radar_msg = MsgRadarPoint()
        self.radar_msg = radar_data

        self.cdata.track_count = radar_data.track_count

        all_points = radar_data.nTargets

        id_list = []        # id 배열
        result_list = []    # 최종 이차원 결과 배열
        temp_list = []      # 한 track의 정보 담을 임시 배열 선언

        id_index = 0        # id 배열 내, 해당 id의 인덱스
        is_vehicle = 0      # track_type == 1인 경우를 나타내는 flag

        for i in range(0, all_points) :     # 한 프레임 내, 모든 point cloud들 순회

            # 예외처리문 : 어떠한 track에도 속하지 않는 point cloud 처리 위함
            # 'track_info'라는 attribute의 값이 없다는 에러로 인한 종료를 막기 위함
            try :
                if (self.radar_data.track_info[i].track_type == 1) : # vehicle
                    is_vehicle = 1
                else :
                    is_vehicle = 0
            except :
                continue

            # track_type == 1(차량) 인 경우
            if is_vehicle == 1 :
                track_id = self.radar_data.track_info[i].track_id

                # id 배열에 해당 id가 없는 경우(초기)
                if track_id not in id_list :
                    id_list.append(track_id)
                    temp_list.append(track_id)  # temp_list[0] : track_id
                    temp_list.append(self.radar_data.track_info[i].track_type) # temp_list[1] : track_type (==1)
                    temp_list.append(self.radar_data.track_info[i].track_range) # temp_list[2] : track_range
                    for j in range(3, 9) :
                        temp_list.append(0)
                    # temp_list 예시 : [3, 1, 153.5, 0, 0, 0, 0, 0, 0]
                    result_list.append(temp_list)

                # id 배열에서 해당 id의 index 가져오기
                id_index = id_list.index(track_id)
                result_list[id_index][3] = result_list[id_index][3] + 1
                result_list[id_index][4] = result_list[id_index][4] + self.radar_data.points.x
                result_list[id_index][5] = result_list[id_index][5] + self.radar_data.points.y
                result_list[id_index][6] = result_list[id_index][6] + self.radar_data.points.z
                result_list[id_index][7] = result_list[id_index][7] + self.radar_data.points.doppler
                result_list[id_index][8] = result_list[id_index][8] + self.radar_data.points.power

                # 다음 for문 돌기 위해 flag 초기화
                is_vehicle = 0

            # track_type 이 차량이 아닌 경우, 다음 point cloud 순회
            else :
                continue

        i = 0
        for i in range(len(id_list)) :
            result_list[i][4] = result_list[i][4] / result_list[i][3]
            result_list[i][5] = result_list[i][5] / result_list[i][3]
            result_list[i][6] = result_list[i][6] / result_list[i][3]
            result_list[i][7] = result_list[i][7] / result_list[i][3]
            result_list[i][8] = result_list[i][8] / result_list[i][3]

            self.cdata.centroid[i].track_id = result_list[i][0]
            self.cdata.centroid[i].track_type = result_list[i][1]
            self.cdata.centroid[i].track_range = result_list[i][2]
            self.cdata.centroid[i].point_count = result_list[i][3]
            self.cdata.centroid[i].x = result_list[i][4]
            self.cdata.centroid[i].y = result_list[i][5]
            self.cdata.centroid[i].z = result_list[i][6]
            self.cdata.centroid[i].doppler = result_list[i][7]
            self.cdata.centroid[i].power = result_list[i][8]

def main() :
    ct = Centroid_Track()
    rospy.init_node('Centroid_Track_Node', anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() :
        ct.centroid_pub.publish(ct.cdata)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__' :
    main()
