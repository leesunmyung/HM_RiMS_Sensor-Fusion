#!/usr/bin/env python3
import rospy
import os
import cantools
from can_msgs.msg import Frame
from mobileye_msgs.msg import ObstacleInfo_N, MyLane_N, LaneInfo_N, Mobileye_N
from std_msgs.msg import String, Bool
import time

#1. Mobileye에서 어떤 Data를 받을 건지 정한 후 메시지 파일부터 만들기
#2. Socketcan_bridge에서 Mobileye 관련 CAN 코드 만들기
#3. dbc 파일을 이용한 파싱 코드 작성 (chassis_can_parser_mobileye.py 고치기)

class can_parser:

    def __init__(self):
        script_dir = os.path.dirname(__file__)
        can_db_path = os.path.join(script_dir, '../DBC/Mobileye.dbc')
        self.db = cantools.database.load_file(can_db_path)
        self.cb_time = time.time()

        rospy.Subscriber("can/mobileye/rx", Frame, self.rx_callback)
        self.rx_pub = rospy.Publisher("/mobileye", Mobileye_N, queue_size=1)
        self.cdata = Mobileye_N()
        self.cdata.max_obstacles = 13
        
        for i in range(self.cdata.max_obstacles):
        	self.cdata.obstacle.append(ObstacleInfo_N())

		
    def fill_obstacle_info(self, obstacle, info_type, signals):

        if info_type == 0:
            obstacle.obstacle_brake_lights = str(signals['Obstacle_Brake_Lights'])
            obstacle.obstacle_status = str(signals['Obstacle_Status'])
            obstacle.obstacle_type = str(signals['Obstacle_Type'])
            obstacle.obstacle_rel_vel_x = signals['Obstacle_Relative_Velocity_X']
            obstacle.obstacle_pos_x = signals['Obstacle_Position_X']
            obstacle.obstacle_pos_y = signals['Obstacle_Position_Y']
            #obstacle.obstacle_ID = signals['Obstacle_ID']

        elif info_type == 1:
            obstacle.obstacle_lane = str(signals['Obstacle_Lane'])
            obstacle.obstacle_age = signals['Obstacle_Age']
            obstacle.obstacle_width = signals['Obstacle_Width']
            obstacle.obstacle_length = signals['Obstacle_Length']

        elif info_type == 2:
            obstacle.object_accel_x = signals['Object_Accel_X']

    def fill_mylane_info(self, mylane, signals):
            
        mylane.pitch_angle = signals['Pitch_Angle']
        mylane.yaw_angle = signals['Yaw_Angle']
        mylane.lane_heading = signals['Lane_Heading']
        mylane.lane_curvature = signals['Lane_Curvature']

    def fill_lane_info(self, lane, info_type, signals):
        
        if info_type == 0:
            lane.curve_param_C3 = signals['Curve_Parameter_C3']
            lane.curve_param_C2 = signals['Curve_Parameter_C2']
            lane.curve_param_C0 = signals['Curve_Parameter_C0']
            lane.quality = str(signals['Quality'])
            lane.lane_type = str(signals['Lane_Type'])

        elif info_type == 1:
            lane.curve_param_C1 = signals['Curve_Parameter_C1']

    def rx_callback(self, data):
        try:
            signals = self.db.decode_message(data.id, data.data)
        except KeyError:
            return

        self.cdata.header.stamp = data.header.stamp
        
        #MyLane 정보 파싱
        if data.id == 1847:
            self.fill_mylane_info(self.cdata.mylane, signals)

        #left_lane, right_lane 정보 파싱
        elif data.id >= 1894 and data.id <= 1897:
            lane_message_id = data.id - 1894
            (lane_id, info_type) = divmod(lane_message_id, 2)

            if lane_id == 0:
                self.fill_lane_info(self.cdata.left_lane, info_type, signals)

            elif lane_id == 1:
                self.fill_lane_info(self.cdata.right_lane, info_type, signals)

        # #next_left_lane, next_right_lane 정보 파싱
        # elif data.id >= 1900 and data.id <= 1915:
        #     lane_message_id = data.id - 1900
        #     (lane_id, lr_type) = divmod(lane_message_id, 4)
        #     (lr_type, info_type) = divmod(lr_type, 2)
            
        #     if lr_type == 0:
        #         self.fill_lane_info(self.cdata.next_left_lane[lane_id], info_type, signals)

        #     elif lr_type == 1:
        #         self.fill_lane_info(self.cdata.next_right_lane[lane_id], info_type, signals)
        
        # #obstacle 정보 파싱
        elif data.id >= 1849 and data.id <= 1887:
            obstacle_message_id = data.id - 1849
            (obstacle_id, info_type) = divmod(obstacle_message_id, 3)
            self.fill_obstacle_info(self.cdata.obstacle[obstacle_id], info_type, signals)


def main():
	cp = can_parser()
	rospy.init_node('mobileye_can_parser', anonymous=False)
	rate = rospy.Rate(10)
    
	while not rospy.is_shutdown():
		cp.rx_pub.publish(cp.cdata)
		rate.sleep()
	rospy.spin()
	
if __name__ == '__main__':
	main()
	
			

