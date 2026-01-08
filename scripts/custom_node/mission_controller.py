#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Controller Node
- State Manager의 상태에 따라 각 미션 노드를 enable/disable
- 현재 상태에 해당하는 노드만 cmd_vel 발행
"""

import rospy
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist


class MissionController:
    def __init__(self):
        rospy.init_node('mission_controller')
        
        # State definitions
        self.STATES = {
            "WAITING_TRAFFIC_LIGHT": 0,
            "DETECT_PEDESTRIAN": 1,
            "ROTARY": 2,
            "OBSTACLE_AVOIDANCE": 3,
            "PARKING": 4
        }
        
        self.current_state = "WAITING_TRAFFIC_LIGHT"
        
        # Lane steering from lane_detect_node
        self.lane_steering = 0.0
        self.lane_speed = 0.25
        
        # Mission-specific stop flags
        self.traffic_stop = True  # 초기값 True: 신호등 상태 받기 전까지 정지
        self.pedestrian_stop = False
        self.roundabout_stop = False
        self.obstacle_steering = 0.0
        self.obstacle_active = False
        self.parking_active = False
        
        # Publishers for enabling/disabling each mission node
        self.pub_traffic_enable = rospy.Publisher('/limo/traffic_enable', Bool, queue_size=1)
        self.pub_pedestrian_enable = rospy.Publisher('/limo/pedestrian_enable', Bool, queue_size=1)
        self.pub_roundabout_enable = rospy.Publisher('/limo/roundabout_enable', Bool, queue_size=1)
        self.pub_obstacle_enable = rospy.Publisher('/limo/obstacle_enable', Bool, queue_size=1)
        self.pub_parking_enable = rospy.Publisher('/limo/parking_enable', Bool, queue_size=1)
        
        # cmd_vel publisher (only mission_controller publishes to cmd_vel)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.sub_state = rospy.Subscriber('/state_manager/state', String, self.state_callback, queue_size=1)
        
        # Lane detection data
        self.sub_steering = rospy.Subscriber('/limo/steering_offset', Float32, self.steering_callback, queue_size=1)
        self.sub_speed = rospy.Subscriber('/limo/lane_speed', Float32, self.speed_callback, queue_size=1)
        
        # Mission stop signals
        self.sub_traffic_stop = rospy.Subscriber('/limo/traffic_stop', Bool, self.traffic_stop_callback, queue_size=1)
        self.sub_pedestrian_stop = rospy.Subscriber('/limo/pedestrian_stop', Bool, self.pedestrian_stop_callback, queue_size=1)
        self.sub_roundabout_stop = rospy.Subscriber('/limo/roundabout_stop', Bool, self.roundabout_stop_callback, queue_size=1)
        
        # Obstacle avoidance steering
        self.sub_obstacle_steering = rospy.Subscriber('/limo/obstacle_steering', Float32, self.obstacle_steering_callback, queue_size=1)
        self.sub_obstacle_active = rospy.Subscriber('/limo/obstacle_active', Bool, self.obstacle_active_callback, queue_size=1)
        
        # Parking status
        self.sub_parking_active = rospy.Subscriber('/limo/parking_active', Bool, self.parking_active_callback, queue_size=1)
        
        # Control loop timer (20Hz)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        # Enable status publish timer (1Hz)
        self.enable_timer = rospy.Timer(rospy.Duration(1.0), self.publish_enable_status)
        
        rospy.loginfo("="*60)
        rospy.loginfo("Mission Controller Node initialized")
        rospy.loginfo("Waiting for state from /state_manager/state")
        rospy.loginfo("="*60)
    
    def state_callback(self, msg):
        """State Manager로부터 상태 수신"""
        if self.current_state != msg.data:
            rospy.loginfo(f"State changed: {self.current_state} -> {msg.data}")
            self.current_state = msg.data
            self.publish_enable_status(None)  # 즉시 enable 상태 업데이트
    
    def steering_callback(self, msg):
        self.lane_steering = msg.data
    
    def speed_callback(self, msg):
        self.lane_speed = msg.data
    
    def traffic_stop_callback(self, msg):
        self.traffic_stop = msg.data
    
    def pedestrian_stop_callback(self, msg):
        self.pedestrian_stop = msg.data
    
    def roundabout_stop_callback(self, msg):
        self.roundabout_stop = msg.data
    
    def obstacle_steering_callback(self, msg):
        self.obstacle_steering = msg.data
    
    def obstacle_active_callback(self, msg):
        self.obstacle_active = msg.data
    
    def parking_active_callback(self, msg):
        self.parking_active = msg.data
    
    def publish_enable_status(self, event):
        """각 미션 노드에 enable/disable 상태 발행"""
        self.pub_traffic_enable.publish(Bool(data=(self.current_state == "WAITING_TRAFFIC_LIGHT")))
        self.pub_pedestrian_enable.publish(Bool(data=(self.current_state == "DETECT_PEDESTRIAN")))
        self.pub_roundabout_enable.publish(Bool(data=(self.current_state == "ROTARY")))
        self.pub_obstacle_enable.publish(Bool(data=(self.current_state == "OBSTACLE_AVOIDANCE")))
        self.pub_parking_enable.publish(Bool(data=(self.current_state == "PARKING")))
    
    def control_loop(self, event):
        """현재 상태에 따라 cmd_vel 발행"""
        cmd = Twist()
        
        if self.current_state == "WAITING_TRAFFIC_LIGHT":
            # 신호등 미션: 빨간불이면 정지, 초록불이면 차선 추적
            if self.traffic_stop:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = self.lane_speed
                cmd.angular.z = -self.lane_steering
        
        elif self.current_state == "DETECT_PEDESTRIAN":
            # 보행자 미션: 보행자 감지시 정지, 아니면 차선 추적
            if self.pedestrian_stop:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = self.lane_speed
                cmd.angular.z = -self.lane_steering
        
        elif self.current_state == "ROTARY":
            # 로터리 미션: 장애물 감지시 정지, 아니면 차선 추적
            if self.roundabout_stop:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = self.lane_speed
                cmd.angular.z = -self.lane_steering
        
        elif self.current_state == "OBSTACLE_AVOIDANCE":
            # 장애물 회피 미션: 장애물 회피 active면 회피 조향, 아니면 차선 추적
            if self.obstacle_active:
                cmd.linear.x = self.lane_speed * 0.8  # 회피 중 속도 감소
                cmd.angular.z = self.obstacle_steering
            else:
                cmd.linear.x = self.lane_speed
                cmd.angular.z = -self.lane_steering
        
        elif self.current_state == "PARKING":
            # 주차 미션: parking_node가 직접 제어하거나, 차선 추적
            if self.parking_active:
                # parking_node가 직접 cmd_vel 발행
                return
            else:
                cmd.linear.x = self.lane_speed
                cmd.angular.z = -self.lane_steering
        
        else:
            # 알 수 없는 상태: 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(5.0, f"Unknown state: {self.current_state}")
        
        self.pub_cmd.publish(cmd)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = MissionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
