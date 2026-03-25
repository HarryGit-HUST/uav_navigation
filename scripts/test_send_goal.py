#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

class MockFSM:
    def __init__(self):
        rospy.init_node('mock_fsm_node', anonymous=True)
        # 订阅你的 Ego-Controller 发出的状态
        rospy.Subscriber("/ego_controller/status", Int8, self.status_cb)
        # 发布目标点给 Ego-Controller
        self.goal_pub = rospy.Publisher("/fsm/ego_goal", PoseStamped, queue_size=1)
        
        self.current_status = 0
        self.status_dict = {0: "待机", 1: "飞行中", 2: "已到达", 3: "规划失败"}

    def status_cb(self, msg):
        if self.current_status != msg.data:
            self.current_status = msg.data
            rospy.loginfo(f"[Mock_FSM] 收到 Ego-Controller 状态更新: {self.status_dict.get(msg.data, '未知')}")

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world" # 或 map，取决于你的里程计基坐标系
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.6 # 固定为你的飞行高度
        
        # 为了安全，这里姿态给默认的无旋转
        goal.pose.orientation.w = 1.0 
        
        rospy.loginfo(f"[Mock_FSM] 正在下发目标点: X={x}, Y={y}")
        self.goal_pub.publish(goal)

    def run(self):
        rospy.sleep(1.0) # 等待发布器建立连接
        while not rospy.is_shutdown():
            try:
                # 交互式输入目标点
                user_input = input("\n请输入目标点坐标 X Y (例如: 3.5 0.0)，输入 q 退出: ")
                if user_input.lower() == 'q':
                    break
                coords = user_input.split()
                if len(coords) == 2:
                    self.send_goal(coords[0], coords[1])
                else:
                    print("输入格式错误，请用空格隔开 X 和 Y")
            except Exception as e:
                print(e)

if __name__ == '__main__':
    try:
        mock = MockFSM()
        mock.run()
    except rospy.ROSInterruptException:
        pass