#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.srv import CommandBoolRequest,SetModeRequest
from keyboard_init import PX4_Control


class Control():
    def __init__(self):
        # 初始化
        self.px4_node=PX4_Control()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2
        # 实例化时即开始运行
        self.test_px4()
        

    # 无人机 切换模式、解锁、起飞
    def test_px4(self):
        rate = rospy.Rate(20)
        # 无人机起飞程序
        while(not rospy.is_shutdown() and not self.px4_node.current_state.connected):
            rate.sleep()
      
        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.px4_node.target_pos_pub.publish(self.pose)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()
        # 订阅者 在回调函数中实现键盘控制无人机运动（改变位置）
        rospy.Subscriber("keyboard_input", String, self.keyboard_callback,self.pose)
        
        while(not rospy.is_shutdown()):
            # 切换无人机的模式
            if(self.px4_node.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.px4_node.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            # 解锁无人机
            else:
                if(not self.px4_node.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.px4_node.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            # 不断发布无人机的位置信息
            self.px4_node.target_pos_pub.publish(self.pose)      
            rate.sleep()

    
    # 发布者的回调函数
    def keyboard_callback(self, msg, pose):
        # 无人机解锁
        input_key = msg.data
        if input_key.lower() == "'w'":
            pose.pose.position.z += 0.1
        elif input_key.lower() == "'s'" and pose.pose.position.z > 0.1:
            pose.pose.position.z -= 0.1
        elif input_key.lower() == "'l'":
            pose.pose.position.y -= 0.05
        elif input_key.lower() == "'i'":
            pose.pose.position.x += 0.05
        elif input_key.lower() == "'j'":
            pose.pose.position.y += 0.05
        elif input_key.lower() == "'k'":
            pose.pose.position.x -= 0.05
        rospy.loginfo(pose)
        


if __name__ == '__main__':
    try:
        px4_control_node = Control()
        
    except rospy.ROSInterruptException:
        pass



