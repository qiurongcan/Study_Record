#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBoolRequest,SetModeRequest
from pos_init import PX4_Control


class Control():
    def __init__(self):
        # 初始化
        self.px4_node=PX4_Control()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2
        
        self.test_px4()
        

    # 无人机 切换模式、解锁、起飞
    def test_px4(self):
        rate = rospy.Rate(20)

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
        # 订阅位置的输入
        rospy.Subscriber("pos_input", PoseStamped, self.pos_callback,self.pose)
        
        while(not rospy.is_shutdown()):
            if(self.px4_node.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.px4_node.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not self.px4_node.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.px4_node.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()
            self.px4_node.target_pos_pub.publish(self.pose)
            # self.px4_node.attitude_pub.publish(self.attitude)       
            rate.sleep()

    
    # 回调函数 更新无人机的位置为键盘输入的位置
    def pos_callback(self, msg, pose):
        pose.pose.position.x=msg.pose.position.x
        pose.pose.position.y=msg.pose.position.y
        pose.pose.position.z=msg.pose.position.z
        rospy.loginfo(pose)
        


if __name__ == '__main__':
    try:
        px4_control_node = Control()
    except rospy.ROSInterruptException:
        pass



