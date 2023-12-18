#! /usr/bin/python3

# 初始化无人机的话题订阅、发布、服务等操作
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool,SetMode
from mavros_msgs.srv import CommandBoolRequest,SetModeRequest
from std_msgs.msg import String


class PX4_Control():
    def __init__(self):
        # 初始化位置消息
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0.1
        # 初始化一个节点
        self.px4_control_node=rospy.init_node('px4_control_node')
        # 实例化消息
        self.current_state=State()
        self.local_pos=PoseStamped()

        # 订阅无人机的状态
        self.state_sub=rospy.Subscriber('/mavros/state',State,self.state_callback)
        # 订阅无人机的位置
        self.local_pos_sub=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.local_pose_callback)
        # 订阅键盘发布的消息
       
        # 发布无人机位置
        self.target_pos_pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
        
        # 此处可以添加发布其他话题
        # --------------------

        # 无人机解锁服务端
        self.arming_client=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        # 无人机模式设置服务端
        self.set_mode_client=rospy.ServiceProxy('/mavros/set_mode',SetMode)

        self.run()

    
    def run(self):
        # 发布话题的频率
        self.rate = rospy.Rate(20)
        
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()
        # 订阅键盘输入的消息
        rospy.Subscriber("keyboard_input2", String, self.sub_keyboard_callback)
        # 解锁和切换模式之前，先发布消息

        while(not rospy.is_shutdown()):

            self.target_pos_pub.publish(self.pose)    
            self.rate.sleep()



    # 回调函数
    def state_callback(self,msg):
        self.current_state=msg

    def local_pose_callback(self,msg):
        self.local_pos=msg
        
    
    def sub_keyboard_callback(self,msg):
        key = msg.data
        # 解锁请求消息
        arm_cmd = CommandBoolRequest()
        # 切换模式请求消息
        offb_set_mode = SetModeRequest()
        
        # 切换成offboard模式
        if key.lower()=="b":
            offb_set_mode.custom_mode = 'OFFBOARD'
            arm_cmd.value = True
            self.change_mode_cmd(offb_set_mode,arm_cmd)
        
        # # y解锁
        # elif key.lower()=='y':
        #     
        # # t上锁
        # elif key.lower()=='t':
        #     arm_cmd.value = False

        elif key.lower() == "w":
            self.pose.pose.position.z += 0.1
        elif key.lower() == "s" and self.pose.pose.position.z > 0.1:
            self.pose.pose.position.z -= 0.1
        elif key.lower() == "l":
            self.pose.pose.position.y -= 0.05
        elif key.lower() == "i":
            self.pose.pose.position.x += 0.05
        elif key.lower() == "j":
            self.pose.pose.position.y += 0.05
        elif key.lower() == "k":
            self.pose.pose.position.x -= 0.05
        rospy.loginfo(self.pose)


    def change_mode_cmd(self,mode,cmd):
        last_req = rospy.Time.now()
        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.target_pos_pub.publish(self.pose)
            self.rate.sleep()
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.arming_client.call(cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        


if __name__=="__main__":
    PX4_Control()

