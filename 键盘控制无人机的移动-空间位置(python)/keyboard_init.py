import rospy
from mavros_msgs.msg import State,WaypointList,WaypointReached,AttitudeTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool,SetMode

class PX4_Control():
    """
    一个用于初始化节点启动、消息的类
    """
    def __init__(self):
        # 初始化一个节点
        self.px4_control_node=rospy.init_node('px4_control_node')
        # 实例化消息
        self.current_state=State()
        self.local_pos=PoseStamped()
        # 订阅无人机的状态和无人机的当前位置
        self.state_sub=rospy.Subscriber('/mavros/state',State,self.state_callback)
        self.local_pos_sub=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.local_pose_callback)
        # 发布无人机的位置
        self.target_pos_pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
        # 设置客户端
        self.arming_client=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.set_mode_client=rospy.ServiceProxy('/mavros/set_mode',SetMode)


    # 定义回调函数
    def state_callback(self,msg):
        self.current_state=msg

    def local_pose_callback(self,msg):
        self.local_pos=msg







