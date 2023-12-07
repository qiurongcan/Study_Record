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
        self.px4_control_node=rospy.init_node('px4_control_node2')
        # 实例化消息
        self.current_state=State()
        self.local_pos=PoseStamped()
        # self.waypoint_list=WaypointList()
        # self.waypoint_reached=WaypointReached()
        # 实例化订阅、发布、服务等
        self.state_sub=rospy.Subscriber('/mavros/state',State,self.state_callback)
        self.local_pos_sub=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.local_pose_callback)
        # self.waypoint_sub=rospy.Subscriber("/mavros/mission/Waypoints",WaypointList,self.waypoint_callback)
        # self.waypoint_reached_sub=rospy.Subscriber(r'/mavros/mission/reached',WaypointReached,self.waypoint_reached_callback)

        # 发布无人机的位置
        self.target_pos_pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
        # self.attitude_pub=rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=10)

        self.arming_client=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.set_mode_client=rospy.ServiceProxy('/mavros/set_mode',SetMode)


    # 定义回调函数

    def state_callback(self,msg):
        self.current_state=msg

    def local_pose_callback(self,msg):
        self.local_pos=msg

    # def waypoint_callback(self,msg):
    #     self.waypoint_list=msg
    
    # def waypoint_reached_callback(self,msg):
    #     self.waypoint_reached=msg






