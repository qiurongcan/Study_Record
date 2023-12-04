#! /usr/bin/python3
"""
开头需要加上这一行注释，不然找不到python运行的环境
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 创建一个接收无人机状态的变量
current_state = State()

# 创建一个回调函数
def state_cb(msg):
    global current_state
    current_state = msg


# 主函数
if __name__ == "__main__":
    # 初始化一个节点，随便起一个名字
    rospy.init_node("offb_node_py")

    # 实例化一个订阅，订阅话题mavros/state，变量类型为State，回调函数为state_cb
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    # 实例化一个发布 发布话题为mavros/setpoint_position/local(无人机的位置),变量类型为PoseStamped，队列长度为10
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # 等待/mavros/cmd/arming的请求
    rospy.wait_for_service("/mavros/cmd/arming")
    # 实例化一个客户端，客户端为/mavros/cmd/arming,变量类型为CommandBool，用于解锁
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    # 等待/mavros/set_mode的请求
    rospy.wait_for_service("/mavros/set_mode")
    # 实例化一个客户端，客户端为/mavros/set_mode，变量类型为SetMode
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection 等待连接
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # 实例化消息
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    # 在开始之前先发布位置，相当于消息试探，否则无法进行模式的切换
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    # 实例化一个切换模式请求的消息
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    # 实例化一个解锁的请求消息
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        # 如果没有切换成为OFFBOARD模式，尝试切换模式
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            # 如果没有解锁，尝试进行解锁
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        # 此时发布位置，无人机进行运动
        local_pos_pub.publish(pose)

        rate.sleep()
