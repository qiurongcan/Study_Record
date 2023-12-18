#! /usr/bin/python3
import rospy
import sys,select,os
import tty,termios
from std_msgs.msg import String


# 获取键盘输入的指令
def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist,_,_=select.select([sys.stdin],[],[],0.1)
    if rlist:
        key=sys.stdin.read(1)
    else:
        key=""
    
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    return key


if __name__=="__main__":
    # 初始化节点
    rospy.init_node('keyboard_pub')
    # 发布话题的名称
    keyboard_pub=rospy.Publisher('/keyboard_input2',String,queue_size=10)
    msg=String()
    while 1:
        # 设置settings
        settings=termios.tcgetattr(sys.stdin)
        # 获取键盘输入的值
        key=get_key()
        # 如果为q，则推出键盘控制模式
        if key=="q":
            rospy.loginfo("-------推出键盘操作模式--------")
            break
        # 如果不为空，则读取键盘，并发布消息
        if key !="":
            msg.data=str(key)
            keyboard_pub.publish(msg)

    # 恢复终端设置
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        

