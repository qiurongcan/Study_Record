#! /usr/bin/python3

import rospy
from std_msgs.msg import String
from pynput import keyboard

"""
只能实现无人机的平移
发布键盘控制消息
通过按下键盘控制无人机的移动，这里实现一个无人机的位置控制
包括高度上升和下降 w/s
左右移动 j/l
前后移动 i/k
是全局监听键盘的消息
"""

global flag
flag=1
def main():
    
    rospy.init_node('keyboard_pub') 
    # 创建键盘发布者  
    keyboard_pub=rospy.Publisher('/keyboard_input',String,queue_size=10)

    while not rospy.is_shutdown():

        def on_press(key):
            # 按下键盘发布一次消息
            msg=String()
            # 按下键盘赋值一次
            msg.data=str(key).strip()
            # print(msg.data)
            keyboard_pub.publish(msg)

        def on_release(key):
            # 按下esc退出键盘控制
            if key == keyboard.Key.esc:
                global flag
                flag=0
                return False
       
        with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
            listener.join()
        if flag==0:
            break
        rospy.spin()
            


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


