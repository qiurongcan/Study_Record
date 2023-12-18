
# 通过opencv进行简单的图像划分处理

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time
from geometry_msgs.msg import PoseStamped


class ImageDetect():
    """
    1-这是一个通过opencv读取px4图像,通过回调函数进行处理
    2-通过颜色识别进行目标检测,检测出目标的位置
    3-对比目标位置相对于图像中心点的位置,计算无人机应该相应移动的位置
    4-发布无人机得消息
    5-无人机移动,移动与无人机的初始摆放位置有关
    -------------------------------
    使目标始终位于无人机摄像机的中心点中
    """
    
    def __init__(self):
        # 初始化一个节点
        rospy.init_node("image_detect",anonymous=True)

        # 创建图像窗口
        cv2.namedWindow("Image",cv2.WINDOW_NORMAL)

        # 创建一个图像转换器
        self.bridge=CvBridge()

        # 订阅mavros发布的图像消息
        rospy.Subscriber("/iris/camera/rgb/image_raw",Image,self.image_callback)

        # 发布位置消息
        self.pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # 初始化一个物体在无人机相框中的位置
        self.obj_pos=[]

        # 初始化是否检测到物体的标志
        self.find=False

        self.pose=PoseStamped()
        self.pose.pose.position.x=0
        self.pose.pose.position.y=0
        self.pose.pose.position.z=2


    # 无人机图像读取的回调函数
    def image_callback(self,data):
        try:
            start_time=float(time())
            # 将图像转换为cv2格式
            img=self.bridge.imgmsg_to_cv2(data,'bgr8')

            
            img=self.detect(img)


            end_time=float(time())
            # 计算检测的fps
            fps=round(1/(end_time-start_time),2)
            # 显示fps
            cv2.putText(img,f'FPS:{fps}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
            # 显示图像
            cv2.imshow("Image",img)

            # 如果检测到物体
            if self.find:
                # 进行跟踪,传入图像和物体的位置
                self.track(img=img,pos=self.obj_pos)
            self.pos_pub.publish(self.pose)


            key=cv2.waitKey(1) & 0xFF
            # 按q健推出图像检测模式
            if key == ord('q'):
                rospy.signal_shutdown("----推出图像检测模式----")
                cv2.destroyAllWindows()
            
            # 每一次结束,都要将检测物体的标志重置
            self.find=False

        except Exception as e:
            print(e)
        
    
    def run(self):
        rospy.spin()

    # 图像检测 ,这个函数可以修改成不同的检测模型
    def detect(self,img):
        # print(img.shape)
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        # 定义蓝色的范围
        lower_blue=np.array([100,50,50])
        upper_blue=np.array([130,255,255])

        # 创建蓝色掩码
        mask=cv2.inRange(hsv,lower_blue,upper_blue)

        # 查找物体轮廓
        contours, _ =cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x,y,w,h=cv2.boundingRect(contour)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

            self.obj_pos=[x,y,w,h]
            self.find=True

        return img


    # 进行一个简单的二维平面跟踪
    def track(self,img,pos):
        # 图像的中心位置
        y0,x0,_=img.shape
        x0=x0/2;y0=y0/2
        # print(x0,y0)
        # 物体的位置
        x,y,w,h=pos

        # 物体的中心位置
        x1=x+w/2
        y1=y+h/2
        # print(x1,y1)
        # 相对位置
        dx=x1-x0
        dy=y1-y0

        print(f'相对位置为:{dx,dy} px')
        
        self.move(dx=dx,dy=dy)

        # 无人机移动,是相对位置,即dx,dy为0

    def move(self,dx,dy):
        # 到达指定的位置
        if np.abs(dx)<=10 and np.abs(dy)<=10:
            return
        # 进行y方向上的移动
        if np.abs(dy)>10:
            if dy>0:
                self.pose.pose.position.x -=0.01
            
            else:
                self.pose.pose.position.x +=0.01
        
        if np.abs(dx)>10:
            if dx>0:
                self.pose.pose.position.y -=0.01
            
            else:
                self.pose.pose.position.y +=0.01
            
            
            # 发布无人机的位置
            rospy.loginfo(f'无人机的位置 x:{self.pose.pose.position.x},y:{self.pose.pose.position.y}')
            



if __name__=="__main__":
    img_detect=ImageDetect()
    img_detect.run()
        

"""
存在的不足:
1.无人机是在二维平面进行移动,高度z是没有变化的,需要实现三维空间的跟踪
2.锁定在中心区域位置的时候,波动比较剧烈,可能是设置了10px的范围的问题
3.只能通过位置控制来实现,在速度,姿态上暂时不能实现
4.不能跟踪物体的旋转,需要检测物体旋转,作出相应的偏航角旋转
5.多个目标的时候,会出现不知道跟踪哪个目标,需要一个锁定目标的方法
6.没有测算物体与实际目标之间的距离问题
7.没有实现无人机在跟踪过程中躲避障碍物的问题
8.没有解决目标丢失的问题,如何重新找到目标
9.预测跟踪目标的轨迹问题
10.跟踪速度延迟较大
11.还需要熟悉一下px4的速度,位置,角度,角速度控制的原理
"""

