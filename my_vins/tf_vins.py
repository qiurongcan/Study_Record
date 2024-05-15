import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import tf
import sys

# vehicle_type = sys.argv[1]
# vehicle_id = sys.argv[2]
local_pose = PoseStamped()
local_pose.header.frame_id = 'world'
quaternion = tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

def vins_callback(data):    
    local_pose.pose.position.x = data.pose.pose.position.x
    local_pose.pose.position.y = data.pose.pose.position.y
    local_pose.pose.position.z = data.pose.pose.position.z
    q_= Quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
    q_ = q_*q
    local_pose.pose.orientation.w = q_[0]
    local_pose.pose.orientation.x = q_[1]
    local_pose.pose.orientation.y = q_[2]
    local_pose.pose.orientation.z = q_[3]

rospy.init_node('vins_transfer')
rospy.Subscriber("/vins_estimator/camera_pose", Odometry, vins_callback,queue_size=1)
position_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
rate = rospy.Rate(30) 

while not rospy.is_shutdown():
    if (local_pose.pose.position == Point()):
        continue
    else:
        print("Vins pose received")
        local_pose.header.stamp = rospy.Time.now()
        position_pub.publish(local_pose) 
    try:
        rate.sleep()
    except:
        continue
