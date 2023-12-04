# MAVROS的使用

px4使用的是NED坐标系，而mavros使用的是ENU坐标系，mavros在转换为mavlink发送给px4时会自动将ENU转换为NED。



## 1.坐标系的划分

分为global坐标系、local坐标系、body坐标系

1. global坐标系是gps坐标系
2. local坐标系是当地坐标系，原点为上电的地方
3. body坐标系就是机体坐标系

base_link坐标系是body系



## 2.常用话题（mavros对px4的操作）

### （1）/mavros/state

- 订阅
- 功能：订阅mavros的一些状态数据，如连接状态、是否解锁、当前无人机的模式
- 数据类型：mavros_msgs/State

```
string MODE_PX4_MANUAL=MANUAL
string MODE_PX4_ACRO=ACRO
string MODE_PX4_ALTITUDE=ALTCTL
string MODE_PX4_POSITION=POSCTL
string MODE_PX4_OFFBOARD=OFFBOARD
string MODE_PX4_STABILIZED=STABILIZED
string MODE_PX4_RATTITUDE=RATTITUDE
string MODE_PX4_MISSION=AUTO.MISSION
string MODE_PX4_LOITER=AUTO.LOITER
string MODE_PX4_RTL=AUTO.RTL
string MODE_PX4_LAND=AUTO.LAND
string MODE_PX4_RTGS=AUTO.RTGS
string MODE_PX4_READY=AUTO.READY
string MODE_PX4_TAKEOFF=AUTO.TAKEOFF
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
```



### （2）/mavros/setpoint_position/local

- 发布
- 发布指点飞行，当前坐标系为local当地坐标系
- 数据类型：geometry_msgs/PoseStamped

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position  //local 坐标系下的位置（xyz），只有 position 成员变量生效
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

### （3）/mavros/local_position/pose

- 订阅
- 功能：话题内容为当前无人机坐标系在local世界坐标系的位姿，local世界坐标系是以无人机PX4上电点为原点，三轴朝向为东北天（ENU）；无人机坐标系为body坐标系，三轴朝向为前左上；说明一下IMU中的加速度计的大小问题，也就是线加速度，加速度计是考虑除重力外的合力，也就是说当无人机自由落体运动时，Z轴的加速度为0，当无人机静止放置在地面上时，Z轴加速度为+g，因为此时无人机受到地面的支持力，方向竖直向上，大小为g，所以不要因为静止放置的无人机Z轴加速度为+g，就认为Z轴方向朝下
- 数据类型：geometry_msgs/PoseStamped

### （4）/mavros/local_position/velocity_local

- 订阅/发布：订阅
- 功能：话题内容为当前无人机的三轴速度，包括三轴线速度和三轴角速度，坐标系为local坐标系（以无人机上电点为原点、东北天朝向）
- 数据类型：geometry_msgs/TwistStamped



### （5）/mavros/imu/data(_raw)

- 订阅/发布：订阅
- 功能：话题内容为IMU九轴的数据（XYZ加速度、XYZ角速度、XYZ姿态角），data_raw为原始数据，data为滤波后的数据（**有px4自行生成的四元数数据**）。IMU坐标系为前左上body坐标系。
- 数据类型：sensor_msgs/Imu

### （6）/mavros/setpoint_accel/accel

- 订阅/发布：发布
- 功能：设置无人机的加速度，但效果很差，不推荐使用，如果要控加速度，建议使用控制推力的话题
- 数据类型：geometry_msgs/Vector3Stamped



### （7）/mavros/setpoint_velocity/cmd_vel_unstamped

- 发布
- 功能：设置无人机线速度和角速度，坐标系为local坐标系（东北天）
- 数据类型：geometry_msgs/Twist

### （8）/mavros/setpoint_raw/attitude

- 订阅/发布：发布
- 功能：设置无人机姿态、角速度和推力
- 数据类型：mavros_msgs/AttitudeTarget



## 3.常用的服务

### （1）/mavros/cmd/arming

- Server端/Client端：Client端
- 功能：发布解锁/上锁命令
- 数据类型：mavros_msgs/CommandBool



### （2）/mavros/set_mode

- Server端/Client端：Client端
- 功能：请求切换FCU的飞行模式
- 数据类型：mavros_msgs/SetMode

```
uint8 MAV_MODE_PREFLIGHT=0
uint8 MAV_MODE_STABILIZE_DISARMED=80
uint8 MAV_MODE_STABILIZE_ARMED=208
uint8 MAV_MODE_MANUAL_DISARMED=64
uint8 MAV_MODE_MANUAL_ARMED=192
uint8 MAV_MODE_GUIDED_DISARMED=88
uint8 MAV_MODE_GUIDED_ARMED=216
uint8 MAV_MODE_AUTO_DISARMED=92
uint8 MAV_MODE_AUTO_ARMED=220
uint8 MAV_MODE_TEST_DISARMED=66
uint8 MAV_MODE_TEST_ARMED=194
uint8 base_mode
// 常见的模式有MANUAL、ALTCTL、POSCTL、OFFBOARD、STABILIZED、AUTO.LAND
string custom_mode
---
bool mode_sent

```



