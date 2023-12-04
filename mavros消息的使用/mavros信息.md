# mavros信息

mavros用于无人机通信，可以进行飞控与主控的信息交流

括号内指的是消息的数据类型

## 1.数传

```
3DR_RADIO：
/mavros/radio_status (mavros_msgs/RadioStatus)
```



## 2.全局位置的信息

```
GPS:
/mavros/global_position/global (sensor_msgs/NavSatFix)

sensor_msgs/NavSatFix.msg
-----------------------------------------------
float64 latitude	//经度
float64 lontitude	//纬度
float64 altitude	//高度
float64[9] position_covariance
uint8 position_covariance_type
```



## 3.IMU惯性导航

```
IMU:
/mavros/imu/data (sensor_msgs/Imu)

sensor_msgs/Imu.msg:
------------------------------------------------
std_msgs/Header header
geometry_msgs/Quaternion orientation    //四元数，旋转方向
float64[9] orientation_covariance   //旋转方向的协方差
geometry_msgs/Vector3 angular_velocity    //角速度
float64[9] angular_velocity_covariance     //角速度的协方差
geometry_msgs/Vector3 linear_acceleration    //线加速度
float64[9] linear_acceleration_covariance    //线加速度的协方差

```



## 4.本地位置

**全局位置也是类似的**

发布位置位姿和速度的消息

```
Local pose:
/mavros/local_position/pose (geometry_msgs/PoseStamped)

Local velocity:
/mavros/local_position/velocity (geometry_msgs/TwistStamped)
```



## 5.飞控命令

```
Services:

/mavros/cmd/arming (mavros_msgs/CommandBool)
/mavros/cmd/set_home (mavros_msgs/CommandHome)
/mavros/cmd/takeoff (mavros_msgs/CommandTOL)
/mavros/cmd/land (mavros_msgs/CommandTOL)
/mavros/cmd/trigger_control (mavros_msgs/CommandTriggerControl)

```



## 6.设置飞行位点、速度、加速度

设置加速度

```
set acceleration:
/mavros/setpoint_accel/accel (geometry_msgs/Vector3Stamped)
```

设置速度

```
set velocity:
/mavros/setpoint_velocity/cmd_vel_unstamped (geometry_msgs/Twist)
```

设置位姿

```
set attitude:
/mavros/setpoint_attitude/attitude (geometry_msgs/PoseStamped)
```

设置角速度

```
set angular velocity:
setpoint_attitude/cmd_vel (geometry_msgs/TwistStamped)
```

设置油门

```
set thrust:
/mavros/setpoint_attitude/thrust (mavros_msgs/Thrust)
```

设置飞行点位

```
set local position:
/mavros/setpoint_position/local (geometry_msgs/PoseStamped)

set global position:
/mavros/setpoint_position/global (geographic_msgs/GeoPoseStamped)
```

如果是自己写飞控的化，还需要/mavros/setpointing_raw等话题



## 7.系统状态

用于检测FCU状态

```
Sys state:

Publish:
/mavros/state (mavros_msgs/State)

Service:
/mavros/set_mode (mavros_msgs/SetMode)
```



## 8.系统时间

```
Time:
/mavros/time_reference (sensor_msgs/TimeReference)
```

