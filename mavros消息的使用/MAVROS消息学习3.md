# MAVROS消息学习3

## Command指令

```shell
# 指令消息(不知道干嘛的)
cmd/command 
# 改变解锁状态
cmd/arming
# 设置原点
cmd/set_home
# 设置起飞
cmd/takeoff
# 设置着陆
cmd/land
# 相机跟踪控制
cmd/trigger_control
```

## imu_pub

```shell
# imu数据，飞控计算得到的姿态
imu/data
# Raw数据，无姿态数据
imu/data_raw
# 温度
imu/temperature
# 压力
imu/atm_pressure
```



## local_position

```shell
# 相对于原点的位置（原点：默认无人机上电的点）(x,y,z)
local_position/pose
# 速度
local_position/velocity
```



## setpoint_accel

设置加速度

```shell
# 加速度向量或者力向量
setpoint_accel/accel
# 是否设置力传送
setpoint_accel/send_force(bool)
```





## setpoint_attitude

```shell
# 设置姿态速度
setpoint_attitude/cmd_vel
# 设置姿态
setpoint_attitude/attitude
# 设置升力的阈值
setpoint_attitude/trust
```



## setpoint_position

```shell
setpoint_position/global
setpoint_position/local
setpoint_position/global_to_local
```

