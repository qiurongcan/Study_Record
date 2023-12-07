# 更换PX4的仿真环境

PX4仿真环境的配置文件是`/home/ljw/PX4_Firmware/launch/posix_sitl.launch`，与之前一样，我们不对原始文件进行修改，我们修改副本。

```sh
cd ~/PX4_Firmware/launch
cp posix_sitl.launch posix_sitl_cp.launch
gedit posix_sitl_cp.launch
```



将

```xml
<!-- Gazebo sim -->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="gui" value="$(arg gui)"/>
	<arg name="world_name" value="$(arg world)"/>
	<arg name="debug" value="$(arg debug)"/>
	<arg name="verbose" value="$(arg verbose)"/>
	<arg name="paused" value="$(arg paused)"/>
	<arg name="respawn_gazebo" value="$(arg respawn_gazebo)"/>
</include>
```

修改为

```xml
<!-- Gazebo sim -->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="gui" value="$(arg gui)"/>
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/empty.world"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="verbose" value="$(arg verbose)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="respawn_gazebo" value="$(arg respawn_gazebo)"/>
</include>
```

也就是将原本的gazebo的.world文件换成turtlebot3小车的empty.world文件，这个world里什么都没有。光这样修改还没有生效，因为我们修改的是副本，原始调用这个文件的文件也需要修改，调用这个文件的文件就是之前的mavros_posix_sitl_cp.launch
