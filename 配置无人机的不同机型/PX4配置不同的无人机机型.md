# PX4配置不同的无人机机型

在mavros_posix_sitl.launch文件中修改launch文件

可以用的机型有iris，rover(汽车），typhoon_h480（六旋翼无人机）



## iris系列

### 1.iris标准无人机

```html
<arg name="vehicle" default="iris"/>
```

### 2.带相机的iris无人机(双目相机)（stereo_camera)

```html
<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/>
```

修改为

```html
<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_stereo_camera/iris_stereo_camera.sdf"/>
```

对比一下两个文件的不同

### 3.带深度相机iris无人机（depth_camera)

```html
<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_depth_camera/iris_depth_camera.sdf"/>
```



### 4.深度相机朝下的（downward_depth_camera)

```html
<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_downward_depth_camera/iris_downward_depth_camera.sdf"/>
```



### 5.带雷达的（rplidar）

### 6.扫描范围更广阔的（foggy_lidar)



使用iris系列无人机进行仿真即可

