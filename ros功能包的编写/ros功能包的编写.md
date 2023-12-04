# ros功能包的编写

## 1.创建工作空间

```shell
#创建文件夹
mkdir qrc_ws
cd qrc_ws
mkdir src
cd src
#创建文件夹 初始化工作空间
catkin_init_workspace
#编译工作空间
cd ~/qrc_ws
catkin_make

#设置环境变量
echo "source ~/qrc_ws/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#设置环境变量的方法2
gedit ~/.bashrc
复制 source ~/qrc_ws/catkin_ws/devel/setup.bash 在文件的最后
保存并退出
source ~/.bashrc
```

## 2.创建功能包

```shell
cd qrc_ws/src

#创建python节点的功能包 功能包的名字叫 offboard_py 后面为环境
catkin_create_pkg offboard_py rospy std_msgs
cd offboard_py
#创建scripts来保存python文件
mkdir scripts
cd scripts
#创建python文件
gedit offb_node.py
#编写完成以后记得授权，不然会出问题
chmod +x offb_node.py
```



## 3.运行功能包

```sh
#运行之前,要启动ros环境
roscore
rosrun offboard_py offb_node.py
```

