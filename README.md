# robotiq
### 教程一.带有robotiq夹具的ur5机器人ROS包
#### 1.rviz中UR5的可视化
要用抓手可视化机器人模型，请执行以下操作：
```
roslaunch ur5_description display_with_gripper.launch
```
然后，您可以使用滑块更改关节值和抓手值。

#### 2.在gazebo仿真
要模拟机器人启动，请执行以下操作：
```
roslaunch ur5_gazebo ur5_cubes.launch
```
默认情况下，模拟开始暂停,取消暂停模拟,然后您可以发送命令到关节或抓手。

以下是更改抓取器配置的操作客户端的示例。打开一个新的终端，然后执行：
```
rosrun ur5_gazebo send_gripper.py --value 0.5
```
其值为0.0（闭合）和0.8（张开）之间的浮动值。

将关节​​值发送给机器人的示例可以执行如下：
```
rosrun ur5_gazebo send_joints.py
```
要更改关节的位置可以修改send_joints.py文件。

### 教程二.robotiq驱动的安装
```
sudo apt-get install ros-kinetic-soem
```

>如果报错,修改RobotiqHandPlugin.cpp文件:[RobotiqHandPlugin.cpp](https://bitbucket.org/osrf/drcsim/src/194be8500fef81593f79607a21ee2badd9700a0e/drcsim_gazebo_ros_plugins/src/RobotiqHandPlugin.cpp?at=default&fileviewer=file-view-default)

### 教程三.robotiq与实物相连
#### 1.安装依赖
```
rosdep install robotiq_modbus_tcp
```

#### 2.连接robotiq
```
rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11
```
其中`192.168.1.11`为robotiq的IP

#### 3.测试脚本
```
rosrun robotiq_s_model_control SModelSimpleController.py
```
### 教程四.robotiq实例
#### Usage:
>
S-Model (3 finger)
------------------------------
```
rosrun robotiq_cpp_control rs_gripper_interface_test
rosrun robotiq_fake fake_robotiq_s.py
```
>
C-Models (2 finger)
------------------------------
```
rosrun robotiq_cpp_control rc_gripper_interface_test
rosrun robotiq_fake fake_robotiq_c.py
```
