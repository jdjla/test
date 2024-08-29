# CSU-RM-Sentry

中南大学FYT机器人战队哨兵机器人上位机算法（定位与导航部分）。基于点云分割和Nav2导航框架，导航过程中上坡

**深圳北理莫斯科大学北极熊战队**的同学做了一个很出色的仿真环境，用到了本仓库的一些算法，比本仓库更完善：
https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation

**作者: 邹承甫** 

**找我交流: 3548054568(QQ)**
  > 我们不是强队，我也不是啥大佬，只是个代码搬运工，目前因为升学/就业的事比较忙，经常性忘记回复，见谅...

**思路介绍**

- 使用POINT-LIO/FAST_LIO2获得3D里程计
  > 前者可以输出100+Hz的Odometry，对导航更友好，但相对的，CPU占用会更高
- 使用ICP进行重定位
  > 由于一直开着ICP对性能要求较高，所以我们只在第一次启动或者手动设置/initialpose时进行点云配准。获得初始位姿后只依赖LIO进行定位，没有回环检测，在长时间运行后可能会出现累积误差
- 使用linefit_ground_segmentation对MID360的点云进行分割，分割为地面和障碍物
- 将障碍物的点云从PointCloud2压缩为LaserScan，输入Nav2
- 用Nav2进行导航
  > 目前使用的是TEB算法作为局部规划器

|地面分割效果图|导航功能像Nav2一样实现|
|-|-|
|<img src="assets/segment.png"/>|<img src="assets/rviz.png"/>|

**该项目高度参考了以下开源项目，感谢他们**
- [**rm_vision**](https://gitlab.com/rm_vision): 陈君开源的rm_vision项目
- [**TUP-Sentry-Framwork**](https://github.com/tup-robomaster/TUP2023-Sentry-Framework): 沈阳航空航天大学哨兵导航框架

## 1. 框架

- **rm_bringup** (启动相机驱动，串口驱动，自瞄程序和robot_state_publisher，参考rm_vision项目)
- **rm_interfaces** (自定义msg和srv)
- **rm_robot_description** (机器人的urdf)
- **rm_autoaim** (自瞄算法，基于陈君rm_vision项目做了一点修改)
- **rm_localization** (定位算法)
    - [**fast_lio**](https://github.com/Ericsii/FAST_LIO) (提供里程计)
    - [**point_lio**](https://github.com/whyscience/Point-LIO) (提供里程计)
    - **icp_registration** (使用PCL库进行ICP的点云配准，提供map->odom的变换)
- **rm_navigation** (Nav2的launch和参数)
    - **src** (Nav2的参数)
    - [**third_party**](https://github.com/rst-tu-dortmund/teb_local_planner) (TEB算法)
- **rm_perception** (处理传感器数据的一些算法)
    - [**imu_complementary_filter**](https://github.com/CCNYRoboticsLab/imu_tools) (IMU滤波，暂时不用了，节省资源)
    - [**linefit_ground_segmentation**](https://github.com/lorenwel/linefit_ground_segmentation) (点云分割)
    - [**pointcloud_to_laserscan**](https://github.com/ros-perception/pointcloud_to_laserscan) (将PointCloud2转换为LaserScan)
- **rm_hardware_driver** (传感器的驱动)
    - **rm_camera_driver** (工业相机驱动)
    - **rm_serial_driver** (和下位机进行通信的串口驱动程序，参考rmoss开源项目)
    - [**livox_ros_driver2**](https://github.com/Livox-SDK/livox_ros_driver2) (MID360驱动,本仓库进行了部分的修改)



```sh
src
│
├── rm_bringup                
│
├── rm_interfaces
│
├── rm_robot_description           
│
├── rm_autoaim                
│
├── rm_localization           
│   ├── fast_lio        
│   ├── point_lio   
│   └── icp_registration
│
├── rm_navigation
│   ├── src (Nav2) 
│   └── third_party (TEB)
│
├── rm_perception
│   ├── imu_complementary_filter
│   ├── linefit_ground_segementation_ros2
│   └── pointcloud_to_laserscan
│
└── rm_hardware_driver
    ├── rm_camera_driver        
    ├── rm_serial_driver  
    └── livox_ros_driver2
```

## 2. 安装

安装ros-humble-desktop-full，参考 [ROS2官方文档](https://docs.ros.org/en/humble/index.html)

安装Livox-SDK2，参考 [LIVOX-SDK2官方仓库](https://github.com/Livox-SDK/Livox-SDK2)

克隆仓库到本地

```bash
git clone https://github.com/CSU-FYT-Vision/CSU-RM-Sentry && cd CSU-RM-Sentry
```

安装依赖

```bash
sudo apt-get install -y  libpcl-ros-dev
rosdep install --from-paths src --ignore-src -r -y
```

编译
```bash
colcon build --symlink-install
```

运行

> 运行前请按照实际机器人情况，修改rm_robot_description/urdf/sentry.urdf中的坐标系定义

```bash
# 建图
./mapping.sh
# 导航
./nav.sh
```

- 如何移植到自己的机器人上？
  - 1. 修改rm_robot_description中urdf中的坐标系定义
  - 2. 修改rm_sensors/livox_ros_driver2中的mid360外参
  - 3. 根据实际情况修改rm_localization/linefit_ground_segmentation_ros/launch/segmentation_params.yaml中的sensor_height参数
  - 4. 根据实际情况修改rm_perception/pointcloud_to_laserscan/launch/pointcloud_to_laserscan_launch.py中min_height和max_height参数
  - 5. 根据需求修改nav2的参数
  - 6. 你可能还要调一调FAST_LIO的参数和linefit_ground_segmentation的参数以达到最好效果


- 如何配合rm_vision？
  - 在rm_vision项目中，需要IMU(rm_vision里的odom系，下称为gimbal_odom)到相机的坐标变换，这可能与导航的tf树有冲突，解决方法可以参考下面：
    - 1. 修改urdf文件和LIO，将LIO的输出从livox_frame转到gimbal_odom，让自瞄与导航共用一个tf树
    - 2. 自瞄和导航分别使用不同的tf树，这样会造成移动射击时命中率下降

## 3. 依赖

- **系统**
  - Ubuntu 22.04
  - ROS Humble (desktop-full)
- **库**
  - [LIVOX-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
  - libpcl-ros-dev
  - eigen、pcl、opoencv、ceres等

## 4. 硬件

- **Mini PC** 
    - cpu: AMD Ryzen R7 7735HS 
    - ram: 16GB DDR5
- **传感器**
    - Livox MID-360 
    - 大恒水星工业相机
