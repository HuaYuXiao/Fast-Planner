# Fast-Planner

**Fast-Planner** is developed aiming to enable quadrotor fast flight in complex unknown environments. It contains a rich set of carefully designed planning algorithms. 

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FFast-Planner.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.6.9-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)

Planning模块从fast-planner框架借鉴而来，可以实现无人机快速自主飞行。
框架前端kinodynamic路径搜索，后端采用基于样条的轨迹生成，同时还包含了时间调节系统。
Fast-planner可以在及其短的时间内（几毫秒）生成高质量轨迹(依赖增量式sdf地图构建，速度很快，但不是全局更新)。
这里由于建图工作（mapping模块）在别处完成，采用引入全局地图输入，利用sdf_tool生产全局sdf图，然后对系统做软约束优化。

>参考文献  
>[__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (RA-L), 2019.


## 1. 安装

* 非线性优化工具箱 [**NLopt**](https://github.com/stevengj/nlopt)

```bash
cd ~/tool_ws/src/
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```


## 2. 编译

```bash
cd ~/planner_ws
catkin_make install --source src/Fast-Planner --build build/Fast-Planner
```

## 3. 运行

* 运行轨迹优化，加载离线地图，等待目标点输入.  

```bash
roslaunch prometheus_plan_manage prometheus_planning_test_static.launch
```

> 修改 pcd_file 为自己的配置参数

* 运行rviz显示地图、轨迹，同时给出目标点.  

```bash
roslaunch prometheus_plan_manage rviz_static.launch
```

* 从rviz输入需要的期望goal, 选择3d navigation, 同时按下鼠标左右键，然后上下移动标记z大小.


## 4. 致谢
* 使用 **nlopt**作为非线性优化工具 （位于/ThirdParty）(https://nlopt.readthedocs.io/en/latest/NLopt_Installation)
* 使用 **sdf_tool**为地图转化工具
* 参考 **fast-planner** 优化框架

## 5. 说明
* 与控制接口  plan_manage/src/traj_server.cpp  （未完，待补充）
> msgs/msg/PositionReference.msg

* 输入odom信息（topic: "/planning/odom_world"）
* 输入pcd地图信息（目前地图只支持有限空间地图，地图大小、分辨率在launch文件设置，topic： "/planning/global_point_cloud"）
* 从rviz输入目标点信息，目标点高度不要为负值，x,y方向不要超出地图范围（地图参数在launch文件中设置）