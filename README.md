# Fast-Planner

**Fast-Planner** is developed aiming to enable quadrotor fast flight in complex unknown environments. It contains a rich set of carefully designed planning algorithms. Modified from [dyn_planner](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/planning/FastPlanner/plan_manage)

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FFast-Planner.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-2.7.18-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

Planning模块从fast-planner框架借鉴而来，可以实现无人机快速自主飞行。
框架前端kinodynamic路径搜索，后端采用基于样条的轨迹生成，同时还包含了时间调节系统。
Fast-planner可以在及其短的时间内（几毫秒）生成高质量轨迹(依赖增量式sdf地图构建，速度很快，但不是全局更新)。
这里由于建图工作（mapping模块）在别处完成，采用引入全局地图输入，利用sdf_tool生产全局sdf图，然后对系统做软约束优化。

>参考文献  
>[__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (RA-L), 2019.


## Release Note

- v1.1.0: upgrade `sdf_map`
- v1.0.1: 
  - remove `flight_type`
  - remove `message_pub`
  - remove `load_map`
  - remove `pcdpubCallback`
  - remove `sim_mode`

## Installation

Before use, make sure you have installed following packages:

- [libboost](https://www.boost.org/users/history/version_1_65_1.html): suggest version: 1.65.1
- [nlopt](https://github.com/stevengj/nlopt.git): **NEVER** install with `apt install ros-noetic-nlopt`!

```bash
catkin_make install --source src/Fast-Planner --build build/fast_planner
```


## Launch

```bash
roslaunch fast_planner simulation.launch
```

* 从rviz输入需要的期望goal, 选择3d navigation, 同时按下鼠标左右键，然后上下移动标记z大小.
* 从rviz输入目标点信息，目标点高度不要为负值，x,y方向不要超出地图范围（地图参数在launch文件中设置）
* 运行轨迹优化，加载离线地图，等待目标点输入.


[img](log/2024-05-14/rosgraph.png)
