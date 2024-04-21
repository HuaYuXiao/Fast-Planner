# arc_utilities

C++ and Python utilities used in lab projects

### Compile

```bash
catkin_make install --source src/Fast-Planner/plan_env/ThirdParty/arc_utilities --build build/arc_utilities
```

### Install

```bash
sudo cp ~/planner_ws/src/Fast-Planner/plan_env/ThirdParty/arc_utilities/lib/libarc_utilities.so /opt/ros/noetic/lib/
sudo cp -r ~/planner_ws/src/Fast-Planner/plan_env/ThirdParty/arc_utilities/include/arc_utilities /opt/ros/noetic/lib/
```

check if has been updated

```bash
ls -la /opt/ros/noetic/lib/ | grep arc_utilities
```
