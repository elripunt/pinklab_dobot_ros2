# pinklab_dobot_ros2

## bringup
```
source install/local_setup.py
ros2 run dobot_ros2 dashboard_server
```
## power on 
```
ros2 service call /dashboard_server dobot_ros2_interface/srv/Dashboard "{req_str: power on}"
#service cmd: "power on", "power off", "clear error", "gripper on", "gripper off"
```
## move to goal 
```
ros2 action send_goal --feedback /move_goal_server dobot_ros2_interface/action/Move "{goal_pose: {position: {x: 300, y: 19, z: -56.5}, orientation: {w: 1}}}"
```
