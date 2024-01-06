# pinklab_dobot_ros2

## bringup
```
source install/local_setup.py
ros2 run dobot_ros2 dashboard_server
```
## 로봇 전원 제어
```
ros2 service call /dashboard_server dobot_ros2_interface/srv/Dashboard "{req_str: power on}"
#service cmd: "power on", "power off", "clear error", "gripper on", "gripper off"
```
### OR - client 실행
```
ros2 run dobot_ros2 dashboard_client "power on"
#service cmd: "power on", "power off", "clear error", "gripper on", "gripper off"
```

## 로봇팔 속도 설정
```
ros2 param set /dashboard speed_value 30
# value: 0 ~ 100
```

## move to goal - 로봇팔 이동
```
ros2 action send_goal --feedback /move_goal_server dobot_ros2_interface/action/Move "{goal_pose: {position: {x: 300, y: 19, z: -56.5}, orientation: {w: 1}}}"
```
### OR - client 실행
```
 ros2 run dobot_ros2 move_goal_clinet 300 19 -10
```
