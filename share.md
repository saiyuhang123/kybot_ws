nvidia@nvidia-desktop:~/kybot_ws$ source ~/kybot_ws/install/setup.bash
ros2 launch my_rviz_panel mission_executor.launch.py
[INFO] [launch]: All log files can be found below /home/nvidia/.ros/log/2026-08-02-19-16-06-362724-nvidia-desktop-28616
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mission_executor-1]: process started with pid [28629]
[mission_executor-1] [INFO] [1785669366.540653648] [mission_executor]: MissionExecutor ready
[mission_executor-1] [INFO] [1785669366.540960816] [mission_executor]: Registered action handler: 'log'
[mission_executor-1] [INFO] [1785669366.552492688] [mission_executor]: Registered action handler: 'grasp'
[mission_executor-1] [INFO] [1785669366.553574608] [mission_executor]: Registered action handler: 'place'
[mission_executor-1] [INFO] [1785669366.554546320] [mission_executor]: Registered action handler: 'home2'
[mission_executor-1] [INFO] [1785669366.555732240] [mission_executor]: Registered action handler: 'ready'
[mission_executor-1] [INFO] [1785669744.183067792] [mission_executor]: Mission started: 1 waypoints
[mission_executor-1] [INFO] [1785669744.192818512] [mission_executor]: Checking camera login...
[mission_executor-1] [INFO] [1785669744.272189072] [mission_executor]: Camera login: Login OK, user_id=0 (success=1)
[mission_executor-1] [INFO] [1785669744.272715568] [mission_executor]: Stowing arm to Home2 before navigation...
[mission_executor-1] [INFO] [1785669759.907900496] [mission_executor]: Arm Home2: 已回 Home2 位姿 (success=1)
[mission_executor-1] [INFO] [1785669759.908026288] [mission_executor]: [1/1] Navigating...
[mission_executor-1] [INFO] [1785669772.746400144] [mission_executor]: [1/1] Arrived
[mission_executor-1] [INFO] [1785669772.746510960] [mission_executor]: [1/1] PTZ check: pan=0
[mission_executor-1] [INFO] [1785669772.746535792] [mission_executor]: [1/1] Capture check: do_capture=1
[mission_executor-1] [INFO] [1785669772.746563312] [mission_executor]: [1/1] Capturing...
[mission_executor-1] [INFO] [1785669772.746630160] [mission_executor]: capturePicture: checking service...
[mission_executor-1] [INFO] [1785669772.746984592] [mission_executor]: capturePicture: service available, sending request
[mission_executor-1] [INFO] [1785669773.106318160] [mission_executor]: capturePicture result: success=1, msg=Captured: /home/nvidia/kybot_ws/src/hk_camera/pic_capture/1785669772747729968.jpg
[mission_executor-1] [INFO] [1785669773.106927344] [mission_executor]: [1/1] Capture OK
[mission_executor-1] [INFO] [1785669773.107005232] [mission_executor]: Mission completed: 1 waypointsr]: Passing new path to controller.
[component_container_isolated-1] [INFO] [1785669771.482705392] [controller_server]: Passing new path to controller.
[component_container_isolated-1] [INFO] [1785669772.482708368] [controller_server]: Passing new path to controller.
[component_container_isolated-1] [INFO] [1785669772.707230576] [controller_server]: Reached the goal!
[component_container_isolated-1] [INFO] [1785669772.745700592] [bt_navigator]: Goal succeeded


