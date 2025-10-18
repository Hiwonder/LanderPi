
#关闭所有ros节点
~/.stop_ros.sh

cd ~/ros2_ws 
#colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release --symlink-install
#单独编译某个包
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select xxx



#######################################################
#线速度校准(ROSMentor_Mecanum, MentorPi_Acker)
ros2 launch calibration linear_calib.launch.py

#角速度校准(ROSMentor_Mecanum)
ros2 launch calibration angular_calib.launch.py

#imu校准
ros2 launch ros_robot_controller ros_robot_controller.launch.py
ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml

#查看imu校准效果
ros2 launch peripherals imu_view.launch.py

#深度摄像头RGB图像可视化()
ros2 launch peripherals depth_camera.launch.py
rviz2

#单目摄像头可视化
ros2 launch peripherals usb_cam.launch.py
rviz2

#雷达数据可视化
ros2 launch peripherals lidar_view.launch.py
#######################################################



#######################################################
#雷达功能
ros2 launch app lidar_node.launch.py debug:=true
#雷达避障(ROSMentor_Mecanum, MentorPi_Acker)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"

#雷达跟随(ROSMentor_Mecanum, MentorPi_Acker)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 2}"

#雷达警卫(ROSMentor_Mecanum)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 3}"

#巡线
ros2 launch app line_following_node.launch.py debug:=true
ros2 service call /line_following/enter std_srvs/srv/Trigger {}
#鼠标左键点击画面取色
ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"

#目标跟踪
ros2 launch app object_tracking_node.launch.py debug:=true
ros2 service call /object_tracking/enter std_srvs/srv/Trigger {}
#鼠标左键点击画面取色
ros2 service call /object_tracking/set_running std_srvs/srv/SetBool "{data: True}"


# hand_gesture
ros2 launch app hand_gesture_node.launch.py debug:=true
ros2 service call /hand_gesture/enter std_srvs/srv/Trigger {}
ros2 service call /hand_gesture/set_running std_srvs/srv/SetBool "{data: True}"
#######################################################



#######################################################
ros2 launch peripherals depth_camera.launch.py

#二维码生成
cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_creater.py
#二维码检测
cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_detecter.py

#人脸检测
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_detect.py

#人脸网格
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py

#手关键点检测
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand.py

#肢体关键点检测
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 pose.py

#背景分割
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 self_segmentation.py

#整体检测
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 holistic.py

#3D物体检测
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 objectron.py

#指尖轨迹
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand_gesture.py

#颜色识别
cd ~/ros2_ws/src/example/example/color_detect && python3 color_detect_demo.py
#######################################################


##################### MediaPipe 2 ####################
# 肢体体感控制
ros2 launch example body_control.launch.py

# 人体融合RGB控制
ros2 launch example body_and_rgb_control.launch.py

# 人体姿态检测（跌倒检测）
ros2 launch example fall_down_detect.launch.py 


#无人驾驶
ros2 launch example self_driving.launch.py
#######################################################



#######################################################
#2D建图
ros2 launch slam slam.launch.py

#rviz查看建图效果
ros2 launch slam rviz_slam.launch.py

#键盘控制(可选)
ros2 launch peripherals teleop_key_control.launch.py

#保存地图
#/home/ubuntu/ros2_ws/src/slam/maps/map_01.yaml
cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "map_01" --ros-args -p map_subscribe_transient_local:=true

#cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "保存名称" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/robot_1
#######################################################



#######################################################
#2D导航
##rviz发布导航目标
ros2 launch navigation rviz_navigation.launch.py

ros2 launch navigation navigation.launch.py map:=map_01
#######################################################



#######################################################
#3D建图(depth_cam)
ros2 launch slam rtabmap_slam.launch.py

#rviz查看建图效果
ros2 launch slam rviz_rtabmap.launch.py

#键盘控制(可选)
ros2 launch peripherals teleop_key_control.launch.py
#######################################################



#######################################################
#3D导航(depth_cam)
ros2 launch navigation rtabmap_navigation.launch.py

#rviz发布导航目标
ros2 launch navigation rviz_rtabmap_navigation.launch.py
#######################################################



####### simulations #######
#urdf可视化
ros2 launch mentorpi_description display.launch.py
#######################################################


####### 2D Vision ########
#手部跟随
ros2 launch example hand_track_node.launch.py

#色块分拣
ros2 launch example color_sorting_node.launch.py debug:=true

#色块追踪
ros2 launch example color_track_node.launch.py

#巡线清障
ros2 launch example line_follow_clean_node.launch.py debug:=true

# 垃圾分类
ros2 launch example garbage_classification.launch.py debug:=true

# 定点导航
## 夹取测试
ros2 launch example automatic_pick.launch.py debug:=true
## 开启定点导航
ros2 launch example navigation_transport.launch.py map:=map_01
########################


########使用上位机software在关闭APP自启范围的情况下需要先打开摄像头服务
ros2 launch peripherals depth_camera.launch.py

#lab_tool
python3 ~/software/lab_tool/main.py

#servo_tool（调整pwm舵机偏差）
python3 ~/software/servo_tool/main.py
######################


### 语音控制玩法
# 语音控制移动
ros2 launch xf_mic_asr_offline voice_control_move.launch.py

# 语音控制颜色追踪
ros2 launch xf_mic_asr_offline voice_control_color_track.launch.py

# 语音控制颜色分拣
ros2 launch xf_mic_asr_offline voice_control_color_sorting.launch.py

# 语音控制颜色检测
ros2 launch xf_mic_asr_offline voice_control_color_detect.launch.py

# 语音控制垃圾分类
ros2 launch xf_mic_asr_offline voice_control_garbage_classification.launch.py

# 语音控制导航
ros2 launch xf_mic_asr_offline voice_control_navigation.launch.py

# 语音控制导航搬运
ros2 launch xf_mic_asr_offline voice_control_navigation_transport.launch.py
######################

#######large_models_examples#######
#前后左右，横向移动，然后漂移
ros2 launch large_models_examples llm_control_move.launch.py

#追踪红色物体
#ros2 topic pub --once /vocal_detect/asr_result std_msgs/msg/String '{data: "追踪红色物体"}'
ros2 topic pub --once /vocal_detect/asr_result std_msgs/msg/String '{data: "夹取红色的方块,放到红色的盒子里"}'
ros2 topic pub --once /vocal_detect/asr_result std_msgs/msg/String '{data: "你看到了什么"}'
ros2 topic pub --once /vocal_detect/asr_result std_msgs/msg/String '{data: "追踪红色的小球"}'
ros2 topic pub --once /vocal_detect/asr_result std_msgs/msg/String '{data: "去厨房看看大门有没有关，然后回来告诉我"}'
ros2 launch large_models_examples llm_color_track.launch.py

#沿着黑线走，遇到障碍就停下
ros2 launch large_models_examples llm_visual_patrol.launch.py

#描述下你看到了什么
ros2 launch large_models_examples vllm_with_camera.launch.py

#追踪前面的小球
ros2 launch large_models_examples vllm_track.launch.py

#去厨房看看大门有没有关，然后回来告诉我
ros2 launch large_models_examples vllm_navigation.launch.py map:=map_01

# 将红色方块放到红色的盒子里
ros2 launch large_models_examples vllm_navigation_transport.launch.py map:=map_01
########################
