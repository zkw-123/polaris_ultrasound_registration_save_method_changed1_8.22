ROS 2 Polaris Ultrasound 使用指南
容器操作
构建并启动容器
bash# 构建并启动容器（后台运行）
sudo docker compose up -d

# 查看容器运行状态
sudo docker ps

# 查看容器日志
sudo docker logs polaris_ultrasound
进入容器
bash# 进入容器交互式终端
sudo docker exec -it polaris_ultrasound bash
停止和移除容器
bash# 停止容器
sudo docker compose stop

# 停止并移除容器
sudo docker compose down

# 如果需要重建容器（修改Dockerfile后）
sudo docker compose up -d --build
容器内操作
构建ROS 2包
bash# 进入容器后，首先构建ROS 2包
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
source /ros2_ws/install/setup.bash
运行Polaris跟踪节点
bash# 运行Polaris跟踪节点，使用ROM文件
ros2 run polaris_ultrasound polaris_reader --rom_path /opt/ndi/rom_files/your_rom_file.rom

# 如果有多个ROM文件，可以多次使用--rom_path参数
ros2 run polaris_ultrasound polaris_reader --rom_path /opt/ndi/rom_files/tool1.rom --rom_path /opt/ndi/rom_files/tool2.rom

# 保存跟踪数据
ros2 run polaris_ultrasound polaris_reader --rom_path /opt/ndi/rom_files/tool1.rom --save_flag --save_path /ros2_ws/logs
运行超声成像节点
bash# 运行超声成像节点，使用ROI掩码
ros2 run polaris_ultrasound ultrasound_reader --mask_path /ros2_ws/masks/ari85_pixl_map.npy --device_id 0 --display

# 保存超声图像
ros2 run polaris_ultrasound ultrasound_reader --mask_path /ros2_ws/masks/ari85_pixl_map.npy --device_id 0 --save_path /ros2_ws/ultrasound_data --save_images

# 调整采样率
ros2 run polaris_ultrasound ultrasound_reader --mask_path /ros2_ws/masks/ari85_pixl_map.npy --device_id 0 --sample_rate 60
运行校准数据记录器
bash# 先启动Polaris节点
ros2 run polaris_ultrasound polaris_reader --rom_path /opt/ndi/rom_files/your_rom_file.rom

# 然后在另一个终端启动校准记录器
ros2 run polaris_ultrasound calibration_recorder --ros-args \
  -p device_id:=0 \
  -p save_path:=/ros2_ws/ultrasound_data \
  -p frame_rate:=30.0 \
  -p sync_threshold:=0.05
检查和监控数据
bash# 列出所有可用的ROS 2话题
ros2 topic list

# 监控Polaris跟踪数据
ros2 topic echo /ndi_transforms

# 查看节点状态
ros2 node list
ros2 node info /ros2_polaris_reader
数据管理
从容器复制数据到主机
在主机终端（不是容器内）执行：
bash# 复制日志数据到主机当前目录
sudo docker cp polaris_ultrasound:/ros2_ws/logs ./

# 复制超声图像数据到主机当前目录
sudo docker cp polaris_ultrasound:/ros2_ws/ultrasound_data ./
常见问题排查
设备访问问题
bash# 检查设备是否正确连接
ls -l /dev/ttyUSB*
ls -l /dev/video*

# 检查设备权限
sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/video0
查看错误日志
bash# 在容器内查看ROS日志
ros2 log list
ros2 log info
重启节点或重新构建
bash# 重启节点
# 按Ctrl+C终止节点，然后重新运行

# 修改代码后重新构建
cd /ros2_ws
colcon build --symlink-install
source /ros2_ws/install/setup.bash