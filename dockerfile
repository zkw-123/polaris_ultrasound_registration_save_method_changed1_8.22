FROM ros:humble-ros-base

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    libusb-1.0-0-dev \
    udev \
    libv4l-dev \
    v4l-utils \
    ffmpeg \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgl1-mesa-glx \
    libopenexr-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libwebp-dev \
    # ROS包
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-ros \
    ros-humble-rclpy \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    python3-colcon-common-extensions \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 安装Python包 - 修复版本冲突
RUN pip3 install --no-cache-dir \
    numpy==1.24.4 \
    scipy==1.10.1 \
    matplotlib==3.7.2 \
    opencv-python==4.8.1.78 \
    scikit-surgerynditracker==1.0.1 \
    pyserial==3.5 \
    pillow==10.0.0 \
    scikit-surgerycore==0.7.2

# 创建工作空间
RUN mkdir -p /ros2_ws/src

# 创建数据和日志目录
RUN mkdir -p /ros2_ws/logs /ros2_ws/ultrasound_data

# 设置工作目录
WORKDIR /ros2_ws

# 复制入口脚本
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]