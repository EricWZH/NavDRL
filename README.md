# Deep Reinforcement Learning robot navigation

## Install

* Ubuntu 20.04操作系统
* ros2 foxy

* 安装依赖：

```
echo "deb [trusted=yes] https://nginx-1253383465.cos.ap-guangzhou.myqcloud.com/apt/focal/$(dpkg --print-architecture) ./"| sudo tee /etc/apt/sources.list.d/robot.list
sudo apt update
sudo apt install python3-pip svar svar-zmq svar-messenger-ros2 libyaml-dev libyaml-cpp-dev zmq-broker zbus-cpp svar-zbus svar-yaml zbus2ros rosbag2cvte zbusbag2cvte python3-colcon-common-extensions libceres-dev libopencv-dev libgoogle-glog-dev
```

TODO: 补充python依赖说明

## Usage

训练：

```
cd src 
python3 train_velodyne_td3.py
```

测试:

```
cd src 
python3 test_velodyne_td3.py
```

可视化

```
rviz2 -d vis.rviz
```
