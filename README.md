# NAV100使用流程

## 创建ROS工作空间
- `mkdir -p ~/catkin_ws/src`
- `cd ~/catkin_ws/src`

## 在src文件夹下git down本项目
- `git clone https://github.com/curvin/imu_publish.git`

## 查看串口，授予权限
- `ls -l /dev/ttyUSB*`
- `sudo chmod 666 /dev/ttyUSB0` ***其中USB0为上一步扫描出的串口号，若不为USB0，请到`imu_data.cpp`中修改***

## 编译工程 
- `cd ..`
- `catkin_make`

## 运行
- 新建终端运行`roscore`
- 在工程目录下
  - `source devel/setup.bash`
  - `rosrun imu_publish imu_data`
