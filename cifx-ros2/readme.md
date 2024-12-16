## ros2工作空间创建及功能包编译

`mkdir -p ros_ws && cd ros_ws`

`mkdir -p src && cd src`

赋值ethercat_master进入当前文件夹

回到src

`cd ..`

编译

`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

`source ./install/local_setup.zsh`

运行可执行文件,注意替换路径，测试elmo程序是否成功运行

`sudo zsh -c 'source /home/xxx/ros_ws/install/setup.zsh; install/ethercat_device_configurator/lib/ethercat_device_configurator/standalone src/libertech_ros2_humanoid_control/ethercat_master/ethercat_device_configurator/example_config/setup.yaml'`








