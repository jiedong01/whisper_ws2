#   环境配置参考：https://github.com/mgonzs13/whisper_ros
#   请将所有ros2_ws改成whisper_ws2

### 本代码属于两台电脑配合版：
#### 电脑1：（实现麦克风听取语音）启用whisper_ws2   
`<cd whisper_ws2>`
`<colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=release -DGGML_CUDA=ON>`
`<source install/setup.bash>`
#### launch: 
`<ros2 launch xyz_bringup2 xyz_launch.py>`

