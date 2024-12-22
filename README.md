#   环境配置参考：https://github.com/mgonzs13/whisper_ros
#   请将所有ros2_ws改成你的workspsace

### 本代码属于两台电脑配合版：分别放在两个branch里面
#### 电脑1：（实现麦克风听取语音）启用whisper_ws2   
`<cd whisper_ws2>`

`<colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=release -DGGML_CUDA=ON>`

`<source install/setup.bash>`

#### launch: 

`<ros2 launch xyz_bringup2 xyz_launch.py>`

#### 电脑2：（将文本变成速度指令，控制机器人移动）启用whisper_ws1
`<cd whisper_ws1>`

 `< colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=release -DGGML_CUDA=ON>`
 
 `<source install/setup.bash>`

 `< ros2 topic echo /cmd_vel>`    #查看控制指令 cmd_vel，确保机器人根据物体识别做出相应反应

![image](https://github.com/user-attachments/assets/ef623e59-7dc4-41d2-b91e-eda8589fd6e4)


 #### 另外开一个terminal：
 
`<rqt_graph>`

![image](https://github.com/user-attachments/assets/6aa6db07-66ba-42f6-b839-b0b3f9ae0f09)


                                                                   ---------lenovo不能启用麦克风版
  ##### 如果你可以循环听取语音，看到匹配速度指令的发出，那么就证明你运行成功了
