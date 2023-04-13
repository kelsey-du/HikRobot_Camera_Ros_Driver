# HIKROBOT-MVS-CAMERA-ROS
The ros driver package of Hikvision Industrial Camera SDK. Support configuration parameters, 
the parameters have been optimized, and the photos have been transcoded from mono12 to mono16 format.

# Install
### SDK
官网驱动，机器视觉工业相机客户端MVS V2.1.2（Linux）
https://www.hikrobotics.com/cn/machinevision/service/download?module=0

下载x86_64，deb版本

运行上位机：
```
cd /opt/MVS/bin
./MVS.sh
```

### lsusb:
```
Bus 002 Device 010: ID 2bdf:0001  
Bus 002 Device 009: ID 2bdf:0001
```

# launch run
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
```

