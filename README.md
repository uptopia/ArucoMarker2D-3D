# ArucoMarker2D-3D

##Dependencies
realsense-ros
cv2.aruco
cv_bridge
pcl

```
cd ~
mkdir ~/ArucoMarker2D-3D/src && cd ~/ArucoMarker2D-3D/src
git clone https://github.com/uptopia/ArucoMarker2D-3D.git
cd ..
catkin_make
. devep/setup.bash
```

## save_img.py
```
# roscore

# cd ~/realsense_ws
# . devel/setup.bash
# roslaunch realsense2_camera rs_camera.launch

# python aruco_ros.py
```

## aruco_ros.py
```
# roscore

# cd ~/realsense_ws
# . devel/setup.bash
# roslaunch realsense2_camera rs_camera.launch
# roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud

# rostopic list

# cd ~/ArucoMarker2D-3D/src/detect_aruco/src
# python aruco_ros.py
```

## aruco_cloud.cpp
```
# roscore

執行上方aruco_ros.py區塊的程式碼
發布ROS topic "/aruco_corners"

rosrun aruco_cloud aruco_cloud
```