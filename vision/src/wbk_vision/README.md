# wbk_vision (ROS2 Foxy)

Minimal USB webcam publisher for Jetson Nano (Ubuntu 18.04, ROS2 Foxy).

## Build and source
```
cd ~/ros2_ws  # your workspace containing src/wbk_vision
colcon build --symlink-install
source install/setup.bash
```

## Run the node directly
```
ros2 run wbk_vision usb_camera_node \
  --ros-args -p device_id:=0 -p width:=640 -p height:=480 -p fps:=30 -p frame_id:=camera
```

## Launch file
```
ros2 launch wbk_vision usb_camera.launch.py \
  device_id:=0 width:=640 height:=480 fps:=30 frame_id:=camera
```

## Verify topics
```
ros2 topic list
ros2 topic echo /camera/image_raw
ros2 topic echo /camera/camera_info
```

## Run camera calibration
```
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.024 \
  image:=/camera/image_raw camera:=/camera
```



