#README
## To view the image , run this in the terminal :
``` 
rqt_image_view /yolo/visualization
```
##to check the manual controller alone : 
``` 
source ~/"competition auto "/competition_auto/autoPart/install/setup.bash ```
ros2 run auto_bot manual
```
## launch the 2 scripts :
```
cd ~/"competition auto "/competition_auto/autoPart
rm -rf build/ install/ log/
colcon build
source install/setup.bash
ros2 launch auto_bot compete_launch.py
```
## launch the bring up : 
```
ros2 launch competetion_bringup bringup.launch.py
```
## check the ultrasonic topic : 
```
  ros2 topic echo /ultrasonic_distance
```
