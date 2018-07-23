# MATRIX localization for autonomous model cars

This project implements the [MATRIX-Localization](http://page.mi.fu-berlin.de/rojas/2003/matrix.pdf) for autonomous model cars.

[>>> wiki][098be986]

## Short Instructions using the simulator

0. build the package
   - ``` cd localization/catkin_ws_user ```
   - ``` catkin build ```
1. start the simulator with the correct map and camera orientation/position
   - ``` roslaunch seat_car_gazebo sim.launch ```
2. undistort the camera image using the _fisheye_camera_matrix_ package
   - ``` source [localization/catkin_ws_user/]devel/setup.bash ```
   - ``` roslaunch fisheye_camera_matrix undistorted_image_publisher.launch ```
3. start the MATRIX localization
   - ``` source [localization/catkin_ws_user/]devel/setup.bash ```
   - ``` rosrun local_localization local_localization_cameraOverlay ```


  [098be986]: https://github.com/Draekwon/localization/wiki "https://github.com/Draekwon/localization/wiki"
