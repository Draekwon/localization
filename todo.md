**26.06.18**
* ~~rename my package~~ _done_
* ~~remove namespaces~~ _done_
* ~~make image locations ros-dependent~~ _done_
  - for this, set environment var $ROS_PACKAGE_PATH in the eclipse run config to whatever 'echo $ROS_PACKAGE_PATH' returns (after sourcing ofc) **-> wiki**


* integrate camera undistortion and normalization into my package
   - __undistortion using opencv undistort or the imgproc package from ros does not work in the simulator. which is bs...__
   - -> I will probably skip this one
   - (maybe make a roslaunch file for this
   - probably use a different algorithm for this than what is used until now)
* change how I treat the camera overlay -> right now I scale the underlying image. I should scale the camera image to the appropriate size instead
   - this might gain some performance as well
   - make sure I can set the scale factor as parameter in objects constructor
   - the scale factor should probably be delivered by the global localization (nyi)


* start the wiki
  - push my changes to the appropriate branch of the seat_car_simulator and explain them in the wiki
  - create doxygen (code documentation) to support the wiki
