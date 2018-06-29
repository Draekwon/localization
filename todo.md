**26.06.18**
* ~~rename my package~~ _done_
* ~~remove namespaces~~ _done_
* ~~make image locations ros-dependent~~ _done_
  - for this, set environment var $ROS_PACKAGE_PATH in the eclipse run config to whatever 'echo $ROS_PACKAGE_PATH' returns (after sourcing ofc) **-> wiki**


* ~~integrate camera undistortion and normalization into my package~~
   - __tried doing this. Undistortion using opencv undistort or the imgproc package from ros does not work in the simulator.__
     - -> I will probably skip this one
     - (maybe make a roslaunch file for this)
* change how I treat the camera overlay -> right now I scale the underlying image. I should scale the camera image to the appropriate size instead
   - this might gain some performance as well
   - make sure I can set the scale factor as parameter in objects constructor
   - the scale factor should probably be delivered by the global localization (nyi)


* start the wiki
  - push my changes to the appropriate branch of the seat_car_simulator and explain them in the wiki
  - create doxygen (code documentation) to support the wiki


**29.06.18**

* ~~replace edge detection with thresholding~~
   - ~~use eroding and dilating for noise reduction~~
   - ~~do that for the force and distance fields as well~~
   _done_ more or less
* ~~change how I treat the camera overlay -> right now I scale the underlying image. I should scale the camera image to the appropriate size instead~~
   - ~~this might gain some performance as well~~ _-> find this out_
   - ~~make sure I can set the scale factor as parameter in objects constructor~~ _done_
   - the scale factor should probably be delivered by the global localization (nyi) **-> wiki**


* start the wiki
  - push my changes to the appropriate branch of the seat_car_simulator and explain them in the wiki
  - create doxygen (code documentation) to support the wiki - _mostly done_
