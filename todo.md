* rename my package
* integrate camera undistortion and normalization into my package
 * maybe make a roslaunch file for this
 * probably use a different algorithm for this than what is used until now
* change how I treat the camera overlay -> right now I scale the underlying image. I should scale the camera image to the appropriate size instead
 * this might gain some performance as well
 * make sure I can give the object the scale factor as parameter
 * the scale factor should be delivered by the global localization (nyi)
* start the wiki
 * push my changes to the appropriate branch of the seat_car_simulator
