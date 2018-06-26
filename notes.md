http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf

- p.2: 3 methods of localizing: weighted mean, best particle, robust mean
- p.4: deal with "drift" using noise (?!)
- p.4: find out standard deviation using experimental data
- p.6: find a good resampling algorithm (perhaps in appendix D)

main points:
   * robust mean has best accuracy but is expensive (how? would that not just be O(2n)=O(n) ?)
   * find a good and cheap noise algorithm. the algorithm used here was a normal distribution using fancy calculated means and standard deviation. it also used experimental data to correct odometry error
   * the noise algorithm could (and probably should) deal with odometry error
   * a good and cheap resampling algorithm will be necessary

      * maybe combine resampling and calculating mean position?

--------------------------------------

use Qt with ROS:
https://github.com/Levi-Armstrong/ros_qtc_plugins/wiki/Setup-Qt-Creator-for-ROS

--------------------------------------

use opencv ORB/SURF algorithm combined with opencv feature matcher for comparing virtual images and the real one maybe?
https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html#matcher
https://docs.opencv.org/3.1.0/d5/d6f/tutorial_feature_flann_matcher.html

--------------------------------------

http://wiki.ros.org/ROS/Tutorials/CreatingPackage
http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingPublisherSubscriber#Running_the_nodes

--------------------------------------

I used this tutorial for now:
https://github.com/AutoModelCar/seat_car_simulator
which makes everything said above useless/pointless

--------------------------------------

in simulator the camera is 40.5 cm above the board, so ~43-44 cm above ground approximately

--------------------------------------

https://www.ri.cmu.edu/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf

--------------------------------------
the same as above?
https://rse-lab.cs.washington.edu/abstracts/sampling-aaai-99.abstract.html

--------------------------------------

http://robots.stanford.edu/papers/thrun.robust-mcl.pdf

--------------------------------------
--------------------------------------

http://page.mi.fu-berlin.de/rojas/2003/matrix.pdf

--------------------------------------
--------------------------------------

http://legacydirs.umiacs.umd.edu/~fer/cmsc828/classes/fox.mcmc-book.pdf

--------------------------------------

https://ac.els-cdn.com/S1474667016350595/1-s2.0-S1474667016350595-main.pdf?_tid=f05d34be-8bba-4082-82b9-bdf81801e5e2&acdnat=1525786050_639e7c3ce59d8b4aea67d9048fa09d25
