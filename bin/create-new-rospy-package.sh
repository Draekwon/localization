#!/bin/bash

echo "This script will create a new catkin package and eclipse project files for it."
echo ""
echo "Please enter your desired package name: "

read PACKAGE_NAME

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
cd ../catkin_ws_user/src/

if [ ! -z "$PACKAGE_NAME" ]; then
	catkin create pkg $PACKAGE_NAME --catkin-deps rospy std_msgs sensor_msgs

	cd $DIR
	./make-eclipse-projects.sh $PACKAGE_NAME
fi
