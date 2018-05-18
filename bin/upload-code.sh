#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
cd ..
DIR="$( pwd )"

echo "Uploading to catkin_ws_user to 192.168.43.103 ..."
scp -v -r $DIR/catkin_ws_user/src root@192.168.43.103:/root/catkin_ws_user/

echo "Finished!"
