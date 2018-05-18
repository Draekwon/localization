#!/bin/bash

echo "Stopping ROS ..."
ssh root@192.168.43.103 'pkill -f ros'

sleep 3

echo "Running autostart.sh ..."
ssh root@192.168.43.103 './autostart.sh' &

echo "Finished!"
