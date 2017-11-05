#!/bin/bash

( roscore > ~/ROS-files/roslog.txt 2>&1 ) &
echo $! > ~/ROS-files/ros.pid
