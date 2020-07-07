#! /bin/bash
mkdir -p test_ws/src
cp -r competition_modules test_ws/src

catkin_make -C test_ws/
