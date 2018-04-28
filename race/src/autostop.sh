#!/bin/bash

handler () {
    sleep 1
    roscore &
    pid1=$!
    sleep 1
    rosrun rosserial_python serial_node.py /dev/ttyACM0 &
    pid2=$!
    sleep 5
    rostopic pub /drive_pwm race/drive_values "pwm_drive: 9831
pwm_angle: 9831" -1
    sleep 1
    kill -2 $pid2
    sleep 1
    kill -2 $pid1
    exit 0
}

trap "handler" SIGHUP
while [ 1 ]; do :; done
