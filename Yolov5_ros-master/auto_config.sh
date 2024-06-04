#!/bin/bash
 
 # --- for every Terminal-tab
 source ~/.bashrc  # load ~/.bashrc setup
cd /home/yami/Yolov5_ros-master
# 预计需要三个终端
# 1. 启动yolo_v5 一般不寄
{
	gnome-terminal --tab "yolo_v5" -- "./shell_log.sh"


}&
# 2. 启动串口通信
sleep 1s
{
	gnome-terminal --tab "rosserials" -- "./shell_rosseroal.sh 2> output.txt"
}&
# 3. 挂一个终端始终jians
sleep 1s
{
	 gnome-terminal --tab "tail" -- "./tail.sh"
}
# Error opening serial: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'
# chmod: 无法访问 '/dev/ttyUSB0': 没有那个文件或目录
