#!/bin/bash 
specific_output1="Error opening serial: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'"  
specific_output2="chmod: 无法访问 '/dev/ttyUSB0': 没有那个文件或目录"  
    
script_to_execute="/home/yami/Yolov5_ros-master/shell_rosseroal.sh"  
  
echo "Start_Detection"  
tail -f /home/yami/Yolov5_ros-master/output.log | grep -E "$specific_output1|$specific_output2" | while read line  
do  
    echo "检测到特定输出： $line"  
     "$script_to_execute"  
done



