#!/usr/bin/bash
#>>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
 __conda_setup="$('/home/yami/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
	    eval "$__conda_setup"
   else
	        if [ -f "/home/yami/anaconda3/etc/profile.d/conda.sh" ]; then
			        . "/home/yami/anaconda3/etc/profile.d/conda.sh"
				    else
					            export PATH="/home/yami/anaconda3/bin:$PATH"
						        fi
fi
unset __conda_setup
# <<< conda initialize <<<

conda activate yolo
# cd /home/yami/Yolov5_ros-master
source ./devel/setup.bash
roslaunch yolov5_ros 5.launch
exec bash

 
