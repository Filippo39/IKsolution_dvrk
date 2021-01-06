# IKsolution_dvrk
# you have to indicate in the field kinematic, in console file, the file "psm-large-needle-driver-tip.json"

# There are two pdf files in the folder explaining how to install and approach the dvrk

# open terminal and run 
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/YOUR-USER/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json

# in another terminal run
rosrun dvrk_python IKScript.py 
