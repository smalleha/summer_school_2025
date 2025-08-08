#! /bin/bash

### BEGIN INIT
gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/limo/limo_robot/devel/setup.bash;roslaunch limo_multi navigation.launch"
sleep 10

wait
exit 0
