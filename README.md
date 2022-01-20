# PGE_SME_Robot_Mobile_Autonome
Welcome to our project

## Usefull Links
[Drive folder](https://drive.google.com/drive/folders/1Ol4AokiO4WkVhKb9K3ikbNwlIXcn7H8S)
[ROS](https://www.ros.org/blog/getting-started/)
[PyQt](https://pypi.org/project/PyQt5/)
[Project Wikis](https://github.com/PGE-SME-2021/GUI_Robot/wiki)

## Usefull aliases

If you want to use them you can add them to your bashrc file with the next script on terminal
```bash
echo 'alias ros-env="export ROS_MASTER_URI=http://10.42.0.1:11311 && export ROS_IP=`hostname -I`"' >> ~/.bashrc
echo 'alias ros-comp="cd ~/catkin_ws/ && catkin_make && source devel    /setup.bash"' >> ~/.bashrc
echo 'alias ros-start="source /opt/ros/noetic/setup.bash && cd ~/GUI_Robot/ && source devel/setup.bash"' >> ~/.bashrc
echo 'alias ros-print="printenv | grep ROS"' >> ~/.bashrc
source ~/.bashrc
```
