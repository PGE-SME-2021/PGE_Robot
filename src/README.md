<!--
*** Merci d'avoir consulté le modèle Best-README. Si vous avez une suggestion
*** pour ameliorer notre projet portant sur un robot mobile.
*** Cette partie vous parle de la cartagraphie, navigation et détection des obstacles avec le RPLIDAR et ROS sous RPI4.
*** Le groupe LIDAR vous souhaite une bonne lecture :D
-->

## Authors 👤

**NIANE ABDOULAYE** 

**DIOUME ABDOULAYE** 

**HOURI MERYEM**

**AMADOU BOUBACAR**

**DOUKI THIZIRI**

**CAMARA CHEIKH**

***
# 📎 PGE_MASTER_2_SME_UPS_RPLIDAR_GROUPS :  Creating a README file for GitHub

![left 100%](images/rpilidar.jpg?raw=true)

_`Project start date 22/10/2021`_

<!-- LOGO DU PROJET -->
<br />
<p align="center">

<!-- TABLE DES MATIÈRES -->
<details open="open">
  <summary><h2 style="display: inline-block">Table des matières</h2></summary>
  <ol>
    <li><a href="#a-propos-du-projet">About the project</a></li>
    <li>
      <a href="#connect-your-rplidar">Connect your RPLIDAR</a>
      <ul>
        <li><a href="#Prerequisites">Prerequisites</a></li>
      </ul>
    </li>
    <li><a href="#Workspace-configuration-and-installation-of-RPLIDAR-ROS-packages">Workspace configuration and installation of RPLIDAR ROS packages</ a></li>
    <li><a href="#launch-of-the-project">Launch of the project</a>
      <ul>
        <li><a href="#Run-the-RPLIDAR-launch-file">Run the RPLIDAR launch file</a></li>
        <li><a href="#rviz">rviz</a></li>
      </ul></li>
    <li><a href="#Creating-a-map-using-the-Hector-SLAM-ROS-package">Creating a map using the Hector-SLAM ROS package</a></li>
    <li><a href="#Save-the-Map">Save the Map</a>
    <ul>
        <li><a href="#Convert-your-map-to-png-format"> Convert your map to png format</a></li>
        <li><a href="#Using-the-map_server-method-to-register-the-map">Using the map_server method to register the map</a></li>
      </ul>
    <li><a href="#edit-map">Edit Map</a></li>
    <li><a href="#Configuring-the-ROS-navigation-stack-on-a-robot">Configuring the ROS navigation stack on a robot</a>
      <ul>
        <li><a href="#Prerequisites-for-the-navigation">Prerequisites for the navigation</a></li>
        <li><a href="#Install-the-ROS-Navigation-Stack">Install the ROS Navigation Stack</a></li>
      </ul></li>
  <li><a href="#Software-and-hardware-used">Software and hardware used</a></li>
    <li><a href="#customize-configuration">Customize configuration</a></li>
  </ol>
</details>

<!-- A PROPOS DU PROJET -->
## A propos du projet

Un robot mobile autonome est un type de robot capable de circuler sans l’intervention de
l’homme, en toute autonomie, donc capable de se déplacer dans son environnement de
manière indépendante.
Cette technologie diffère de son prédécesseur, le véhicule à guidage autonome qui s'appuie
sur des pistes ou des chemins prédéfinis et nécessite souvent la surveillance d'un opérateur.
Notre robot utilise un ensemble sophistiqué de capteurs pour planifier la trajectoire, afin
d'interpréter et de naviguer dans son environnement sans être alimenté par un câble.
Un capteur indispensable est le lidar représentant les yeux du robot.
Dans cette documentation, nous vous montrons l’ensemble des paramétrages nécessaires
pour faire fonctionné le RPlidar A1

## Connect your RPLIDAR
### Prerequisites
#### Open a terminal and verify permissions
```
ls -l /dev | grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
```

#### Update the list of packages
```
sudo apt-get update
```
#### Installation of dependencies
```
sudo apt-get install gedit
```

## Workspace configuration and installation of RPLIDAR ROS packages
#### Installation of dependencies
```
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git
```
#### Creation de l'espace de travail Catkin
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```
#### Modify the file .bashrc
```
gedit ~/.bashrc
```
#### Put these two lines at the end of the .bashrc file
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
#### Save the file.

#### Type this command on the current terminal
```
source ~/catkin_ws/devel/setup.bash
```
#### Go to the src folder
```
cd src
```
#### Clone the project into your src folder
```
lien clone à mettre ici
```
#### Open this file
```
cd ~/catkin_ws/src/rplidar_ros/src
```
#### Change the permissions
```
sudo chmod +x node.cpp
```
#### Go to the workspace
```
cd ~/catkin_ws/
```
#### Compile the project
```
catkin_make
```

## Launch of the project
### Run the RPLIDAR launch file
```
roslaunch rplidar_ros rplidar.launch
```
#### Open a new terminal to see the list of topics
```
rostopic list
```
#### Open a new terminal
```
rostopic echo /scan
```
<p align="center" > Warning! keep the terminal open for the next section <br /> </p>

### rviz
#### Open a new terminal and run rviz
```
rviz
```
#### Once the rviz window appears
<p>
Put 
<br />
</p>
"
Fixed Frame = laser
"
<p>
Click on the bottom left of the window on add and select LaserScan then click on OK 
<br/>
</p>

<p>
Put 
<br/>
</p>
"
LaserScan = /scan
"
<p>
You can increase the size for a better visualization of the data
<br/>
</p>
"
size(m) = 0.04
"
<p>
To close the terminal press CTRL+C
<br/>
</p>



## Creating a map using the Hector-SLAM ROS package
#### Installing Qt4
```
sudo apt-get install qt4-qmake qt4-dev-tools
```
#### Update the packages
```
sudo apt-get update
```
#### Open this file
```
cd ~/catkin_ws/src/hector_slam
```
#### Open a new terminal 
```
cd ~/catkin_ws/
```
#### Compile the package
```
catkin_make
```
<h align="center">If you see an error message of type</h>
<p>
Project ‘cv_bridge’ specifies ‘/usr/include/opencv’ as an include dir, which is not found. It does neither exist as an absolute directory nor in....
<br/>
</p>

#### Enter 
```
cd /usr/include
sudo ln -s opencv4/ opencv
```
#### Compile the package
```
cd ~/catkin_ws/
catkin_make
```
#### Reboot your computer
```
sudo shutdown -h now
```

#### Launch of the mapping
#### Open a new terminal and start RPLIDAR
```
cd ~/catkin_ws/
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch
```
<h align="center">Now that the LIDAR is running, let's start the mapping</h>

#### Open a new terminal
```
roslaunch hector_slam_launch tutorial.launch
```
<h align="center">You can move the LIDAR very slowly around the room to get a good map</h>

## Save the Map
### Using the map_server method to register the map
#### Installing map_server
```
sudo apt-get install ros-kinetic-map-server
```
#### Create the folder for the map
```
mkdir ~/catkin_ws/maps
```
<h align="center">Start the mapping process (see the section Starting the mapping above)</h>
#### Open a new terminal
```
cd ~/catkin_ws/maps
rosrun map_server map_saver -f my_map_rplidar_ros
```
<h align="center">The map will be saved in the ~/catkin_ws/maps directory in yaml and pgm format.
<br />
<h>Type CTRL+C to close the terminal</h>

<h align="center">Load the registered map</h>

#### Open a new terminal
```
cd ~/catkin_ws/maps
roscore
```
#### Load the map with the command
```
rosrun map_server map_server my_map_rplidar_ros.yaml
```
#### Open rviz in a new terminal
```
rviz
```
<h align="center"> Press the add button at the bottom left and add Map</h>
<p>
"Set Topic = /map"
<br/>
</p>

<h>You should see your map on rviz</h>

### Convert your map to png format
#### Install the package
```
sudo apt-get install imagemagick
convert my_map_rplidar_ros.pgm my_map_rplidar_ros.png
```

## Edit map
#### Install gimp
```
sudo apt-get update
sudo apt-get install gimp
```
#### Launch gimp
```
gimp
```
<p>
To load the image, go to File -> Open , then locate your image
<br/>
</p>

## Configuring the ROS navigation stack on a robot
### Prerequisites for the navigation
1. Transform Configuration
2. LIDAR Information
3. Odometry Information
4. Data from wheel encoders
### Install the ROS Navigation Stack
```
sudo apt-get install ros-kinetic-navigation
```
#### Open a terminal window, and type
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps navstack_pub
```
#### Open a new terminal window and launch the launch file
```
roslaunch navstack_pub rplidar_navigation.launch
```
***

### Software and hardware used

| Langages           | Applications    | Materiels    |
| :-------------:     |:--------------:    |:--------------: |
| C++, python    | Visual Studio Code, ROS | RPI4
| Git/GitHub, Gazebo  | OS ubuntu 14.04 et 18.04  | RPLidar et les materiels de la salle H0 de UPS |


## Customize configuration
#### See [Configuration Reference]
```
https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/
```

#### Merci à tous

***

&hearts;&nbsp;&nbsp;&nbsp;&nbsp;ROBOTS

<span style="font-family:Papyrus; font-size:4em;">FOR ROBOTICS FANS ESPECIALLY RPLIDAR </span>

<!--[This is an image](https://myoctocat.com/assets/images/base-octocat.svg)-->

![left 10%](images/fin.jpeg?raw=true)

# PGE_MASTER_2_SME_UPS_2022_RPLIDAR_GROUPS FOR THE COMPANY OpenIndus











