# "Fetch & Carry" installation
===============

## Install Ubuntu
The Fetch & Carry scenario and its related component has been test under Ubuntu 10.04 LTS. If you do not have a Ubuntu distribution on you computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make. We use the email address to associate your commits with your GitHub account:

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS Electric
For the Fetch & Carry scenario the ROS distribution "Electric" is required. Other distributions have not been tested and therefore we do not recommend to use another distribution than "Electric". Follow the instructions on 

     http://www.ros.org/wiki/electric/Installation/Ubuntu

to install the basic ROS environment (make sure that you installed the ros-electric-desktop-full). Do not forget to update your .bashrc
  
Additionally to the ''ros-electric-desktop-full'' package, some few other packages are required to make the Fetch & Carry scenario compilable and executable. Therefore install the following packages:

     sudo apt-get install ros-electric-desktop-full ros-electric-arm-navigation ros-electric-pr2-controllers ros-electric-object-manipulation ros-electric-pr2-kinematics ros-electric-joystick-drivers ros-electric-laser-drivers ros-electric-cob-common  ros-electric-pr2-simulator ros-electric-openni-kinect ros-electric-pr2-apps ros-electric-bosch-drivers python-pygraphviz sudo apt-get install libmysqlclient-dev python-scipy libcap-dev bzr yaml-cpp

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://www.ros.org/wiki/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## youBot Software
### Install BROCRE
BROCRE uses robotpkg to download archives containing ROS stacks or packages. The BROCRE build system is based on rosmake to compile the downloaded archives and resolve build and system dependencies. Please follow the installation instructions on

     http://www.best-of-robotics.org/brocre/installation.html

to install BROCRE.

### Install youBot Realated Software 
BROCRE eases the installation of the required youBot packages. The BROCRE GUI can be start with the following command:

     cd your_home_dir/brocre/pkgtools/brocre_tools
     python brocre_package_install_gui.py

To install a specific package you need to select the respective package and click the "Install" button. For the Fetch & Carry scenario you need to install the following packages:

     youbot_driver
     b-it-bots_youbot-ros-pkg
     b-it-bots_youbot-manipulation
     
After the installation, the installed packages should be highlighted in green.

Checkout the research-camp-5 repository

     git clone git://github.com/b-it-bots/research-camp-5.git

### Compile Packages
Once the ROS package path is extended with our new directory, 

    echo "export ROS_PACKAGE_PATH=~/brics_software:\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc

the Fetch&Carry can be compiled with:

     rosmake raw_fetch_and_carry --rosdep-install


If no errors occur you can proceed to the next step. In case that there are errors, please write an email to jan.paulus@h-brs.de or frederik.hegger@h-brs.de including the error message and also the other debug outputs of the rosmake command.


### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line needs to be added to the .bashrc:

     echo "export ROBOT=youbot" >> ~/.bashrc
     source ~/.bashrc



#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. So please add the following line to your .bashrc:

     echo "export ROBOT_ENV=rc5" >> ~/.bashrc
     source ~/.bashrc



## Start the Fetch&Carry Scenario 
### In Simulation
    roslaunch raw_fetch_and_carry fetch_and_carry_demo_sim.launch

### At the Real Robot
    roslaunch raw_fetch_and_carry fetch_and_carry_demo.launch

### Execute the Scheduling Script
    rosrun raw_fetch_and_carry fetch_and_carry_demo.py