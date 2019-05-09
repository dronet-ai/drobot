## Drobot : Fast and Furious Computation for Drone Navigation

**Authors:** [Zeryab Moussaoui](https://fr.linkedin.com/in/zeryab-moussaoui-9a728029), [Yacine Ben Ameur]( https://www.linkedin.com/in/yacine-ben-ameur-b15aa0165) , [Houssem Meghnouj](https://www.linkedin.com/in/houssem-meghnoudj-229735148), [Nabil Tchoulak](https://www.linkedin.com/in/mohamed-nabil-tchoulak-b43670167) , [Merouane Guettache](https://www.linkedin.com/in/merouane-guettache-261560181/)

**23 Mars 2019**: Submit the V1 code according to Alpha-Pilot deadline.

**02 Avril 2019**: Resolve all submission issues.

**04 Avril 2019**: Resolve remaining issue.

**10 May 2019** : Adding technical report.

The aim of this code is to complete the selection of the Lockheed Martin's Drone Race "Alpha Pilot" (https://www.herox.com/alphapilot/78-test-3) but can be used for other drone race contests. 



The Alpha Pilot qualification evaluates team's skills in both Control Engineering and Real-time programming, by controlling an autonomous drone on a provided simulator, [FlightGoggles](http://flightgoggles.mit.edu/) 

![](https://www.youtube.com/watch?v=e_3Yw0uPRKE)

# Architecture

Drobot V1 is a simple cascaded high levels position and attitude controllers, sensing pose from both IMU and camera : 

![](https://i.ibb.co/L1CZr3x/drobot.png)

# Prerequirements

## Recommanded Hardware

Tested on both :
* GPU 1050Ti , 16 Go of RAM
* AWS g3s.xlarge

## Software

* Ubuntu 16.04.5 LTS
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* Open CV 3.4
* CUDA drivers
* [FlightGoggles](http://flightgoggles.mit.edu/)

# Installation

Download the repository and execute : 
```
./install.bash && source catkin_ws/devel/setup.bash
```

# Running

Execute ROS node : 
```
rosrun flightgoggles scorer.sh
```

# About the Code

* Position and Attitude controllers in [Controller.py](./catkin_ws/src/control/scripts/leaderbord_groundtruth_v3.py)
* Position Estimation in [Stereo-SLAM.cpp](./catkin_ws/src/orb_slam_2_ros-master/ros/src/StereoNode.cc)

Tools codes from : [Drobot-dev](https://github.com/Nabiltchoulak/Drobot-controller-files-)

# Technical details

Please read following article : [Fast and Furious Computation for Drone Navigation](./Fast%20and%20Furious%20Computation%20for%20Drone%20Navigation.pdf)

# Related Publications

If you use of our code or our report in your project , please cite :

     @article{Drobot,
      title={{Drobot}: Fast and Furious Computation for Drone Navigation},
      author={Zeryab Moussaoui and al.},
      journal={Github},
      year={2019}
     }
     
