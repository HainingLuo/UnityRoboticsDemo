# Unity Robotics Demo

Use Unity like Gazebo!

<p align="center">
  <img src="images/unity_demo.gif" alt="animated" />
</p>

---
## Prerequisites
**1. Tested on Ubuntu 18.04 with ROS melodic** <br>
**2. Downloading this demo to your local directory** <br>
```
    git clone https://github.com/HainingLuo/UnityRoboticsDemo.git
```
**3. Install Unity3D version 2020.2.0b9**

If you are using Linux, you may need to Download Unity3D archive versions by:  <br>
1. Go to the [download page](https://unity3d.com/unity/beta/2020.2.0b9) of the archive version.
1. Right click the green button ‘install with unity hub’ to copy its link address.
2. CD to Unity Hub directory and start Unity hub with command: `./UnityHub.AppImage <link address>`.

## Setting up ROS workspace
```
    cd unity_ws/src
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    git clone https://github.com/ImperialCollegeLondon/yumi-prl
    cd yumi && git checkout melodic cd ../..
    cd .. && catkin_make && source devel/setup.bash
```
## Setting up Unity
