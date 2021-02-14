cd unity_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
git clone https://github.com/ImperialCollegeLondon/yumi-prl
cd yumi && git checkout melodic cd ../..
cd .. && catkin_make && source devel/setup.bash
