ROS code

Used to communicate with Particle Photon via RS232

Put this in your catkin_ws workspace folder

Run this node with the following command:
  roslaunch myrobot robot_velocity_controllers.launch

My experience with Particle Photon USB RS232 comms is it can take somewhere in the order of 15sec for the port to become available. So wait for this period before launching myrobot node

