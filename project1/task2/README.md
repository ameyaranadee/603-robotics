# 1. Setup catkin workspace if needed
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# 2. Create the package
cd ~/catkin_ws/src
catkin_create_pkg P1D2_ameya_ranade rospy geometry_msgs turtlesim std_msgs

# 3. Add script
cd P1D2_ameya_ranade
mkdir scripts
cd scripts
touch P1D2_ameya_ranade.py
chmod +x P1D2_ameya_ranade.py
# (Edit the file to add your code)

# 4. Edit CMakeLists.txt and package.xml as needed
# (Make sure Python script is installed properly)

# 5. Build and source
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 6. Run the experiment in separate terminals:

# Terminal 1
roscore

# Terminal 2
source ~/catkin_ws/devel/setup.bash
rosrun turtlesim turtlesim_node

# Terminal 3
source ~/catkin_ws/devel/setup.bash
rosrun P1D2_firstname_lastname P1D2_ameya_ranade.py
