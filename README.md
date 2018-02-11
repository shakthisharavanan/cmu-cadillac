## ROS packages to simulate the CMU Cadillac AV in gazebo

Note:- cmu_cadillac_navigation is not fully developed

### Requirements
* ROS indigo with Gazebo 2
* Other ROS packages: ackermann_msgs, openslam_gmapping, slam_gmapping, robot_pose_ekf
* Required gazebo models

## Install
clone & compile cmu_cadillac

```sh
cd /PATH/TO/CATKIN_WS/src
git clone https://github.com/shakthisharavanan/cmu-cadillac.git
cd ..
catkin_make
```

## Run simulation
To simulate parallel parking procedure:
```sh
roslaunch cmu_cadillac_gazebo parking.launch
cd cmu_cadillac/cmu_cadillac_gazebo/nodes
python parking_controller_modified.py
```

To visualize in RViz:
```sh
roslaunch cmu_cadillac_description cmu_cadillac_rviz.launch
```

To simulate parallel parking in a one way street with vehicles on either side
```sh
roslaunch cmu_cadillac_gazebo parking.launch
cd cmu_cadillac/cmu_cadillac_gazebo/nodes
python one-way_parking.py
```



To simulate parallel parking in curved roads
```sh
roslaunch cmu_cadillac_gazebo parking.launch
cd cmu_cadillac/cmu_cadillac_gazebo/nodes
python parking_curved.py
```


