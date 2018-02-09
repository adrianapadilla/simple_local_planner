

# Simple Local Planner

Plugin to the ROS base_local_planner. Implements a wrapper for a simple path planner that follows the global path updated at a certain frequency. 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for simulation on a virtual robot. However, they can easily be extrapolated to be used for testing on a real robot.

### Prerequisites

Software.
- Ubuntu Linux OS
- [Robot Operating System (ROS)](http://wiki.ros.org/ROS/Installation) 
- [The ROS navigation package](http://wiki.ros.org/navigation)
- [Gazebo](http://gazebosim.org/download)

Hardware:
- Robot model compatible with this project: Pioneer 3-AT
- Laser compatible with this project: Hokuyo laser

### Installing and Using

The following tutorial assumes that you have downloaded and installed ROS, the navigation package and Gazebo. The algorithm in this project has been developed to be used with a specific robot model: the Pioneer 3-AT, and a specific laser: the Hokuyo laser. Please make sure you have access to this robot and laser on Gazebo before going any further. Also, if you haven’t already created a world on Gazebo to test this project, make sure to do so. Any world will do. You will also need a map of that world, so use [gmapping](http://wiki.ros.org/gmapping) or any other mapping tool to create one.

Once ready, install this project. 

In order to use the project, I will provide the move_base.launch file used during development. Feel free to merge it with your own, if applicable. Three .yaml files containing the costmap common parameters, global costmap parameters and local costmap parameters are also provided.

**move_base.launch**
```
<launch>
  <master auto="start"/>

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
   
    <remap from="cmd_vel" to="<INSERT YOUR VELOCITY TOPIC HERE>" />

    <!-- replacing default local planner with the project local planner —>
    <param name="base_local_planner" value="simple_local_planner/SimplePlannerROS" />

    <rosparam file=“<INSERT FILE ADDRESS>/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="<INSERT FILE ADDRESS>/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="<INSERT FILE ADDRESS>/local_costmap_params.yaml" command="load" />
    <rosparam file="<INSERT FILE ADDRESS>/global_costmap_params.yaml" command="load" />

  </node>

</launch>
```

**costmap_common_params.yaml**
```
obstacle_range: 2.5
raytrace_range: 3.0
footprint: [[0.55, 0.55], [-0.55, 0.55], [-0.55, -0.55], [0.55, -0.55]]
#robot_radius: ir_of_robot
inflation_radius: 0.55
observation_sources: laser_scan_sensor 
laser_scan_sensor: {sensor_frame: hokuyo_link, data_type: LaserScan, topic: laser, marking: true, clearing: true}
```

**local_costmap_params.yaml**
```
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 0.25
  publish_frequency: 5.0
  static_map: false
  rolling_window: true
  width: 6.0
  height: 6.0
  resolution: 0.05
```

**global_costmap_params.yaml**
```
global_costmap:
  global_frame: /map
  robot_base_frame: base_link
  update_frequency: 0.25
  static_map: true
```

Also, if you haven’t already got an amcl launch file, feel free to use the following:

**amcl.launch**
```
<launch>
    <node name="amcl" pkg="amcl" type="amcl" output="screen" >
        <remap from="scan" to=“<INSERT YOUR LASER TOPIC HERE>”/>
        
        <!-- Overall filter parameters -->
        <param name="min_particles" value="500"/>
        <param name="max_particles" value="5000"/>
        <param name="kld_err" value="0.05"/>
        <param name="kld_z" value="0.99"/>
        <param name="update_min_d" value="0.01"/>
        <param name="update_min_a" value="0.05"/>
        <param name="resample_interval" value="1"/>
        <param name="transform_tolerance" value="0.2"/>
        <param name="recovery_alpha_slow" value="0.0"/>
        <param name="recovery_alpha_fast" value="0.0"/>
        <param name="gui_publish_rate" value="100.0"/>
        
        <!-- Laser model parameters -->
        <param name="laser_max_beams" value="30"/>
        <param name="laser_z_hit" value="0.5"/>
        <param name="laser_z_short" value="0.05"/>
        <param name="laser_z_max" value="0.05"/>
        <param name="laser_z_rand" value="0.5"/>
        <param name="laser_sigma_hit" value="0.2"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_model_type" value="likelihood_field"/>
        <param name="laser_likelihood_max_dist" value="2.0"/>
        
        <!-- Odometery model parameters -->
        <param name="odom_model_type" value="diff"/>
        <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/>
        <param name="odom_alpha3" value="0.8"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="odom_frame_id" value="odom"/>
        <param name="base_frame_id" value="base_link"/>
        <param name="global_frame_id" value="map"/>
    </node>
</launch>
```

Once done creating the above files, it’s time to try out the project!

Start by launching Gazebo. Open the world you will be using for this simulation, as well as the robot model mentioned above. 

Run map_server with the name of your map, like so:
```
rosrun map_server map_server <INSERT YOUR MAP NAME>.yaml
```
Next, launch amcl. [amcl](http://wiki.ros.org/amcl) takes in the previous laser-based map and the robot’s laser scans and transform messages, and outputs pose estimates. This output is necessary information for a path planning algorithm such as the one implemented in this project. In a new terminal, write:
```
roslaunch amcl.launch
```
Next, run rviz. This is a 3D visualization tool for ROS that will allow you to have more information about what is going on in Gazebo. You can use rviz to choose a destination point for the robot to travel to, as well as visualize the global and local paths. In order to start rviz, write the following in a new terminal:
```
rosrun rviz rviz
```
Once all the above steps have been completed, you are ready to launch the move_base.launch file. Again, in a new terminal, write:
```
roslaunch move_base.launch
```
Now you are ready to choose a destination for the robot in rviz. In the top bar of the program you should see a button saying “2D nav goal”. Click this button and set a destination. 

Watch the robot move to the destination!

## Authors

Adriana Padilla 
- Linkedin : https://www.linkedin.com/in/adriana-m-padilla/
- Email: adriana.m.padilla@hotmail.com

## License

MIT License

**Copyright (c) 2017 Adriana Padilla**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.



