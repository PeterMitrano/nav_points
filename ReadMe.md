# Nav Points [![Build Status](https://travis-ci.org/PeterMitrano/nav_points.svg?branch=master)](https://travis-ci.org/PeterMitrano/nav_points)

Nav Points was made because setting repeatable goals for navigation in rviz is an obvious need. Nav Points works by putting interactive markers in rviz at specific locations. When you click on the marker, it sends a goal, on the `/move_base_simple/goal` topic by default (but you can change that).

The goals are stored in the parameter server as `/nav_points/nav_points`, which is a list of yaml elements like `'{x: 1, y: 2, theta: 1.5707}'`.

A config file can be used to initialize this list. For now, it is static. In the future I'll make a ROS API (pub/sub) for adding/removing, and a manual drag thing in rviz.

Remember to add the interactive markers in rviz.
