# Remote Center of Motion
Implementation of [Task Control with Remote Center of Motion Constraint for Minimally Invasive Robotic Surgery](https://ieeexplore.ieee.org/abstract/document/6631412?casa_token=2dfBz_G3vPoAAAAA:BwZvpCcLNtu8vhJQOkqPDvdDlvFbUEjcC8aXuTqRln92TV7RuOctKLwy2Sk_o1WBWxO89QWY) Aghakhani et al. with ROS and Moveit! integration. 

## Overview
Two classes provide functionallity

 - [RCMImpl](rcm_impl/include/rcm_impl/rcm_impl.h), computes joint angle updates under the control law presented in the paper

 - [RCMActionServer](rcm_impl/include/rcm_impl/rcm_action_server.h), has a [RCMImpl](rcm_impl/include/rcm_impl/rcm_impl.h) and implements a state machine that communicates to ROS via an action client

Overview shown below

<br/>
<img src="img/rcm_overview.png" width="800"/>

## Configuration
Configurations in [params.yml](rcm_impl/config/params.yml).

 - RCM computed in between links `link_pi` and `link_pip1`
 - Controls `planning_group`, as defined by Moveit! setup assistant
 - Joint position goals published to action server under `control_client`
