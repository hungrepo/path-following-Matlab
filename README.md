# path-following-Matlab
## Description
This repo contains Matlab codes that implement and animate the path following methods studied in the paper entitled:

Nguyen Hung et al. A review of path following control strategies for autonomous robotic vehicles: theory, simulations, and experiments, Journal of Field Robotics, 2022.

Link: https://nt-hung.github.io/files/pdf/research/JFR2022_preprint.pdf

## Prerequisite
- Matlab2016a or newer
- Casadi - a tool to formulate optimal control problems [only need for MPC methods (Method 5 and 7)]

## Installation
- Chose a folder that you want to locate your project, then 
- clone/download the project
- run PFtools.m and see the results.

If you want to test different type of the path or controller, just change parameters in variables "path_type" and "controller" in PFtools.m

## Relevent ROS package

The path following methods also were implemented in the followng ROS package: https://github.com/dsor-isr/Paper-PathFollowingSurvey

## References 
If you find the code useful and would like to cite, please kindly reference to the following paper:

Hung N., Rego F., Quintas J., Cruz J., Jacinto M., Souto D. et al. (2022) A review of path following control strategies for autonomous
robotic vehicles: Theory, simulations, and experiments. Journal of Field Robotics, 1–33.
https://doi.org/10.1002/rob.22142

## Videos 

 
