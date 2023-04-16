# Overview
CSCI 520 Assignment 3

By Yao Lin

Open Source on github: https://github.com/matudinglin/Computer-Animation-and-Simulation-Projects

Keywords: skinning, forward kinematics (FK), inverse kinematics (IK), Eigen, C++

# Requirments Check List
## Basic Requirements Check List
- Skinning
  - Linear Blend Skinning
  - Dual-Quaternion Skinning
- Forward Kinematics
- Inverse Kinematics
  - Pseudo Inverse method
  - Jacobian Transpose method
  - Tikhonov Regularization method
  - Damped least squares method
  - http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf

## Animation Requirements Check List
- In results/...
  - 300 jpg sreenshots
  - 000.gif animation
  - 000.mp4 animation

## Extra Credits Check List
- Implement other skinning methods:
  - dual-quaternion skinning. 
- Provide a comparison between linear blend skinning and dual quaternion skinning
- Implement other IK algorithms
  - Pseudo Inverse method
  - Jacobian Transpose method
  - Damped least squares method
- Provide a comparison between these IK algorithms.
- When the user moves the IK handle for a long distance, divide the IK process into several sub-steps to improve the solution, where each sub-step solves the IK problem on a portion of the original distance.

## Comparison between LBS and DQS

## comparison four IK methods


# Showcase
Animation:

![animation](./results/000.gif)

Images:

![001](./results/001.jpg)

![093](./results/093.jpg)



