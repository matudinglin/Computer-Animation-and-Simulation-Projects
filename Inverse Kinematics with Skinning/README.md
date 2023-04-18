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
  - http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf

## Animation Requirements Check List
- In results/...
  - armadillo-Tikhonov-LBS.gif/mp4
  - dragon-Tikhonov-LBS.gif
  - hand-Tikhonov-LBS.gif

## Extra Credits Check List
- Implement other skinning methods:
  - dual-quaternion skinning. 
- Provide a comparison between linear blend skinning and dual quaternion skinning
- Implement other IK algorithms
  - Pseudo Inverse method
  - Jacobian Transpose method
- Provide a Analysis of these IK algorithms.
- When the user moves the IK handle for a long distance, divide the IK process into several sub-steps to improve the solution, where each sub-step solves the IK problem on a portion of the original distance.

## Comparison between LBS and DQS

See results in Showcase.

**Accuracy:** LBS can struggle with accurately deforming complex meshes, it will cause Candy-Wrapper Artifact, while DQS is able to handle this problem. It's because LBS is linear blending, which may cause porblem, we can use quaternion and SLERP to solve it.

**Performance:** LBS is faster than DQS. Because DQS required more computation than LBS, including quaternion and matrix transformation, etc.

**In summary,** LBS is a fast and simple technique for skinning in computer animation, while DQS provides greater accuracy for more complex characters but at the cost of increased computational expense.

## Analysis of three IK methods

**Pseudo Inverse method:** This method is computationally efficient, but it can be unstable.

**Jacobian Transpose method:** This method is computationally efficient, and it's more stable than PI method IK, but it's sensitive to the choice of the step size.

**Tikhonov Regularization method:** This method is more stable and accurate than Jacobian Transpose method IK, and converge faster than the Jacobian Transpose method, but need more computation.


# Showcase
## Armadillo, Tikhonov Regularization method IK, Linear Blend Skinning:

![1](./results/armadillo-Tikhonov-LBS.gif)

## Comparison between LBS and DQS:

DQS:

![2](./results/hand-Tikhonov-DQS.gif)

LBS:

![3](./results/hand-Tikhonov-LBS.gif)

## Analysis of three IK methods:

Pseudo Inverse method:

![4](./results/armadillo-Pseudoinverse-LBS.gif)

Jacobian Transpose method:

![5](./results/armadillo-Transpose-LBS.gif)

