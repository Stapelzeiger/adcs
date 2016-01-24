# Attitude Determination Kalman Filter

This module contains a Kalman filter implementation and simulation of the rotation dynamics of solid bodies in 2D & 3D.

## Purpose of this module

This module is written to gain an understanding of how a Kalman filter for attitude estimation works.

## Unit Tests

There are unit tests for most of the essential components.
They are in the `*_test.m` files.
To run the unit tests simply run the matlab command ```clear classes; runtests```.

## run the simulation

For live visualization run the script `run_visualization.m` in matlab.

To run the simulation and show a plot afterwards run the script `run_graph.m` in matlab. This script does multiple runs (each with random initialization) and shows the estimation error plot for rate and attitude.



## source code overview

math subroutines:

 - `quatconj.m` : conjugate of quaternion
 - `quatmult.m` : quaternion multiplication
 - `rotate_by_quaternion.m` : rotate a 3D vector by a quaternion
 - `rotation_matrix_from_quat.m` : generate a rotation matrix from a quaternion
 - `quat_rand.m` : generate a random quternion with uniform probability
 - `quat_from_two_vect.m` : compute a quaternion that rotates a vector to another vector
 - `cross_prod_matrix` : build the cross product matrix from a vector

Kalman filter:

 - `ExtendedKalmanFilter.m` : generic EKF implementation
 - `KalmanFilter.m` : generic KF implementation
 _(there is no constistent distinction in variable naming between F (continuous time) and Phi (discrete time), this should be changed someday)_
 - `EKF3DConstMomentum.m` : simple EKF for 3D body assuming constant momentum
 - `EKF3DConstMomentumSymbolicDerivation.m` : symbolic derivations for the above
 - `MEKF3DConstMomentum.m` : Multiplicative EKF with constant momentum model
 - `MEKF3DConstMomentumSymbolicDerivation.m` : symbolic derivations for the above
 - `MEKF3DGyro.m` : Multiplicative EKF with gyroscope input
 - `MEKF3DGyroSymbolicDerivation.m` : symbolic derivations for the above

Simulation:

 - `RotationBody3D.m` : simulates 3d rotation dynamics & sensor readings
 - `Simulation3DBody.m` : runs the kalman together with 3d body simulation

Plotting:

 - `run_visualization.m` : run the simulation and visualize 3d cube
 - `cube_plot.m` : draw a cube, used in run_visualization.m
 - `run_graph.m` : run multiple simulations and plot error graphs

Simple 2d KF example:
(this is just a toy example of a linear Kalman filter)

 - `runPosVel2DSystem.m` : set up and run 2d Kalman
 - `RotationBody2D.m` : simulates 2d rotation dynamics & sensor readings
 - `plot_KF_results.m` : 2d graphs with error bar
