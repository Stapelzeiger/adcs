# Attitude Determination Kalman Filter

This module contains a Kalman filter implementation and simulation of the rotation dynamics of solid bodies in 2D & 3D.

## Purpose of this module

This module is written to gain an understanding of how a Kalman filter for attitude estimation works.

## Unit Tests

There are unit tests for all the essential components.
To run the unit tests simply run the matlab command ```runtests```.

## run the simulation with visualization

just run the script `visualization.m` in matlab


## source code overview

math subroutines:

 - `quatconj.m` : conjugate of quaternion
 - `quatmult.m` : quaternion multiplication
 - `rotate_by_quaternion.m` : rotate a 3D vector by a quaternion

Kalman filter:

 - `ExtendedKalmanFilter.m` : generic EKF implementation
 - `KalmanFilter.m` : generic KF implementation

 - `Kalman3DBody.m` : simple EKF for 3D body assuming constant momentum
 - `Kalman3DBodySymbolicDerivation.m` : symbolic derivations for the above
 - `Kalman3DBodyErrorQuatSymbolicDerivation.m` : WIP

Simulation:

 - `RotationBody2D.m` : simulates 2d rotation dynamics & sensor readings
 - `RotationBody3D.m` : simulates 3d rotation dynamics & sensor readings
 - `Simulation3DBody.m` : runs the kalman together with 3d body simulation

Plotting:

 - `visualization.m` : run the simulation and visualize 3d cube
 - `cube_plot.m` : draw a cube

Simple 2d KF example:

 - `runPosVel2DSystem.m` : set up and run 2d Kalman
 - `plot_KF_results.m` : 2d graphs with error bar
