# AAE497-Summer2022
A collection of the end-of-chapter projects from Randal Beard and Timothy McLain's *Small Unmanned Aircraft: Theory and Practice*

Sample code and templates taken from https://github.com/randybeard/uavbook/tree/main/mavsim_python

A README file dedicated to each project is included in the respective folder.

## Chapter 2 - Position and Attitude
The Chapter 2 project program takes in inputs of position and attitude to display a 3D model of the aircraft. 

The program generates two models in separate windows: a 3D mesh representation of the aircraft's vertices and a completed 3D model.
The window with the mesh model has sliders that can be used to translate and rotate the aircraft in the window.
This is updated in real-time in both windows.

## Chapter 3 - Kinematics and Dynamics
The Chapter 3 project program takes in inputs of constant applied forces and moments to calculate and update the dynamic state of the aircraft.
A 3D visualization of the aircraft's behavior is displayed in a separate window.

## Chapter 4 - Forces and Moments
The Chapter 4 project program takes in inputs of actuator commands to calculate and update the dynamic state of the aircraft.
It calculates the dynamic applied forces and moments on the vehicle.
A 3D visualization of the aircraft's behavior is displayed in a separate window.

## Chapter 5 - Linearization
The Chapter 5 project program builds on the previous projects, using the modules to calculate trim, and generate the transfer function and state space model of the aircraft.

The pertinent coefficients to the transfer function models and matrices for the state space models are stored in a separate file.

## Chapter 6 - Autopilots
The Chapter 6 project program takes the transfer function and state space models from the previous project and creates an autopilot. There are two types of autopilot in this project: the Successive Loop Closure (SLC) autopilot and the Linear-Quadratic Regulator (LQR) autopilot. 

The program accepts inputs of commanded altitude, course, and airspeed and displays a visualization of how the aircraft behaves. 
