# Modern Robotics Capstone Project – Final Submission

Name: Vikash Upadhyay  
Course: Modern Robotics – Capstone Project  
Platform: Coursera  

Project Title:
Trajectory Tracking Control of a Mobile Manipulator Using Feedforward and Feedback Control

 
## Project Overview

This project implements trajectory generation and closed-loop control of a mobile manipulator using modern robotics principles. The objective is to track a desired end-effector trajectory accurately using feedback and feedforward control laws.

The system generates a reference trajectory, computes tracking errors, applies a feedback-plus-feedforward controller, and outputs wheel and joint velocities for motion execution.

 
## Control Strategy

Control Law Used:
Feedforward + Feedback (PI Controller)

Control Equation:
V = Adjoint(Tse⁻¹ * Tsed) * Vd + Kp * Xerr + Ki * ∫Xerr dt

Control Gains:
Kp = 5  
Ki = 0.5  

These gains were selected experimentally to achieve smooth trajectory tracking with minimal steady-state error and stable system response.

--------------------------------------------------

## Software Description

The project consists of the following modules:

1. trajectory_generator.py  
   Generates desired end-effector reference trajectories.

2. feedback_control.py  
   Implements feedforward + feedback control law.

3. main.py  
   Executes simulation, integrates robot motion, and stores results.

4. results.csv  
   Stores time history of robot configuration.

5. error_plot.png  
   Displays trajectory tracking error.

--------------------------------------------------

## How To Run

Install required dependencies:

pip install numpy matplotlib modern_robotics

Run the main program:

python main.py

--------------------------------------------------

## Results

The robot successfully tracks the desired trajectory with minimal tracking error.  
The generated CSV and error plot confirm stable convergence and smooth motion.

--------------------------------------------------

## Notes

All output files strictly follow the format specified in the Modern Robotics Capstone Project wiki.
