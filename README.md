# Stability-Analysis-using-Quadratic-Constraints-for-Systems-with-Neural-Network-Controllers
This code is to accompany the paper [Stability Analysis using Quadratic Constraints for Systems with Neural Network Controllers](https://arxiv.org/pdf/2006.07579.pdf). It computes an ellipsoidal inner-approximation to the region of attraction of NN controlled-systems.

### Authors:
He Yin (he_yin at berkeley.edu)

## Getting Started
All the code is written in MATLAB.

### Prerequisites
There is one package required:
* [MOSEK](https://www.mosek.com/): Commercial semidefinite programming solver

### Way of Using the Code
* Interted pendulum example: go to its folder, and run the file **Pendulum_sin_local.m**

## ROA inner-approximation for a control saturated inverted pendulum
![pendulum](Inverted_Pendulum_control_saturation/pendulum_saturation.pdf)