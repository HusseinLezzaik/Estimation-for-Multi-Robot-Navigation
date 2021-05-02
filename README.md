# Estimation for Multi Robot Navigation

## Introduction
The objective of this project is to study and evaluate on real experiments data fusion methods that improve the localization estimates of two communicating vehicles that share information, one vehicle being equipped with a Lidar.

## Project Keypoints
* Data collected from the Heudiasyc Laboratory UTC at July 2018.
* Scenario 1, the localization of each vehicle will be performed independently and without communication between the vehicles.
* Scenario 2, the leader sends its estimated pose with the associated covariance matrix.
* Scenario 3, same as scenario 2 but implementing the Unscented Transformation (UT).
* Scenario 4, now the follower also sends it relative measure pose (done by the lidar).
* Extended Kalman Filter (EKF), Scripts written in MATLAB

Read `ARS_04_miniproject_A20.pdf` for more details about the problem we solved in our work.

## Getting Started
1.  Clone our repo: `git clone https://github.com/HusseinLezzaik/Estimation-for-Multi-Robot-Navigation.git`
2.  Install [MATLAB](https://fr.mathworks.com/products/matlab-online.html).
3.  The main files are `"pb1.m"`, `"pb2.m"`, `"pb3.m"`, `"pb4.m"` that maintain the solution for each scenario.
4.  `Animation` can be turned off to view the results and plots directly.

You can find more details about our approach, equations, and results in `FINAL_REPORT.pdf`.

## Maintainers
* [Hussein Lezzaik](www.husseinlezzaik.com)
* [Hasan Kassem](https://www.linkedin.com/in/hasan-kassem-02625119b/)
* [Ahmad Shour](https://www.linkedin.com/in/ahmad-shour-1531371a8/)
