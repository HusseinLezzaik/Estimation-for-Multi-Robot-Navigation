# Estimation-for-Multi-Robot-Navigation
The objective of this project is to study and evaluate on real experiments data fusion methods that improve the localization estimates of two communicating vehicles that share information, one vehicle being equipped with a Lidar.
* Data collected from the Heudiasyc Laboratory UTC at July 2018.
* Scenario 1, the localization of each vehicle will be performed independently and without communication between the vehicles.
* Scenario 2, the leader sends its estimated pose with the associated covariance matrix.
* Scenario 3, same as scenario 2 but implementing the Unscented Transformation (UT).
* Scenario 4, now the follower also sends it relative measure pose (done by the lidar).
* Extended Kalman Filter (EKF), Scripts written in MATLAB
## Requirements
Please make sure to have MATLAB installed on your PC.
