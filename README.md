# Estimation-for-Multi-Robot-Navigation
The objective of this project is to study and evaluate on real experiments data fusion methods that improve the localization estimates of two communicating vehicles that share information, one vehicle being equipped with a Lidar.
o Data collected from the Heudiasyc Laboratory UTC at July 2018.
o Scenario 1, the localization of each vehicle will be performed independently and without communication between the vehicles.
o Scenario 2, the leader sends its estimated pose with the associated covariance matrix.
o Scenario 3, same as scenario 2 but implementing the Unscented Transformation (UT).
o Scenario 4, now the follower also sends it relative measure pose (done by the lidar).
o Extended Kalman Filter (EKF), Scripts written in MATLAB
