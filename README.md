## Unscented Kalman Filter Project Code

### Self-Driving Car Engineer Nanodegree Program

System:
Desktop, Windows 10 Bash on Ubuntu (Linux Subsystem)

The goals / steps of this project are the following:

Utilize a unscented kalman filter to find the state of a moving object according to radar and lidar measurements. Calculate RMSE according to ground-truth data. Project steps:

* Code the Predict, UpdateLidar and UpdateRadar functions to successfully build a Unscented Kalman filter.
* Code the RMSE.
* Initialize the state vector and covariance matrice using the first measurements.
* Predict the object position to current timestep and after recieving measurement update the prediction.
* Call correct measurment function (linear (laser), non-linear (radar)) according to sensor type.

### Project Setup: 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Step 1: Install Windows 10 Bash on Ubuntu. Follow the link for a nice [guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) 

Step 2: Install [mobaxterm](https://mobaxterm.mobatek.net/) so you can use Sublime (or any other text editor) inside the Bash. 

Step 3: Instead of Step 1 and 2 you can follow this [thread](https://nickjanetakis.com/blog/using-wsl-and-mobaxterm-to-create-a-linux-dev-environment-on-windows#wsl-conemu-and-mobaxterm-to-the-rescue) which also explains how to install Sublime, a nice text editor to be used on Ubuntu.

#### Data Set 1 Full Run With Different Standart Deviations of Accelaration and Change Rate of Yaw Angle  
<img width="500" alt="Data Set 1 Full Run With Different Standart Deviations of Accelaration and Change Rate of Yaw Angle " src="/images/std_a_ std_yawdd_.JPG">

#### Only Lidar, Only Radar and Lidar-Radar (Sensor Fusion) Modes Results
<img width="500" alt="Data Set 1: Detail" src="/images/lidar_radar_fusion.JPG">
