### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

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
<img width="800" alt="Data Set 1 Full Run With Different Standart Deviations of Accelaration and Change Rate of Yaw Angle " src="/images/std_a_ std_yawdd_.jpg">

Started trials with high (for a bycle) acceleration and change of yaw angle std_a: 9 m^2/s^2, std_yawdd: 9 m^2/s^2 and decreased to 3, 1, 0.5 and 0.25. Results were close around 0.5-1 m^2/s^2. Picked 1 m^2/s^2 since marginal benefit drops from this point.

#### Only Lidar, Only Radar and Lidar-Radar (Sensor Fusion) Modes Results
<img width="800" alt="Data Set 1: Detail" src="/images/lidar_radar_fusion.jpg">

Tried three different situations Lidar Only, Radar Only and Lidar+Radar (Sensor Fusion) to see how the results differ. Lidar gives better result finding the exact location of the bycle while radar is obviously better at determining speed of the bycle. As expected sensor fusion (lidar+radar) gives the best results.  

#### Extended Kalman vs Unscented Kalman Filter
<img width="500" alt="Data Set 1: Detail" src="/images/extended%20kalman%20filter%20result.JPG">

We can see certain amount of decrease in both VX (0.4158 -> 0.3353 and VY (0.4324 -> 0.2254). Though position estimations are close, speed estimations much better with Unscented Kalman Filter.
