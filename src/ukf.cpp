#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_x_;

  time_us_ = 0.0;

  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);  
  Xsig_pred_.setZero();

  weights_ = VectorXd(2*n_aug_+1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_)
  {
    cout << "Initializing for the first time, expecting first measurement." << endl;
    x_.setZero();

    if (meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      //cout << "Lidar x_: "<< x_ << endl;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR ){

      x_(0) =  meas_package.raw_measurements_(0) * cos (meas_package.raw_measurements_(1));
      x_(1) =  meas_package.raw_measurements_(0) * sin (meas_package.raw_measurements_(1));
      //cout << "Radar x_: "<< x_ << endl;
    }  

    time_us_ = meas_package.timestamp_;

    //cout << "P_: "<<P_ << endl;
    P_.setIdentity();
    //cout << "P_ Identity: "<<P_ << endl ;

    is_initialized_ = true;

    cout << "End of initialization: " << is_initialized_ << endl;
    return;
  }

  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  //cout << "delta_t: " << delta_t << "\t";  

  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    cout << "Update Radar Started"<<endl;
    UpdateRadar(meas_package);
  }

  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    cout << "Update Lidar Started"<<endl;
    UpdateLidar(meas_package);
  }

  time_us_ = meas_package.timestamp_;
  //cout << "time_us_: " << time_us_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */ 

  //cout << "Entering Prediction" << endl;
  //cout << Xsig_pred_;

  //**************************************//
  //-------Create Sigma Points------------//
  //**************************************//

  MatrixXd Xsig = MatrixXd(n_x_,2*n_x_+1);
 
  double lambda_nx = sqrt(lambda_+n_x_);

  MatrixXd A = P_.llt().matrixL();

  Xsig.col(0) = x_;  

  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i+1) = x_ + (lambda_nx * A.col(i));
    Xsig.col(i+n_x_+1) = x_ - (lambda_nx * A.col(i));
  }

  //cout << "**********Xsig: \n"<< Xsig << endl;
  //-----------------Control 2018 04 11 OK!

  //************************************//
  //--------Augmented Sigma Points------//
  //************************************//

  //Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.setZero(); //Set zero to avoid unwanted results
  //Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
  P_aug.setZero();
  //Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_,2*n_aug_+1);
  Xsig_aug.setZero();

  //Create augmented mean state
  x_aug.head(5) = x_;
  //Create augmented covariance matrix
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  //cout << "x_aug: " << x_aug <<endl;
  //cout << "P_aug: " << P_aug << endl;

  //Create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  //Define augmented lambda
  double lambda_aug = 3-n_aug_;

  //Create augmented sigma points
  //Assign the first col
  Xsig_aug.col(0) = x_aug;  

  //Set augmented sigma points 
  for(int j=0; j<n_aug_; j++){
    Xsig_aug.col(j+1) = x_aug + sqrt(lambda_aug+n_aug_) * A_aug.col(j);
    Xsig_aug.col(j+1+n_aug_) = x_aug - sqrt(lambda_aug+n_aug_) * A_aug.col(j);
  }

  cout << "**********Xsig_aug: \n"<< Xsig_aug <<endl;
  //-----------------Control 2018 04 11 OK!

  //********************************************//
  //---------Sigma Point Prediction Step--------//
  //********************************************//

  //MatrixXd state_matrix = MatrixXd(5,1);
  //MatrixXd integral_matrix = MatrixXd(5,1);
  //MatrixXd noise_matrix = MatrixXd(5,1);
  
  //predict sigma points
  for(int i=0; i<2 * n_aug_ + 1; i++){

    //state_matrix = Xsig_aug.col(i);
    //cout<< "state_matrix: "<<state_matrix << endl; 

      //std::cout << "noise_ak: " << noise_ak << "noise_yaw_dotdot_k: " << noise_yaw_dotdot_k << "\n";

    //state_matrix.conservativeResize(5, 1);

    //std::cout << state_matrix; 
    float px_k =Xsig_aug(0,i);
    float py_k = Xsig_aug(1,i);  
    float v_k = Xsig_aug(2,i);
    float yaw_k = Xsig_aug(3,i);
    float yawdot_k = Xsig_aug(4,i);
    float noise_ak = Xsig_aug(5,i);
    float noise_yaw_dotdot_k = Xsig_aug(6,i);

      //std::cout << "v_k: " << v_k << "yaw_k: " << yaw_k << "yawdot_k: " << yawdot_k << "\n";

    float px_p, py_p;

    if(fabs(yawdot_k) > 0.001){
      //integral_matrix(0,0) = v_k/yawdot_k*(sin(yaw_k+yawdot_k*delta_t)-sin(yaw_k));
      px_p = px_k + v_k/yawdot_k*(sin(yaw_k+yawdot_k*delta_t)-sin(yaw_k));
      //integral_matrix(1,0) = v_k/yawdot_k*(-cos(yaw_k+yawdot_k*delta_t)+cos(yaw_k));
      py_p = py_k + v_k/yawdot_k*(-cos(yaw_k+yawdot_k*delta_t)+cos(yaw_k));
    }

    else{
      //integral_matrix(0,0) = v_k*cos(yaw_k)*delta_t;
      px_p = px_k + v_k*cos(yaw_k)*delta_t;
      //integral_matrix(1,0) = v_k*sin(yaw_k)*delta_t;
      py_p = py_k + v_k*sin(yaw_k)*delta_t;
    }    

    //integral_matrix(3,0) = yawdot_k * delta_t;

    float v_p = v_k;
    float yaw_p = yaw_k + yawdot_k*delta_t;
    float yawdot_p = yawdot_k;
/*
    noise_matrix(0,0) = (delta_t*delta_t*cos(yaw_k)*noise_ak)/2;
    noise_matrix(1,0) = (delta_t*delta_t*sin(yaw_k)*noise_ak)/2;
    noise_matrix(2,0) = (delta_t*noise_ak);
    noise_matrix(3,0) = (delta_t*delta_t*noise_yaw_dotdot_k)/2;
    noise_matrix(4,0) = delta_t*noise_yaw_dotdot_k;*/

    px_p += (delta_t*delta_t*cos(yaw_k)*noise_ak)/2;
    py_p += (delta_t*delta_t*sin(yaw_k)*noise_ak)/2;
    v_p += (delta_t*noise_ak);
    yaw_p += (delta_t*delta_t*noise_yaw_dotdot_k)/2;
    yawdot_p += delta_t*noise_yaw_dotdot_k;

    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawdot_p;

    //std::cout<<integral_matrix;    
    //Xsig_pred_.col(i) = state_matrix + integral_matrix + noise_matrix;
  }    
  cout << "*****************Xsig_pred_: \n" << Xsig_pred_ << endl;

  //****************************************************//
  //----------Predict Mean and Covariance---------------//
  //****************************************************//

  //Set the weights
  for(int i=0; i<(2*n_aug_+1); i++){

    if(i==0){
      weights_(i) = lambda_aug/(lambda_aug+n_aug_);}

      else{
              //weight of the rest
        weights_(i) = 1/(2*(lambda_aug+n_aug_));}   
  }

  //Predcit the state mean
  x_.setZero();
  for(int j=0; j<(2*n_aug_+1); j++){
 
      x_ += weights_(j)*Xsig_pred_.col(j);} 
      
  //std::cout << x;
  
  //predict state covariance matrix
  P_.setZero();
  for(int k=0; k<(2*n_aug_+1); k++){

    VectorXd x_diff = Xsig_pred_.col(k)-x_;
    //Normalize angles
    x_diff(3) = fmod(x_diff(3), 2*M_PI);

    P_ += weights_(k)*x_diff*x_diff.transpose();
  }

  //cout << "x_ :" << x_ << endl;
  //cout << "P_ :" << P_ << endl;  
}  

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  cout << "***************************\n-----------Lidar started------------\n***************************" << endl;
  n_z = 2;   

  VectorXd z = VectorXd(n_z);
  z.setZero();
  //cout << "z: "<< z << endl;

  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
  
  //cout<< "m1: "<< meas_package.raw_measurements_(0) <<endl;
  //cout<< "m2: "<< meas_package.raw_measurements_(1) <<endl;
  //cout << "z: " << z << endl;
 
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  cout << "R: " << R << endl;

  MatrixXd H = MatrixXd(n_z,n_x_);
  H <<  1,0,0,0,0,
        0,1,0,0,0;

  cout << "H: " << H << endl;

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  VectorXd z_pred = H*x_;
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = fmod(z_diff(1),2*M_PI);

  cout << "z_diff: " << z_diff << endl;

  //Update State and Covariance
  x_ = x_+ K * z_diff;

  MatrixXd I = MatrixXd::Identity(n_x_,n_x_); 
  P_ = (I-K*H)*P_;

  cout << "Lidar x_: " << x_ << endl;
  cout << "Lidar P_: " << P_ << endl;

  cout << "***************************\n-----------Lidar Ends------------\n***************************" << endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //Radar measurement dimension ro, phi, ro_dot
  cout << "***************************\n-----------Radar started------------\n***************************" << endl;

  n_z = 3;
  //********************************************//
  //------------Predict Sigma Points-----------//
  //******************************************//


  //Matrix of sigma points in measurement space
  MatrixXd Zsig =  MatrixXd(n_z, 2*n_aug_+1);
  cout << "Zsig: "<< Zsig <<endl;

  //Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.setZero();
  cout << "z_pred: "<< z_pred <<endl;

  //Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.setZero();
  cout << "S: "<< S <<endl;

  float ro, phi, ro_dot, px, py, v, psi, psi_dot; 

  cout << "transform sigma points into measurement space"<< endl;
  //transform sigma points into measurement space
  for(int i=0; i < 2 * n_aug_ + 1; i++){
      
      px = Xsig_pred_(0,i);
      py = Xsig_pred_(1,i);
      v = Xsig_pred_(2,i);
      psi = Xsig_pred_(3,i);
      psi_dot = Xsig_pred_(4,i);
      
      //std::cout << "px "<< px << " py " <<py << " v " << v << " psi " << psi << "psi_dot " << psi_dot << std::endl;
      ro = sqrt(px*px+py*py);
      phi = atan2(py,px);
      ro_dot = (px*cos(psi)*v + py*sin(psi)*v)/ro;
      
      Zsig(0,i) = ro;
      Zsig(1,i) =phi;
      Zsig(2,i) =ro_dot;
      
      //std::cout << "Zsig: " << Zsig.col(i) << std::endl;
  }
  cout << "calculate mean predicted measurement"<< endl;
  //calculate mean predicted measurement
  for (int j=0; j < 2 * n_aug_ +1; j++){
      z_pred += weights_(j)*Zsig.col(j);
  }
  
  //std::cout << z_pred.size();
  
  MatrixXd R = MatrixXd(n_z,n_z);
  R.setZero();
  
  R << std_radr_*std_radr_ , 0 ,0,
        0, std_radphi_*std_radphi_ ,0,
        0,0,std_radrd_*std_radrd_;
        
  cout << "R: "<<  R <<"\n";

  cout << "calculate innovation covariance matrix S"<< endl;
  //calculate innovation covariance matrix S
  for (int k=0; k < 2 * n_aug_ +1; k++ ){

    VectorXd z_diff = Zsig.col(k) - z_pred;

    //angle normalization
    z_diff(1) = fmod(z_diff(1),2*M_PI);

    S += weights_(k)*z_diff*z_diff.transpose();
  }
  
  S += R;
  cout << "S: " << S;

  //***********************************************//
  //-------Calculate cross correlation matrix------//
  //***********************************************//

  MatrixXd Tc = MatrixXd(n_x_,n_z);
  Tc.setZero();
  cout << "Tc: " << Tc << endl;


  for(int i=0; i< 2 * n_aug_ + 1; i++){

    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = fmod(z_diff(1),2*M_PI);

    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    //Normalize angles
    x_diff(3) = fmod(x_diff(3),2*M_PI);


    Tc += weights_(i)*x_diff*z_diff.transpose();
  } 
  
  //Calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  cout << "K: " << K << endl;

  //Create vector for radar reading data ro, phi, ro_Dot
  VectorXd z = VectorXd(n_z);
  z.setZero();
  cout << "Z empty: " << z<< endl;

  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),meas_package.raw_measurements_(2);

  cout << "Z filled: " << z << endl;
  
  //******************************************************//
  //---------Update state mean and covariance matrix------//
  //******************************************************//

  x_ += K*(z-z_pred);
  P_ += -K*S*K.transpose();

  cout << "x_: " << x_ << endl;
  cout << "P_: " << P_ << endl;

  cout << "***************************\n-----------Radar Ends------------\n***************************" << endl;

}
