#include "ukf.h"
#include "Eigen/Dense"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

   

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  is_initialized_ = false;

   //Initial Time before any measurement
  time_us_ = 0.0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //Size of State Vector 
  n_x_ = 5;

  //Size of augmented Vector, adding two noise parameters
  n_aug_ = 7;

  //Good start Value - Spacing distance of sigma points around the mean
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd( n_x_, n_x_);

  //Predicted Sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  //
  weights_ = VectorXd(2*n_aug_ +1);
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);




  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.5; //1.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.3; // 1.7;

 
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() {}

double UKF::NormalizeAngleBetwnPi(double inpAngle)
{
    double oupAngle;
    oupAngle = fmod(inpAngle + M_PI, 2.0 * M_PI);  // Normalize between 0 and 2Pi
    if (oupAngle < 0) 
    {
        oupAngle += 2.0 * M_PI;  // Shift to ensure it's in the range [0, 2Pi]
    }
    oupAngle -= M_PI;  // Shift to the range [-pi, pi]
    return oupAngle;

}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  

  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      //Get the relevant measurements capable by sensor, and set other state parasm to 0
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0,
            0,
            0;

      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho,phi,rhoDot;
      rho = meas_package.raw_measurements_[0];
      phi = meas_package.raw_measurements_[1];
      rhoDot = meas_package.raw_measurements_[2];

      x_ << rho * cos(phi),
            rho * sin(phi),
            0,
            0,
            0;
      
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, std_radrd_ * std_radrd_, 0, 0,
            0, 0, 0, std_radphi_ * std_radphi_, 0,
            0, 0, 0, 0, 1;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;

    if(checkCode_)
    {
      std::cout << "Initialized with the first measurements" << std::endl;
    }
    
  }

  else
  {
    double dt = (meas_package.timestamp_ - time_us_)/ 1000000.0;
    time_us_ = meas_package.timestamp_;
    // std::cout << "dt: "<< dt << std::endl;

    Prediction(dt);
    if(checkCode_)
    {
        std::cout << "PREDICTION " << std::endl;
        std::cout << "x_: " <<  std::endl;
        std::cout << x_ <<  std::endl;
        std::cout << "P_: " <<  std::endl;
        std::cout << P_ <<  std::endl;
    }

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
    {
      UpdateLidar(meas_package);
      if (checkCode_)
      {
        std::cout << "Updated with LIDAR " << std::endl;
        std::cout << "x_: " <<  std::endl;
        std::cout << x_ <<  std::endl;
        std::cout << "P_: " <<  std::endl;
        std::cout << P_ <<  std::endl;
      }
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      UpdateRadar(meas_package);
      if (checkCode_)
      {
        std::cout << "Updated with RADAR " << std::endl;
        std::cout << "x_: " <<  std::endl;
        std::cout << x_ <<  std::endl;
        std::cout << "P_: " <<  std::endl;
        std::cout << P_ <<  std::endl;
      }

    }

    // sleep_for(nanoseconds(10));
    if(checkCode_)
      std::cout << "--------------------------------------------------------------" << std::endl;
    // sleep_until(system_clock::now() + seconds(1));
    
  }
  


}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //1.AUGMENTATION---------------------------------------------
    //1.1 We need an augmented Mean Vector
    //When I say augmented, the state vector has to be padded with two noises, Linear acceleration and angular acceleration
    VectorXd x_aug(n_aug_);
    x_aug.head(n_x_) = x_; //Setting first 5 elements of augmented state vector to state itself
    
    //Set other elements to 0;
    for(int i=n_x_; i<x_aug.size(); i++ )
    {
      x_aug(i) = 0;
    }

    //1.2 We need an augmented covariance matrix from current covariance matrix
    //Setting up the new variances
    VectorXd varianceValue(n_aug_-n_x_);
    varianceValue << std_a_*std_a_ , std_yawdd_*std_yawdd_; // This can be set in constructor istelf , for generalization
    MatrixXd P_aug(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;

    //Set other variance values
    for(int i=n_x_; i<x_aug.size(); i++ )
    {
      P_aug(i,i) = varianceValue(i-n_x_);
    }

  //2.CREATE SIGMA POINTS---------------------------------------------
    //2.1
    //Create a sigma point matrix structure
    MatrixXd Xsig_aug(n_aug_, 2*n_aug_ + 1);

    //SquareRoot of covariance Matrix
    MatrixXd P_sqrt = P_aug.llt().matrixL();

    //Generate Sigma Points
    //ActualMean ,
    //ActualMean + Sqrt[(lambda + nx)*P_]
    //ActualMean - Sqrt[(lambda + nx)*P_]
    Xsig_aug.col(0) = x_aug;
    for(int i = 0; i < n_aug_; i++)
    {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*P_sqrt.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*P_sqrt.col(i);
    }

  //3.PREDICT SIGMA POINTS:---------------------------------------------
    //Retrieve generated sigma points from each column and predict them
    for(int i =0; i<2*n_aug_+1; i++)
    {
      //Retrieve generated sigma points
      double px = Xsig_aug(0, i);
      double py = Xsig_aug(1, i);
      double v = Xsig_aug(2, i);
      double yaw = Xsig_aug(3, i);
      double yawd = Xsig_aug(4, i);
      double nu_a = Xsig_aug(5, i);
      double nu_yawdd = Xsig_aug(6, i);

      
      double pxPred, pyPred;

      //check if yawd is 0
      if(fabs(yawd) > 0)
      {
        pxPred = px + v / yawd * (sin(yaw + yawd *delta_t) - sin(yaw));
        pyPred = py + v / yawd * (-1 *cos(yaw + yawd * delta_t) + cos(yaw));
      }
      else
      {
        pxPred = px + v * cos(yaw) * delta_t;
        pyPred = py + v * sin(yaw) * delta_t;
      }

      double vPred = v; //constant velocity model
      double yawPred = yaw + yawd*delta_t;
      double yawdPred = yawd; //constant Turn Rate Model

      //Since we havent modeled the noise yet, we must add the effect of noise also with the predcition
      pxPred += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
      pyPred += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
      vPred += delta_t * nu_a;
      yawPred += 0.5 * delta_t * delta_t * nu_yawdd;
      yawdPred += delta_t * nu_yawdd;

      //Push the predicted signma points in Matrix
      Xsig_pred_(0,i) = pxPred;
      Xsig_pred_(1,i) = pyPred;
      Xsig_pred_(2,i) = vPred;
      Xsig_pred_(3,i) = yawPred;
      Xsig_pred_(4,i) = yawdPred;
        
    }

    //4.GENERATE MEAN & COVARIANCE FROM PREDICTED SIGMA POINTS ---------------------------------------------
      //4.1 Get the Mean, using the weights which we set initially and update x_
      x_.fill(0.0);
      for(int i =0; i<2*n_aug_+1; i++)
      {
        x_ = x_+weights_(i)*Xsig_pred_.col(i);
      }
      //4.2 Get the State Covariance Matrix
      //Sum of wi * (Xi - Xmean)*(Xi - Xmean)^T
      P_.fill(0.0);
      for(int i =0; i<2*n_aug_+1; i++)
      {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        x_diff(3) = NormalizeAngleBetwnPi(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
      }  
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  //5. LiDAR BASED UPDATE---------------------------------------------
    //5.1 - Measurement Prediction
    //LiDAR measures X and Y only
    u_int n_z = 2;
    //Matrix for Sigma points considering measurement dimension
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1); //Know the size

    //Reuse the predicted Sigma Points as sigma points in measurement space
    //Since Lidar detects only two X and Y- Trim it down to two elevant rows
    Zsig = Xsig_pred_.topRows(2);

    //Pass into Measuremnet prediction and generate Mean Z and covariance S
    //Note: This should not be confused with the same way we did in Prediction, as here the state
    //dimension is reduced to 2 which affects Mean and Covriance
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
    {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //Create Covariance Matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2*n_aug_ + 1; ++i) 
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i)* z_diff * z_diff.transpose();
    }

    //Skip augmentation using noise as noise in Measurement is purely additive
    //And Standard deviations are already provided by Manufacturers itself
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<  std_laspx_ * std_laspx_, 0,
        0, std_laspy_  * std_laspy_;
    S = S + R;

    //Now we have Predicted Measurement Zpred and Covariance S!!
    //ALready we had computed
    //State Mean X and Covariance P!!
    //Next comes the real measurement z from sensor

    //5.2 LiDAR measurement UPDATE:
    //Xupd = X + K(z - Zpred), where K is Kalman Gain
    //Pupd = P - KSK', K' is transpose
    //But K = TcS^-1, where Tc is coss correlation b/w sigma points in state space and measurement space
    //And Tc is Sum of (Wi * (XsigPred - X_)*(ZsigPred-Zpred))

    VectorXd z = meas_package.raw_measurements_;
    //5.2.1 Compute Cross-Correleation 
    MatrixXd Tc = MatrixXd(n_x_,n_z);
    Tc.fill(0.0);
    for (int i = 0; i < 2*n_aug_ + 1; i++) 
    {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      x_diff(3) = NormalizeAngleBetwnPi(x_diff(3));
      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //5.2.2 - Compute Kalman Gain
    MatrixXd K = Tc * S.inverse();
    //5.2.3 - The difference with actual measurement
    //and usual Kalman Formulae.

    x_ = x_ + K*(z - z_pred);
    P_ = P_ - K*S*K.transpose();

    double laser_NIS = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
    // std::cout << laser_NIS << "," << std::endl;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //6. RADAR BASED UPDATE------------------------------------------------
    //6.1 Measurement Prediction
    //Radar can measure rhi, phi and rho_dot
    //So
    int n_z = 3;
    //Matrix for Sigma points considering measurement dimension
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    //Here the conversion of state->radar space happens.
    
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
    {
      double px = Xsig_pred_(0, i);
      double py = Xsig_pred_(1, i);
      double v = Xsig_pred_(2, i);
      double yaw = Xsig_pred_(3, i);

      double vx = v*cos(yaw);
      double vy = v*sin(yaw);

      //The general formualae for conversion from state -> radar space in order rho, phi, rho_dot
      Zsig(0, i) = sqrt(px * px + py * py);
      Zsig(1, i) = atan2(py, px);
      Zsig(2, i) = (px * vx + py * vy) / sqrt(px * px + py*py);
    }

    //Pass into Measuremnet prediction and generate Mean Z and covariance S
    //Note: This should not be confused with the same way we did in Prediction, as here the state
    //dimension is reduced to 2 which affects Mean and Covriance
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) 
    {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //Create Covariance Matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2*n_aug_ + 1; i++) 
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = NormalizeAngleBetwnPi(z_diff(1));
        S = S + weights_(i)* z_diff * z_diff.transpose();
    }
    

    //Skip augmentation using noise as noise in Measurement is purely additive
    //And Standard deviations are already provided by Manufacturers itself
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<  std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0,std_radrd_ * std_radrd_;
    
    S = S + R;

    //Now we have Predicted Measurement Zpred and Covariance S!!
    //ALready we had computed
    //State Mean X and Covariance P!!
    //Next comes the real measurement z from sensor

    //6.2 RADAR measurement UPDATE:
      //Xupd = X + K(z - Zpred), where K is Kalman Gain
      //Pupd = P - KSK', K' is transpose
      //But K = TcS^-1, where Tc is coss correlation b/w sigma points in state space and measurement space
      //And Tc is Sum of (Wi * (XsigPred - X_)*(ZsigPred-Zpred))

      VectorXd z = meas_package.raw_measurements_;
      //6.2.1 Compute Cross-Correleation 
      MatrixXd Tc = MatrixXd(n_x_,n_z);
      Tc.fill(0.0);
      for (int i = 0; i < 2*n_aug_ + 1; i++) 
      {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //@1 lies phi in radar z
        z_diff(1) = NormalizeAngleBetwnPi(z_diff(1));
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //@3 lies yaw in state x
        x_diff(3) = NormalizeAngleBetwnPi(x_diff(3));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
      }

      //6.2.2 - Compute Kalman Gain
      MatrixXd K = Tc * S.inverse();
      //6.2.3 - The difference with actual measurement
      //and usual Kalman Formulae.
      VectorXd zMeasDiff =  z - z_pred;
      zMeasDiff(1) = NormalizeAngleBetwnPi(zMeasDiff(1));

      x_ = x_ + K*zMeasDiff;

      P_ = P_ - K*S*K.transpose();

      double radar_NIS = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
      // std::cout << radar_NIS << "," << std::endl;



}