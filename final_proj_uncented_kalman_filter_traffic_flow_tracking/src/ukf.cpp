#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // set vector for weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  double weight = 0.5 / (lambda_ + n_aug_);
  weights_(0) = weight_0;

  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  
    weights_(i) = weight;
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    UpdateLidar(meas_package);
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    UpdateRadar(meas_package);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix with predicted sigma points in state space
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred = Xsig_pred_;

  // create vector for predicted state mean
  VectorXd x = VectorXd(n_x_);
  x = x_;

  // create matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x_,n_x_);
  P = P_;

  // create matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // create vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // create matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);

  // call the function the assign the predicted measurement and measurement covariance
  PredictRadarMeasurement(&z_pred, &S, &Zsig);

  // create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;

  // calculate cross correlation matrix
  MatrixXd T = MatrixXd(n_x_, n_z);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    T += weights_(i) * (Xsig_pred.col(i) - x) * (Zsig.col(i) - z_pred).transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K = T * S.inverse();

  // update state mean and covariance matrix
  x += K * (z - z_pred);
  P -= K * S * K.transpose();

  // print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  // write result
  x_ = x;
  P_ = P;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // set state
  VectorXd x = VectorXd(n_x_);
  x = x_;

  // create covariance matrix
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P = P_;

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(n_x_) = x;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_+1) = 0.0;

  // create augmented covariance matrix
  P_aug.fill(0.0); // initialize all elements to 0.0
  P_aug.topLeftCorner(n_x_, n_x_) = P;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd square_root_P_aug = P_aug.llt().matrixL();

  // create augmented sigma points
  // col 0:
  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    // set sigma points as columns of matrix Xsig
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda_ + n_aug_) * square_root_P_aug.col(i);
    Xsig_aug.col(n_aug_+i+1) = x_aug - sqrt(lambda_ + n_aug_) * square_root_P_aug.col(i);
  }

  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_in) {

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug = *Xsig_in;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  double delta_t = 0.1; // time diff in sec

  Xsig_pred.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // predict sigma points
    VectorXd x_col = VectorXd(n_x_);
    x_col.fill(0.0);

    double px            =  Xsig_aug(0, i);
    double py            =  Xsig_aug(1, i);
    double v             =  Xsig_aug(2, i);
    double phi           =  Xsig_aug(3, i);
    double phi_dot       =  Xsig_aug(4, i);
    double niu_a         =  Xsig_aug(5, i);
    double niu_phi_ddot  =  Xsig_aug(6, i);

    // avoid division by zero
    if (fabs(phi_dot) < 0.001) // Note: must use fabs() for floating numbers !!!!!!!!!!!!!!!!!!!!!!
                               //       abs(0.5) = 0, fabs(0.5) = 0.5
    {
        // Note: "1/2" for floating numbers must be written as "1.0/2.0" !!!!!!!!!!!!!!!!!!!!!!!
        x_col(0) = px      + v * cos(phi) * delta_t + 1.0 / 2.0 * delta_t * delta_t * cos(phi) * niu_a;
        x_col(1) = py      + v * sin(phi) * delta_t + 1.0 / 2.0 * delta_t * delta_t * sin(phi) * niu_a;
        x_col(2) = v       + delta_t * niu_a;
        x_col(3) = phi     + 0.5 * delta_t * delta_t * niu_phi_ddot;
        x_col(4) = phi_dot + delta_t * niu_phi_ddot;
    }
    else
    {
        // Note: "1/2" for floating numbers must be written as "1.0/2.0" !!!!!!!!!!!!!!!!!!!!!!!
        x_col(0) = px      + v / phi_dot * ( sin(phi + phi_dot * delta_t) - sin(phi) ) + 1.0 / 2.0 * delta_t * delta_t * cos(phi) * niu_a;
        x_col(1) = py      + v / phi_dot * ( -cos(phi + phi_dot * delta_t) + cos(phi) )  + 1.0 / 2.0 * delta_t * delta_t * sin(phi) * niu_a;
        x_col(2) = v       + delta_t * niu_a;
        x_col(3) = phi     + phi_dot * delta_t + 1.0 / 2.0 * delta_t * delta_t * niu_phi_ddot;
        x_col(4) = phi_dot + delta_t * niu_phi_ddot;
    }

    // write predicted sigma points into right column
    Xsig_pred.col(i) = x_col;
  }

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  Xsig_pred_ = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred = Xsig_pred_;

  // create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // predict state mean
    x += weights_(i) * Xsig_pred.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Useful check: if the phi stays in [-pi, pi]
    // angle normalization
    VectorXd x_diff = Xsig_pred.col(i) - x;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    // predict state covariance matrix
    P += weights_(i) * x_diff * x_diff.transpose();
  }

  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  // write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig) {

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred = Xsig_pred_;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_tmp = VectorXd(n_z);
    VectorXd x_pred = Xsig_pred.col(i);
    double px       =  x_pred(0);
    double py       =  x_pred(1);
    double v        =  x_pred(2);
    double psi      =  x_pred(3);
    double psi_dot  =  x_pred(4);
    // transform sigma points into measurement space
    z_tmp(0) = sqrt(pow(px, 2) + pow(py, 2));
    z_tmp(1) = atan2(py, px);
    z_tmp(2) = (px * cos(psi) * v + py * sin(psi) * v ) / z_tmp(0);

    // normalize the angle phi (z_tmp(1))
    while (z_tmp(1) > M_PI) z_tmp(1) -= 2 * M_PI;
    while (z_tmp(1) < -M_PI) z_tmp(1) += 2 * M_PI;

    // calculate mean predicted measurement
    z_pred += weights_(i) * z_tmp;

    // store info in Zsig
    Zsig.col(i) = z_tmp;
  }

  // predicted covariance matrix
  MatrixXd R = MatrixXd(3, 3);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  S += R;

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // calculate innovation covariance matrix S
    S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose();
  }

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
  *Z_sig = Zsig;
}