#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
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
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4;
  
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
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  lambda_aug_ = 3 - n_aug_;
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_aug_ / (lambda_aug_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_+lambda_aug_);
    weights_(i) = weight;
  }
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  NIS_lidar_above_thres = 0;
  NIS_radar_above_thres = 0;
  NIS_lidar_measurement_num = 0;
  NIS_radar_measurement_num = 0;
  NIS_lidar_thres_list = vector<double>(N, NIS_lidar_thres);
  NIS_radar_thres_list = vector<double>(N, NIS_radar_thres);
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
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float r = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      // float rdot = meas_package.raw_measurements_(2);
      float px = r * cos(phi);
      float py = r * sin(phi);
      x_ << px, py, 0, 0, 0;
      P_(0,0) = (std_radr_ * std_radr_); // maximum uncertainty of x occurs when we assume cos(phi) = 1, i.e. object is head-on or perpendicular to radar
      P_(1,1) = (std_radr_ * std_radr_); // same reasoning as above
      // worst case uncertainty in velocity happens when we assume the tangential velocity = radial velocity as measured by the radar
      P_(2,2) = (std_radrd_ * std_radrd_);
      P_(3,3) = std_radphi_ * std_radphi_;
      P_(4,4) = 1.57 / 2; // upper bound of uncertainty in phi dot should be used here

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float x = meas_package.raw_measurements_(0); 
      float y = meas_package.raw_measurements_(1);
      x_ <<  x, y, 0, 0, 0;
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspy_ * std_laspy_;
      P_(2,2) = 1; // default to 1 when we don't have a better guess
      P_(3,3) = 1;
      P_(4,4) = 1.57 / 2; // upper bound of uncertainty in phi dot

    }

    // P_ = MatrixXd::Identity(5,5);
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    
    return;
  }

  // Predict
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }
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

  // Generate augmented sigma points
  // Create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // Create augmented state covariance matric
  MatrixXd P_aug = MatrixXd(7,7);

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_aug_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_aug_ + n_aug_) * L.col(i);
  }

  // predict sigma points
  double t2 = delta_t * delta_t;
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_aug.col(i)(0);
    double py = Xsig_aug.col(i)(1);
    double v = Xsig_aug.col(i)(2);
    double psi = Xsig_aug.col(i)(3);
    double psi_dot = Xsig_aug.col(i)(4);
    double nu_a = Xsig_aug.col(i)(5);
    double nu_psi_dot_dot = Xsig_aug.col(i)(6);
    double new_px, new_py, new_v, new_psi, new_psi_dot;
    VectorXd new_x = VectorXd(5);
    if (fabs(psi_dot) < 0.001) {
        new_px = px + v*cos(psi)*delta_t + 0.5*t2*cos(psi)*nu_a;
        new_py = py + v*sin(psi)*delta_t + 0.5*t2*sin(psi)*nu_a;
        new_v = v + delta_t*nu_a;
        new_psi = psi + psi_dot * delta_t + 0.5 * delta_t * delta_t * nu_psi_dot_dot;
        new_psi_dot = psi_dot + 0 + delta_t * nu_psi_dot_dot;
        new_x << new_px, new_py, new_v, new_psi, new_psi_dot;
        Xsig_pred_.col(i) = new_x;
    } else {
        new_px = px + v/psi_dot * (sin(psi + psi_dot*delta_t) - sin(psi)) + 0.5 * t2 * cos(psi) * nu_a;
        new_py = py + v/psi_dot * (-cos(psi + psi_dot*delta_t) + cos(psi)) + 0.5 * t2 * sin(psi) * nu_a;
        new_v = v + delta_t*nu_a;
        new_psi = psi + psi_dot*delta_t + 0.5*t2*nu_psi_dot_dot;
        new_psi_dot = psi_dot + delta_t*nu_psi_dot_dot;
        new_x << new_px, new_py, new_v, new_psi, new_psi_dot;
        Xsig_pred_.col(i) = new_x;
    }
  }

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    NormalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

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

  int n_z = 2; // lidar can measure x, y
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z = meas_package.raw_measurements_;

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  S = S + R;

  // Cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // Update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate Lidar NIS
  float eta = z_diff.transpose() * S.inverse() * z_diff;
  if (eta > NIS_lidar_thres) NIS_lidar_above_thres++;
  NIS_lidar_measurement_num++;

  NIS_lidar_history.push_back(eta);
  if (NIS_lidar_measurement_num % 30 == 0 && NIS_lidar_measurement_num != 0) DrawNIS(true); // redraw for every 30 new datapoints
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
  int n_z = 3; // radar can measure r, phi and r_dot
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z = meas_package.raw_measurements_;

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);       // r
    Zsig(1,i) = atan2(p_y, p_x);                   // phi
    if (fabs(Zsig(0,i) < 0.01)) {                  // rdot, handle divisionByZero
      Zsig(2,i) = (p_x * v1 + p_y * v2) / 0.01;
    } else {
      Zsig(2,i) = (p_x * v1 + p_y * v2) / Zsig(0,i);
    }
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    NormalizeAngle(z_diff(1));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  // Cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    NormalizeAngle(z_diff(1));

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;
  NormalizeAngle(z_diff(1));

  // Update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate Radar NIS
  float eta = z_diff.transpose() * S.inverse() * z_diff;
  if (eta > NIS_radar_thres) NIS_radar_above_thres++;
  NIS_radar_measurement_num++;

  NIS_radar_history.push_back(eta);
  if (NIS_radar_measurement_num % 30 == 0 && NIS_radar_measurement_num != 0) DrawNIS(false); // redraw for every 30 new datapoints
}

void UKF::NormalizeAngle(double& phi) {
  phi = atan2(sin(phi), cos(phi));
}

float UKF::GetRadarNISPercentageAboveThres() {
  if (NIS_radar_measurement_num == 0) {
    return NIS_radar_above_thres;
  } else {
    return (NIS_radar_above_thres * 1.0f / NIS_radar_measurement_num);
  }
}

float UKF::GetLidarNISPercentageAboveThres() {
  if (NIS_lidar_measurement_num == 0) {
    return NIS_lidar_above_thres;
  } else {
    return (NIS_lidar_above_thres * 1.0f / NIS_lidar_measurement_num);
  }
}

void UKF::DrawNIS(bool isLidar) {
  if (isLidar) {
    plt::subplot(1,2,0);
    plt::ylim(0,20);
    plt::plot(NIS_lidar_history, "b");
    plt::plot(NIS_lidar_thres_list, "r--");
    plt::title("Lidar NIS");
  } else {
    plt::subplot(1,2,1);
    plt::ylim(0,20);
    plt::plot(NIS_radar_history, "b");
    plt::plot(NIS_radar_thres_list, "r--");
    plt::title("Radar NIS");    
  }
  plt::draw();
  plt::pause(0.0005);
}