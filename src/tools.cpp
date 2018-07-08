#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
  rmse = VectorXd(4);
  rmse << 0,0,0,0;
  prevVx = 0;
  prevVy = 0;
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  assert(estimations.size() > 0);
  assert(ground_truth.size() == estimations.size());

  int lastIdx = estimations.size() - 1;
  // cout << "est: " << endl << estimations[lastIdx] << endl;
  // cout << "gt:  " << endl << ground_truth[lastIdx] << endl;

  VectorXd sumOfSquares = (rmse.array().square() * (estimations.size() -1)) + ((estimations[lastIdx] - ground_truth[lastIdx]).array() * (estimations[lastIdx] - ground_truth[lastIdx]).array());
  rmse = sumOfSquares / estimations.size();
  rmse = rmse.array().sqrt();

  // cout << "RMSE = " << endl << rmse << endl;

  // cout << "Change in vx = " << ground_truth[lastIdx](2) - prevVx << endl;
  // cout << "Change in vy = " << ground_truth[lastIdx](3) - prevVy << endl;

  prevVx = ground_truth[lastIdx](2);
  prevVy = ground_truth[lastIdx](3);

  return rmse;
}