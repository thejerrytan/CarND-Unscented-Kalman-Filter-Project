#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
  rmse = VectorXd(4);
  rmse << 0,0,0,0;
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
  VectorXd sumOfSquares = (rmse.array().square() * (estimations.size() -1)) + ((estimations[lastIdx] - ground_truth[lastIdx]).array() * (estimations[lastIdx] - ground_truth[lastIdx]).array());
  rmse = sumOfSquares / estimations.size();
  rmse = rmse.array().sqrt();

  cout << "RMSE = " << rmse << endl;

  return rmse;
}