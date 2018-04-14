#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	//cout << "est size: "<< estimations.size() << endl;
	assert(estimations.size() > 0 && "Estimation vector cannot be zero");
	assert(estimations.size() == ground_truth.size()  && "Estimation and Ground Truth vector cannot be different");

	VectorXd residual;
	VectorXd residual_square;
	VectorXd total;

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		
		//cout << "est: " << estimations[i] << endl;
		residual = (estimations[i] - ground_truth[i]);
		//cout << "residuals " << i <<" : " << residual << endl;
		residual_square = residual.array()*residual.array();
		//cout << "residual_square " << i <<" : " << residual_square << endl;
		rmse += residual_square;
		//cout << "total " << i <<" : " << rmse << endl;
	}	
	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}