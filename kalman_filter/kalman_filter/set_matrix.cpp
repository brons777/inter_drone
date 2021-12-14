#include "library.h"

void mat_set(Eigen::MatrixXd &A_k_minus_1, Eigen::VectorXd& process_noise_v_k_minus_1, Eigen::VectorXd& k1, Eigen::MatrixXd& H_k, Eigen::VectorXd& k, 
			Eigen::VectorXd& sensor_noise_w_k, Eigen::VectorXd& state_estimate_k_minus_1, 
			Eigen::VectorXd& control_vector_k_minus_1, Eigen::VectorXd& k2, double dt)
{
	
	A_k_minus_1 <<
		1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	
	process_noise_v_k_minus_1 << 1, 1, 1, 1, 1, 1, 10 * M_PI / 180, 10 * M_PI / 180, 1, 0.05, 0.05, 0.05;
	k1 << 10, 10, 10, 10, 10, 10, 1000, 100, 1000, 1, 1, 1;

	H_k <<
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	k << 1, 1, 1, 1, 1, 1, 1, 1, 1;
	sensor_noise_w_k << 2, 2, 5, 0.02, 0.02, 0.02, 0.03, 0.05, 0.02;
	
	state_estimate_k_minus_1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	control_vector_k_minus_1 << 1, 1, 1, 1, 1, 1;
	k2 << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;
	

}
