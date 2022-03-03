#include "visual_navigation.h"

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

	
	process_noise_v_k_minus_1 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.04, 0.04, 0.04, 0.005, 0.005, 0.005;
	k1 << 1, 1, 100, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001;

	H_k <<
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	k << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001;
	sensor_noise_w_k << 0.05, 0.05, 0.05, 1, 1, 1, 0.02, 0.02, 0.02, 0.03, 0.05, 0.02;
	
	state_estimate_k_minus_1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	control_vector_k_minus_1 << 1, 1, 1, 1, 1, 1;
	k2 << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;
	

}

void set_z(Eigen::VectorXd& z_k_observation_vector, Eigen::VectorXd& d_k, PointLa La_Curr, PointLa La_Pred)
{

	z_k_observation_vector(0) = La_Curr.X; z_k_observation_vector(1) = La_Curr.Y; z_k_observation_vector(2) = La_Curr.Z;
	z_k_observation_vector(3) = La_Curr.vx; z_k_observation_vector(4) = La_Curr.vy; z_k_observation_vector(5) = La_Curr.vz;
	z_k_observation_vector(6) = La_Curr.XRoll; z_k_observation_vector(7) = La_Curr.YPitch; z_k_observation_vector(8) = La_Curr.ZYaw;
	z_k_observation_vector(9) = La_Curr.wXRoll; z_k_observation_vector(10) = La_Curr.wYPitch; z_k_observation_vector(11) = La_Curr.wZYaw;

	d_k(0) = La_Curr.XRoll - La_Pred.XRoll; d_k(1) = La_Curr.YPitch - La_Pred.YPitch; d_k(2) = La_Curr.ZYaw - La_Pred.ZYaw;
}