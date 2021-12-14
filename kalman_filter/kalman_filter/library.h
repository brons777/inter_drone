#pragma once
#include <iostream>
#include "Eigen"
#include <math.h>
#include <fstream>
#include <string>
#define M_PI 3.14159265358979323846

void mat_set(Eigen::MatrixXd& A_k_minus_1, Eigen::VectorXd& process_noise_v_k_minus_1, Eigen::VectorXd& k1, Eigen::MatrixXd& H_k, Eigen::VectorXd& k,
	Eigen::VectorXd& sensor_noise_w_k, Eigen::VectorXd& state_estimate_k_minus_1, Eigen::VectorXd& control_vector_k_minus_1, Eigen::VectorXd& k2, double dt);

void set_z(std::string line, Eigen::VectorXd& z_k_observation_vector, Eigen::VectorXd& d_k);


