#include "library.h"

void set_z(std::string line, Eigen::VectorXd &z_k_observation_vector, Eigen::VectorXd &d_k)
{
    Eigen::VectorXd v(3);
    Eigen::VectorXd omega(3);
    v << 40, 25, 16.3;
    omega << 0, 0, 7.161972;
    char delimiter = ';';

    std::stringstream stream(line);
    std::string teta, phi, psi, delta_teta, delta_phi, delta_psi, teta_ist, phi_ist, psi_ist, teta_err, phi_err, psi_err, x, y, z;
    getline(stream, teta, delimiter);
    getline(stream, phi, delimiter);
    getline(stream, psi, delimiter);
    getline(stream, delta_teta, delimiter);
    getline(stream, delta_phi, delimiter);
    getline(stream, delta_psi, delimiter);
    getline(stream, teta_ist, delimiter);
    getline(stream, phi_ist, delimiter);
    getline(stream, teta_err, delimiter);
    getline(stream, phi_err, delimiter);
    getline(stream, psi_err, delimiter);
    getline(stream, x, delimiter);
    getline(stream, y, delimiter);
    getline(stream, z, delimiter);

    z_k_observation_vector(0) = v(0); z_k_observation_vector(1) = v(1); z_k_observation_vector(2) = v(2);
    z_k_observation_vector(3) = atof(teta.c_str()) * M_PI / 180; z_k_observation_vector(4) = atof(phi.c_str()) * M_PI / 180; z_k_observation_vector(5) = atof(psi.c_str()) * M_PI / 180;
    z_k_observation_vector(6) = omega(0); z_k_observation_vector(7) = omega(1); z_k_observation_vector(8) = omega(2);
    d_k(0) = atof(delta_teta.c_str()) * M_PI / 180; d_k(1) = atof(delta_phi.c_str()) * M_PI / 180; d_k(2) = atof(delta_psi.c_str()) * M_PI / 180;
}
