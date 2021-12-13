#include <iostream>
#include "Eigen"
#include <math.h>
#include <fstream>
#include <string>
#define filename "book2.csv"
# define M_PI 3.14159265358979323846

Eigen::MatrixXd getB(double teta, double phi, double psi, double deltateta, double deltaphi, double deltapsi, double vx, double vy, double vz, double wx, double wy, double wz, double dt)
{
    Eigen::MatrixXd B(12, 6);
    B <<
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        (cos(teta) * cos(phi))* vx, (-sin(psi) * cos(teta))* vy, sin(teta)* vz, 0, 0, 0,
        (sin(psi) * cos(phi) + cos(psi) * sin(teta) * sin(phi))* vx, (cos(psi) * cos(phi) - sin(psi) * sin(teta) * sin(phi))* vy, (-cos(teta) * sin(phi))* vz, 0, 0, 0,
        (sin(psi) * sin(phi) - cos(psi) * sin(teta) * cos(phi))* vx, (cos(psi) * sin(phi) + sin(psi) * sin(teta) * cos(phi))* vy, (cos(teta) * cos(phi))* vz, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, (cos(deltateta) * cos(deltaphi))* wx, (-sin(deltapsi) * cos(deltateta))* wy, sin(deltateta)* wz,
        0, 0, 0, (sin(deltapsi) * cos(deltaphi) + cos(deltapsi) * sin(deltateta) * sin(deltaphi))* wx, (cos(deltapsi) * cos(deltaphi) - sin(deltapsi) * sin(deltateta) * sin(deltaphi))* wy, -cos(deltateta) * sin(deltaphi) * wz,
        0, 0, 0, (sin(deltapsi) * sin(deltaphi) - cos(deltapsi) * sin(deltateta) * cos(deltaphi))* wx, (cos(deltapsi) * sin(deltaphi) + sin(deltapsi) * sin(deltateta) * cos(deltaphi))* wy, cos(deltateta)* cos(deltaphi)* wz;
    return B;
}

int ekf(Eigen::MatrixXd z_k_observation_vector, Eigen::MatrixXd state_estimate_k_minus_1, Eigen::MatrixXd control_vector_k_minus_1,
    Eigen::MatrixXd P_k_minus_1, Eigen::MatrixXd d_k, Eigen::MatrixXd A_k_minus_1, Eigen::VectorXd process_noise_v_k_minus_1,
    Eigen::MatrixXd Q_k, Eigen::MatrixXd sensor_noise_w_k, Eigen::MatrixXd H_k, Eigen::MatrixXd R_k,
    Eigen::MatrixXd& state_estimate_k, Eigen::MatrixXd& P_k, double dt)
{
    Eigen::VectorXd measurement_residual_y_k(9);
    Eigen::MatrixXd S_k(9, 9);
    Eigen::MatrixXd K_k(12, 9);

    state_estimate_k =
        A_k_minus_1 * (state_estimate_k_minus_1)+(getB(state_estimate_k_minus_1(7), state_estimate_k_minus_1(6), state_estimate_k_minus_1(8),
            d_k(0), d_k(1), d_k(2), state_estimate_k_minus_1(3), state_estimate_k_minus_1(4), state_estimate_k_minus_1(5),
            state_estimate_k_minus_1(9), state_estimate_k_minus_1(10), state_estimate_k_minus_1(11), dt)) * (control_vector_k_minus_1)+(process_noise_v_k_minus_1);

    P_k = A_k_minus_1 * P_k_minus_1 * A_k_minus_1.transpose() + Q_k;

    measurement_residual_y_k = z_k_observation_vector - ((H_k * state_estimate_k) + (sensor_noise_w_k));
    S_k = H_k * P_k * H_k.transpose() + R_k;
    K_k = P_k * H_k.transpose() * S_k.inverse();
    state_estimate_k = state_estimate_k + (K_k * measurement_residual_y_k);
    P_k = P_k - (K_k * H_k * P_k);

    return 0;

}


int main()
{
    double dt = 1.0;
    Eigen::MatrixXd A_k_minus_1(12, 12);
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

    Eigen::VectorXd process_noise_v_k_minus_1(12);
    process_noise_v_k_minus_1 << 1, 1, 1, 1, 1, 1, 10 * M_PI / 180, 10 * M_PI / 180, 1, 0.05, 0.05, 0.05;


    Eigen::Matrix< double, 12, 1> k1;
    k1 << 10, 10, 10, 10, 10, 10, 1000, 100, 1000, 1, 1, 1;
    Eigen::Matrix< double, 12, 12> Q_k = k1.asDiagonal();


    Eigen::MatrixXd H_k(9, 12);
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

    Eigen::Matrix< double, 9, 1> k;
    k << 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Eigen::Matrix< double, 9, 9> R_k = k.asDiagonal();

    Eigen::VectorXd sensor_noise_w_k(9);
    sensor_noise_w_k << 2, 2, 5, 0.02, 0.02, 0.02, 0.03, 0.05, 0.02;

    Eigen::MatrixXd state_estimate_k(12, 1);
    Eigen::MatrixXd P_k(12, 12);

    Eigen::VectorXd v(3);
    v << 40, 25, 16.3;
    Eigen::VectorXd omega(3);
    omega << 0, 0, 7.161972;

    Eigen::VectorXd state_estimate_k_minus_1(12);
    state_estimate_k_minus_1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd control_vector_k_minus_1(6);
    control_vector_k_minus_1 << 1, 1, 1, 1, 1, 1;

    Eigen::Matrix< double, 12, 1> k2;
    k2 << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;
    Eigen::Matrix< double, 12, 12> P_k_minus_1 = k2.asDiagonal();

    Eigen::MatrixXd rez(12, 298);
    Eigen::VectorXd temp(12);
    temp << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    rez.col(0) = temp;
    Eigen::VectorXd z_k_observation_vector(9, 1);
    Eigen::VectorXd d_k(3, 1);


    int l = 1;
    std::ofstream myfile;
    myfile.open("result.csv");
    myfile << "x; y; z; vx; vy; vz; phi; teta; psi; omegaX; omegaY; omegaZ\n";
    std::ifstream work_file(filename);
    std::string line;
    char delimiter = ';';
    std::getline(work_file, line);
    std::getline(work_file, line);

    myfile << std::to_string(rez(0, 0)) + ";" << std::to_string(rez(1, 0)) + ";" << std::to_string(rez(2, 0)) + ";" << std::to_string(rez(3, 0)) + ";" <<
        std::to_string(rez(4, 0)) + ";" << std::to_string(rez(5, 0)) + ";" << std::to_string(rez(6, 0)) + ";" << std::to_string(rez(7, 0)) + ";" <<
        std::to_string(rez(8, 0)) + ";" << std::to_string(rez(9, 0)) + ";" << std::to_string(rez(10, 0)) + ";" << std::to_string(rez(11, 0)) + "\n";

    while (getline(work_file, line))
    {

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
        z_k_observation_vector(3) = atof(teta.c_str()); z_k_observation_vector(4) = atof(phi.c_str()); z_k_observation_vector(5) = atof(psi.c_str());
        z_k_observation_vector(6) = omega(0); z_k_observation_vector(7) = omega(1); z_k_observation_vector(8) = omega(2);
        d_k(0) = atof(delta_teta.c_str()); d_k(1) = atof(delta_phi.c_str()); d_k(2) = atof(delta_psi.c_str());

        ekf(z_k_observation_vector, state_estimate_k_minus_1, control_vector_k_minus_1, P_k_minus_1, d_k, A_k_minus_1, process_noise_v_k_minus_1,
            Q_k, sensor_noise_w_k, H_k, R_k, state_estimate_k, P_k, dt);

        rez.col(l) = state_estimate_k;
        state_estimate_k_minus_1 = state_estimate_k;
        P_k_minus_1 = P_k;
        myfile << std::to_string(rez(0, l)) + ";" << std::to_string(rez(1, l)) + ";" << std::to_string(rez(2, l)) + ";" << std::to_string(rez(3, l)) + ";" <<
            std::to_string(rez(4, l)) + ";" << std::to_string(rez(5, l)) + ";" << std::to_string(rez(6, l)) + ";" << std::to_string(rez(7, l)) + ";" <<
            std::to_string(rez(8, l)) + ";" << std::to_string(rez(9, l)) + ";" << std::to_string(rez(10, l)) + ";" << std::to_string(rez(11, l)) + "\n";
        l += 1;
    }
    work_file.close();
    myfile.close();


}
