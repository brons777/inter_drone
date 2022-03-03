#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#pragma once
#include <iostream>
#include "Eigen"
#include <math.h>
#include <fstream>
#include <string>
#define M_PI 3.14159265358979323846

struct PointLa
{
	double X;
	double Y;
	double Z;
	double XRoll;	// ���� �� ��� �
	double YPitch;	// ������ �� ��� Y
	double ZYaw;		// �������� �� ��� Z
	double vx;  // �������� ��������
	double vy;
	double vz;
	double wXRoll; // ������� �������� �� ��� �
	double wYPitch; // ������� �������� �� ��� Y
	double wZYaw; // ������� �������� �� ��� Z
	int m_time;
};

void mat_set(Eigen::MatrixXd& A_k_minus_1, Eigen::VectorXd& process_noise_v_k_minus_1, Eigen::VectorXd& k1, Eigen::MatrixXd& H_k, Eigen::VectorXd& k,
	Eigen::VectorXd& sensor_noise_w_k, Eigen::VectorXd& state_estimate_k_minus_1, Eigen::VectorXd& control_vector_k_minus_1, Eigen::VectorXd& k2, double dt);

void set_z(Eigen::VectorXd& z_k_observation_vector, Eigen::VectorXd& d_k, PointLa La_Curr, PointLa La_Pred);

class TVisualNavigation
{
private:
	PointLa PointLa_Pred; // ������ ��������� ����������
	PointLa PointLa_Curr; // ������ ��������� �������
	cv::Mat CameraMatrix; // ������� ���������� ���������� ������
	cv::Mat DistCoeff;    // ������ ������������� ���������
	
	int SIZEBLOCK = 32;  // ������ ���������� ����������� (SIZEBLOCK*SIZEBLOCK)
	int SIZESEARCH = 64; // ������ �������� ����������� (2*SIZESEARCH)*(2*SIZESEARCH)
	int COUNTBLOCK = 6;  // ���������� ������� ����� �� ��������� � �����������
	int COUNTBLOCKGUT = COUNTBLOCK*COUNTBLOCK/2; // ����� �� ����������� ���������� ����� � ���������� �����
	double POROG_CORRELATION = 0.7; // ����� �� �������������� ������� ���. ������ 1.0
	double POROG_DXDY = 6; // ����� �� ����������� ������ �������� �������� � �������� ����� �������

public:
	// ����������� �� ���������, ��������� � ���������� ��� 1 � 2 ����� �������� ������� �������� ���������
	// ������� ���������� ���������� ����� �������� ��� ����������� �������� 
	TVisualNavigation();
	// �����������
	TVisualNavigation(cv::Mat cm, cv::Mat dist_coeffs);
	TVisualNavigation(cv::Mat cm, cv::Mat dist_coeffs, PointLa PointLa1, PointLa PointLa2);
	// �������� ������ ��������� ��� ������ ������� �����
	void SetPointLaPred(PointLa PointLa1);
	// �������� ������ ��������� ��� ������ ������� �����
	void SetPointLaCurr(PointLa PointLa1);
	// �������� ��������� ��� ������������ ����������� ������
	// sizeblock - ������ �������� ����� ����������� ������ 16, 32, 64 ��������
	// sizesearch - ������ ������� ������  ����-����� 32, 64, 96 ��������
	// countblock - ���������� ������� ������ �� ����������� � ���������, �.�. ����� ���-�� ����� countblock*countblock
	// porog - ����� �� �������� �������������� ������� 0,5....0,75
	void SetParamOpticalFlow(int sizeblock, int sizesearch, int countblock, double porog);
	// ���������� ������ ��������� ��� ������ ������� �����
	PointLa GetPointLaPred();
	// ���������� ������ ��������� ��� ������ ������� �����
	PointLa GetPointLaCurr();
	// ��������� ���������� ������� ���������� ���������� ������
	void SetCameraMatrix(cv::Mat cm);
	// ������� ������� ���������� ���������� ������
	cv::Mat GetCameraMatrix();
	// ��������� ���������� ������� ������������� ���������
	void SetDistCoeffs(cv::Mat cm);
	// ������� ������� ������������� ���������
	cv::Mat GetDistCoeffs();
	// ���������� ����������� ������ �� ���� �����������
	// ������� ���������:
	// im1 - ������ �����������, im2 - ������ �����������
	// �������� ���������:
	// src_pts - ���������� ������� ����� ��������������� ������� ����������� (��� ������������ ���������)
	// dst_pts - ���������� ������� ����� ��������������� ������� ����������� (��� ������������ ���������)
	int fun_build_flow_correlation(cv::Mat im1, cv::Mat im2, std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts);
	// ������ ������� ��������� ��� ������� ���������
	// �� ������ ��������� ������� ����� (src_pts, dst_pts) � ������� ��������� ��� ������ ������� (PointLa_Pred)
	// time1 - ����� ����������� ������� �����
	// time2 - ����� ����������� ������� �����
	PointLa funRealPoints(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts, int time1, int time2);
	// ������������ ����������� ������ � ��������� ����
	void DrawOpticalFlow(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts);
	// ���������� ��������� ������� ��������� � �������� ���� (PointLa_Pred)
	void SaveFileVector(const char* filename);
	// ������������� ��������� ������� ����� �������� ������������� ���������
	// src_pts - ���������� ������� ����� ��������������� ������� �����������
	// dst_pts - ���������� ������� ����� ��������������� ������� �����������
	void CorrectDistortion(std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts);

private:
	int fun_delete_anomaly(std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts);
	cv::Mat selectRotationMat(cv::Mat R1, cv::Mat R2, int& num); // ������������ ������� �������� ����� �������
	cv::Mat selectRotationMat2(cv::Mat R1, cv::Mat R2, int& num); // ������������ ������� �������� ��� ������� �����
	cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f& theta);
	void GetRotation(double& ZYaw, double& YPitch, double& XRoll, cv::Mat M);
	cv::Mat checking_unit_matrix(cv::Mat M, double e);
	int my_px_to_obj(std::vector<cv::Point2f> keypointsUV, cv::Mat r, cv::Vec3f position, cv::Mat& keypointsXYZ, double H);
	int my_prepare_data_for_lms(std::vector<cv::Point2f> dst_pts, cv::Mat keypointsXYZ, cv::Mat r, cv::Mat& coefficients);
	int my_construct_matrices_lsm(cv::Mat coefficients, cv::Vec3f& position);
};

class my_ekf
{
    Eigen::MatrixXd A_k_minus_1;
    Eigen::VectorXd process_noise_v_k_minus_1;
    Eigen::VectorXd k1;
    Eigen::MatrixXd Q_k;
    Eigen::MatrixXd H_k;
    Eigen::VectorXd k;
    Eigen::VectorXd v;
    Eigen::VectorXd omega;
    Eigen::VectorXd temp;
    Eigen::MatrixXd R_k;
    Eigen::VectorXd sensor_noise_w_k;
    Eigen::VectorXd state_estimate_k_minus_1;
    Eigen::VectorXd k2;
    Eigen::VectorXd control_vector_k_minus_1;
    Eigen::MatrixXd P_k_minus_1;
	std::ofstream myfile;

public:

    my_ekf(Eigen::MatrixXd A_k_minus_1, Eigen::VectorXd process_noise_v_k_minus_1, Eigen::VectorXd k1, Eigen::MatrixXd H_k, Eigen::VectorXd k,
        Eigen::VectorXd sensor_noise_w_k, Eigen::VectorXd state_estimate_k_minus_1,
        Eigen::VectorXd k2, Eigen::VectorXd control_vector_k_minus_1)
    {
        this->A_k_minus_1 = A_k_minus_1;
        this->process_noise_v_k_minus_1 = process_noise_v_k_minus_1;
        this->Q_k = k1.asDiagonal();
        this->H_k = H_k;
        this->R_k = k.asDiagonal();
        this->sensor_noise_w_k = sensor_noise_w_k;
        this->state_estimate_k_minus_1 = state_estimate_k_minus_1;
        this->P_k_minus_1 = k2.asDiagonal();
        this->control_vector_k_minus_1 = control_vector_k_minus_1;
		myfile.open("C:\\Nikita\\2021\\704\\inter_drone\\git\\kalman_filter\\kalman_filter\\result4.csv", std::ios::app);
		myfile << "x; y; z; vx; vy; vz; phi; teta; psi; omegaX; omegaY; omegaZ\n";
		myfile.close();
		Eigen::MatrixXd rez(12, 10000);

    }

    Eigen::MatrixXd getB(double teta, double phi, double psi, double deltateta, double deltaphi, double deltapsi, double vx, double vy, double vz,
        double wx, double wy, double wz)
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

    int ekf(Eigen::VectorXd z_k_observation_vector, Eigen::VectorXd state_estimate_k_minus_1, Eigen::VectorXd control_vector_k_minus_1,
        Eigen::MatrixXd P_k_minus_1, Eigen::VectorXd d_k, Eigen::MatrixXd A_k_minus_1, Eigen::VectorXd process_noise_v_k_minus_1, Eigen::MatrixXd Q_k,
        Eigen::VectorXd sensor_noise_w_k, Eigen::MatrixXd H_k, Eigen::MatrixXd R_k, Eigen::VectorXd& state_estimate_k, Eigen::MatrixXd& P_k)
    {
        Eigen::VectorXd measurement_residual_y_k(12);
        Eigen::MatrixXd S_k(12, 12);
        Eigen::MatrixXd K_k(12, 12);

        state_estimate_k = A_k_minus_1 * (state_estimate_k_minus_1)+getB(state_estimate_k_minus_1(7), state_estimate_k_minus_1(6),
            state_estimate_k_minus_1(8), d_k(1), d_k(0), d_k(2), state_estimate_k_minus_1(3), state_estimate_k_minus_1(4),
            state_estimate_k_minus_1(5), state_estimate_k_minus_1(9), state_estimate_k_minus_1(10),
            state_estimate_k_minus_1(11)) * (control_vector_k_minus_1)+(process_noise_v_k_minus_1);
        P_k = A_k_minus_1 * P_k_minus_1 * A_k_minus_1.transpose() + Q_k;

        measurement_residual_y_k = z_k_observation_vector - ((H_k * state_estimate_k) + (sensor_noise_w_k));

        S_k = H_k * P_k * H_k.transpose() + R_k;
        K_k = P_k * H_k.transpose() * S_k.inverse();
        state_estimate_k = state_estimate_k + (K_k * measurement_residual_y_k);
        P_k = P_k - (K_k * H_k * P_k);
        return 0;

    }

    void process(Eigen::MatrixXd& rez, Eigen::VectorXd z_k_observation_vector, Eigen::VectorXd d_k, int l)
    {
        Eigen::VectorXd state_estimate_k(12);
        Eigen::MatrixXd P_k(12, 12);
		

        my_ekf::ekf(z_k_observation_vector, state_estimate_k_minus_1, control_vector_k_minus_1, P_k_minus_1, d_k,
            A_k_minus_1, process_noise_v_k_minus_1, Q_k, sensor_noise_w_k, H_k, R_k, state_estimate_k, P_k);
        rez.col(l) = state_estimate_k;
        state_estimate_k_minus_1 = state_estimate_k;
        P_k_minus_1 = P_k;
		//std::cout << std::endl << rez.col(0) << std::endl;
    }

	void save_ekf(Eigen::MatrixXd& rez, int l)
	{
		myfile.open("C:\\Nikita\\2021\\704\\inter_drone\\git\\kalman_filter\\kalman_filter\\result4.csv", std::ios::app);
		myfile << std::to_string(rez(0, l)) + ";" << std::to_string(rez(1, l)) + ";" << std::to_string(rez(2, l)) + ";" << std::to_string(rez(3, l)) + ";" <<
			std::to_string(rez(4, l)) + ";" << std::to_string(rez(5, l)) + ";" << std::to_string(rez(6, l) * 180 / M_PI) + ";" << std::to_string(rez(7, l) * 180/M_PI) + ";" <<
			std::to_string(rez(8, l) * 180 / M_PI) + ";" << std::to_string(rez(9, l)) + ";" << std::to_string(rez(10, l)) + ";" << std::to_string(rez(11, l)) + "\n";
		myfile.close();

	}

};