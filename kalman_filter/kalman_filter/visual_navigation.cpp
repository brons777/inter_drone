#include "visual_navigation.h"
#include <fstream>


TVisualNavigation::TVisualNavigation()
{
	PointLa_Pred = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	PointLa_Curr = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	CameraMatrix = cv::Mat(3, 3, CV_64FC1);
	CameraMatrix.at<double>(0, 0) = 910.55399;
	CameraMatrix.at<double>(0, 1) = 0.0;
	CameraMatrix.at<double>(0, 2) = 540.0; // 643.59702;
	CameraMatrix.at<double>(1, 0) = 0.0;
	CameraMatrix.at<double>(1, 1) = 908.73393;
	CameraMatrix.at<double>(1, 2) = 360.0; // 374.39075;
	CameraMatrix.at<double>(2, 0) = 0.0;
	CameraMatrix.at<double>(2, 1) = 0.0;
	CameraMatrix.at<double>(2, 2) = 1.0;
	DistCoeff = cv::Mat(5, 1, CV_64FC1);
	DistCoeff.at<double>(0) = 0;
	DistCoeff.at<double>(1) = 0;
	DistCoeff.at<double>(2) = 0;
	DistCoeff.at<double>(3) = 0;
	DistCoeff.at<double>(4) = 0;
}
TVisualNavigation::TVisualNavigation(cv::Mat cm, cv::Mat dist_coeffs)
{
	CameraMatrix = cv::Mat(3, 3, CV_64FC1);
	CameraMatrix = cm;
	DistCoeff = cv::Mat(5, 1, CV_64FC1);
	DistCoeff = dist_coeffs;
}
TVisualNavigation::TVisualNavigation(cv::Mat cm, cv::Mat dist_coeffs, PointLa PointLa1, PointLa PointLa2)
{
	PointLa_Pred = PointLa1;
	PointLa_Curr = PointLa2;
	CameraMatrix = cv::Mat(3, 3, CV_64FC1);
	CameraMatrix = cm;
	DistCoeff = cv::Mat(5, 1, CV_64FC1);
	DistCoeff = dist_coeffs;
}
void TVisualNavigation::SetPointLaPred(PointLa PointLa1)
{
	PointLa_Pred = PointLa1;
}
void TVisualNavigation::SetPointLaCurr(PointLa PointLa1)
{
	PointLa_Curr = PointLa1;
}
void TVisualNavigation::SetParamOpticalFlow(int sizeblock, int sizesearch, int countblock, double porog)
{
	SIZEBLOCK = sizeblock;  // размер эталонного изображения (SIZEBLOCK*SIZEBLOCK)
	SIZESEARCH = sizesearch; // размер текущего изображения (2*SIZESEARCH)*(2*SIZESEARCH)
	COUNTBLOCK = countblock;  // количество опорных точек по вертикали и горизонтали
	COUNTBLOCKGUT = COUNTBLOCK * COUNTBLOCK / 2; // порог на минимальное количество точек в оптическом поток
	POROG_CORRELATION = porog; // порог на корреляционную функцию мак. размер 1.0
}
PointLa TVisualNavigation::GetPointLaPred()
{
	return PointLa_Pred;
}
PointLa TVisualNavigation::GetPointLaCurr()
{
	return PointLa_Curr;
}
void TVisualNavigation::SetCameraMatrix(cv::Mat cm)
{
	CameraMatrix = cm;
}
cv::Mat TVisualNavigation::GetCameraMatrix()
{
	return CameraMatrix;
}
void TVisualNavigation::SetDistCoeffs(cv::Mat cm)
{
	DistCoeff = cm;
}
cv::Mat TVisualNavigation::GetDistCoeffs()
{
	return DistCoeff;
}
int TVisualNavigation::fun_build_flow_correlation(cv::Mat im1, cv::Mat im2, std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts)
{
	if (im1.rows != im2.rows || im1.cols != im2.cols)
		return -1;			// размеры изображений не совпадают
	int m_w = im1.cols;		// ширина изображения
	int m_h = im1.rows;		// высота изображения

	int n_x, n_y;			// количество блоков
	n_x = COUNTBLOCK;
	n_y = COUNTBLOCK;

	cv::Point p1, p2;		// координаты точек на первом и втором изображении соответственно

	double mean_dxdy = 0.0;
	double mean_alfa = 0.0;

	int n_block = 0;
	for (int i = 0; i < COUNTBLOCK; i++)
	{
		for (int j = 0; j < COUNTBLOCK; j++)
		{
			// определяем координаты эталонного изображения
			int x = round(((double)i + 1) * m_w / (COUNTBLOCK + 1));
			int y = round(((double)j + 1) * m_h / (COUNTBLOCK + 1));
			int x_RI = x; if (x_RI < 0) x_RI = 0;
			int y_RI = y; if (y_RI < 0) y_RI = 0;
			int width_RI = SIZEBLOCK; if (width_RI > m_w - 1) width_RI = m_w - 1;
			int height_RI = SIZEBLOCK; if (height_RI > m_h - 1) height_RI = m_h - 1;
			//cv::Rect roi_RI(x_RI, y_RI, width_RI, height_RI);
			//cv::Mat im_RI = im1(roi_RI);   // эталонное изображение
			cv::Mat im_RI = im1(cv::Rect(x_RI, y_RI, width_RI, height_RI));

			int x1, x2, y1, y2;
			// определяем координаты текущего изображения
			x1 = x - SIZESEARCH; if (x1 < 0) x1 = 0;
			y1 = y - SIZESEARCH; if (y1 < 0) y1 = 0;
			x2 = x + 2 * SIZESEARCH; if (x2 > m_w - 1) x2 = m_w - 1;
			y2 = y + 2 * SIZESEARCH; if (y2 > m_h - 1) y2 = m_h - 1;
			int x_CI = x1;
			int y_CI = y1;
			int width_CI = x2 - x1;
			int height_CI = y2 - y1;

			//cv::Rect roi_CI(x_CI, y_CI, width_CI, height_CI);
			//cv::Mat im_CI = im2(roi_CI);   // текущее изображение
			cv::Mat im_CI = im2(cv::Rect(x_CI, y_CI, width_CI, height_CI));   // текущее изображение

			cv::Mat result;
			result.create(1, 1, CV_16S);

			// взаимно-корреляционная функция
			cv::matchTemplate(im_RI, im_CI, result, cv::TM_CCOEFF_NORMED);
			double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
			cv::Point matchLoc;
			minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

			p1.x = x + SIZEBLOCK / 2;
			p1.y = y + SIZEBLOCK / 2;
			p2.x = x1 + maxLoc.x + SIZEBLOCK / 2;
			p2.y = y1 + maxLoc.y + SIZEBLOCK / 2;

			if (maxVal > POROG_CORRELATION)// пороговое значение корреляционной функции
			{
				src_pts.push_back(p1);
				dst_pts.push_back(p2);
				double dxdy = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
				double alfa = atan2((p1.x - p2.x), (p1.y - p2.y));
				mean_dxdy += dxdy;
				mean_alfa += alfa;
				n_block++; // считаем количество точек для который значение корр.функции больше порога
			}
		}
	}
	if (n_block <= COUNTBLOCKGUT) return 3;
	mean_dxdy = mean_dxdy / n_block;  // мат. ожидание длины вектора смещения
	mean_alfa = mean_alfa / n_block;  // мат. ожидание угла вектора смещения

	double std_dxdy = 0.0;
	double std_alfa = 0.0;

	n_block = 0;
	for (int i = 0; i < src_pts.size(); i++)  // удаление выбросов
	{
			// расчеты среднеквадратическое оклонения длины и угла вектора смещения
			double l = sqrt((src_pts[i].x - dst_pts[i].x) * (src_pts[i].x - dst_pts[i].x)
				+ (src_pts[i].y - dst_pts[i].y) * (src_pts[i].y - dst_pts[i].y));
			std_dxdy += (l - mean_dxdy) * (l - mean_dxdy);
			l = atan2((src_pts[i].x - dst_pts[i].x), (src_pts[i].y - dst_pts[i].y)) - mean_alfa;
			std_alfa += l * l;
			n_block++;
	}

	std_dxdy = std_dxdy / (n_block - 1);
	std_alfa = std_alfa / (n_block - 1);

	if (n_block <= COUNTBLOCKGUT) return 3; // недостаточно количество точек
	if ((mean_dxdy <= POROG_DXDY) && (std_dxdy <= 2))
		return 1;

	// статистическая фильтрация векторов
	fun_delete_anomaly(src_pts, dst_pts);
	if (src_pts.size() <= COUNTBLOCKGUT) return 3; // недостаточно количество точек

	if (mean_dxdy > POROG_DXDY)
	{
		if (std_dxdy > 1.5) return 0; // ОК
		else
		{
			if (std_alfa > 0.05) return 0; // ОК
			else return 2; // плоскопараллельное смещение
		}
	}
	else
	{
		if (std_dxdy > 2) return 0; // ОК
		else return 1; // смещение и поворот отсутствует
	}

}
void TVisualNavigation::DrawOpticalFlow(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts)
{
	double u0, v0;
	u0 = CameraMatrix.at<double>(0, 2);
	v0 = CameraMatrix.at<double>(1, 2);

	cv::Mat img = cv::Mat(v0 * 2, u0 * 2, CV_8UC3, cv::Scalar(127, 127, 127));

	for (int i = 0; i < src_pts.size(); i++)
		line(img, cv::Point(src_pts[i].x, src_pts[i].y), cv::Point(dst_pts[i].x, dst_pts[i].y), cv::Scalar(0, 255, 0), 2, 7, 0);
	
	imshow("OpticalFlow", img);
	cv::waitKey(0);
}
int TVisualNavigation::fun_delete_anomaly(std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts)
{
	if ((src_pts.size() == 0) || (dst_pts.size() == 0))
		return -1;
	if (src_pts.size() != dst_pts.size())
		return -1;

	std::vector<double> vec_dxdy0;
	std::vector<double> vec_alfa0;
	double mean_dxdy = 0, mean_alfa = 0;

	for (int i = 0; i < src_pts.size(); i++)
	{
		double dxdy = sqrt(pow((src_pts[i].x - dst_pts[i].x), 2) + pow((src_pts[i].y - dst_pts[i].y), 2));
		double a1, a2;
		a1 = src_pts[i].x - dst_pts[i].x;
		a2 = src_pts[i].y - dst_pts[i].y;
		float alfa = atan2(a1, a2);
		mean_dxdy += dxdy;
		mean_alfa += alfa;
		vec_dxdy0.push_back(dxdy);
		vec_alfa0.push_back(alfa);
	}
	mean_dxdy = mean_dxdy / src_pts.size();
	mean_alfa = mean_alfa / src_pts.size();

	std::vector<double> vec_dxdy = vec_dxdy0;
	std::vector<double> vec_alfa = vec_alfa0;

	std::sort(vec_dxdy.begin(), vec_dxdy.end());
	std::sort(vec_alfa.begin(), vec_alfa.end());

	double a;

	a = (vec_alfa[0] - vec_alfa[1]) / (vec_alfa[0] - vec_alfa[vec_alfa.size() - 1]);
	if (a > (mean_alfa - vec_alfa[0]) / vec_alfa.size())
	{
		for (int i = 0; i < src_pts.size(); i++)
		{
			if (vec_alfa0[i] == vec_alfa[0])
			{
				src_pts.erase(src_pts.begin() + i);
				dst_pts.erase(dst_pts.begin() + i);
			}
		}
	}

	a = (vec_alfa[vec_alfa.size() - 2] - vec_alfa[vec_alfa.size() - 1]) / (vec_alfa[0] - vec_alfa[vec_alfa.size() - 1]);
	if (a > (vec_alfa[vec_alfa.size() - 1] - mean_alfa) / vec_alfa.size())
	{
		for (int i = 0; i < src_pts.size(); i++)
		{
			if (vec_alfa0[i] == vec_alfa[vec_alfa.size() - 1])
			{
				src_pts.erase(src_pts.begin() + i);
				dst_pts.erase(dst_pts.begin() + i);
			}
		}
	}
	a = (vec_dxdy[0] - vec_dxdy[1]) / (vec_dxdy[0] - vec_dxdy[vec_dxdy.size() - 1]);
	if (a > (mean_dxdy - vec_dxdy[0]) / vec_dxdy.size())
	{
		for (int i = 0; i < src_pts.size(); i++)
		{
			if (vec_dxdy0[i] == vec_dxdy[0])
			{
				src_pts.erase(src_pts.begin() + i);
				dst_pts.erase(dst_pts.begin() + i);
			}
		}
	}
	a = (vec_dxdy[vec_dxdy.size() - 2] - vec_dxdy[vec_dxdy.size() - 1]) / (vec_dxdy[0] - vec_dxdy[vec_dxdy.size() - 1]);
	if (a > (vec_dxdy[vec_dxdy.size() - 1] - mean_dxdy) / vec_dxdy.size())
	{
		for (int i = 0; i < src_pts.size(); i++)
		{
			if (vec_dxdy0[i] == vec_dxdy[vec_dxdy.size() - 1])
			{
				src_pts.erase(src_pts.begin() + i);
				dst_pts.erase(dst_pts.begin() + i);
			}
		}
	}

	return 0;
}
PointLa TVisualNavigation::funRealPoints(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts, int time1, int time2)
{
	cv::Mat EE, R1, R2, t, mask;
	EE = findEssentialMat(cv::Mat(src_pts), cv::Mat(dst_pts), CameraMatrix, cv::RANSAC, 0.9999, 1.5, mask);
	
	cv::Mat R0;
	cv::Vec3f EulerAngles;

	EulerAngles[0] = PointLa_Pred.XRoll;
	EulerAngles[1] = PointLa_Pred.YPitch;
	EulerAngles[2] = PointLa_Pred.ZYaw;

	int num_R;

	R0 = eulerAnglesToRotationMatrix(EulerAngles);
	// переменные для расчета угловых скоростей
	double wPitch, wRoll, wYaw;

	if (EE.rows != 0)
	{
		// расчет матриц поворота и вектора смещения для полученной матрицы сущностей
		decomposeEssentialMat(EE, R1, R2, t);
		// проверка матрицы R1 не единичную матрицу
		R1 = checking_unit_matrix(R1, 0.0005);
		// проверка матрицы R2 не единичную матрицу
		R2 = checking_unit_matrix(R2, 0.0005);

		// расчет новых матриц поворота с учетом предудыщей матрицы поворота
		//cv::Mat R11 = R0 * R1;
		//cv::Mat R22 = R0 * R2;

		cv::Mat R11 = R1 * R0;
		cv::Mat R22 = R2 * R0;

		//R1 = selectRotationMat(R11, R22, num_R);

		R1 = selectRotationMat(R1, R2, num_R);
		GetRotation(wYaw, wPitch, wRoll, R1);

		R1 = R1 * R0;

		cv::Vec3f A;
		double Pitch_1, Roll_1, Yaw_1;
		GetRotation(Yaw_1, Pitch_1, Roll_1, R1);

		A[0] = Roll_1;
		A[1] = Pitch_1;
		A[2] = Yaw_1;
	
		R1 = eulerAnglesToRotationMatrix(A);
	}
	else
	{
		EulerAngles[0] = PointLa_Pred.XRoll;
		EulerAngles[1] = PointLa_Pred.YPitch;
		EulerAngles[2] = PointLa_Pred.ZYaw;

		R0 = eulerAnglesToRotationMatrix(EulerAngles);
		R1 = (cv::Mat_<double>(3, 3) <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1);
		R1 = R0 * R1;

		wPitch = 0.0;
		wRoll = 0.0;
		wYaw = 0.0;
	}

	double Pitch, Roll, Yaw;
	GetRotation(Yaw, Pitch, Roll, R1);

	if (Yaw > 2 * M_PI)
		Yaw = Yaw - 2 * M_PI;
	if (Yaw < -2 * M_PI)
		Yaw = Yaw + 2 * M_PI;

	if (Yaw > M_PI)
		Yaw = Yaw - 2 * M_PI;
	else if (Yaw < -M_PI)
		Yaw = Yaw + 2 * M_PI;

	cv::Vec3f position;
	position[0] = PointLa_Pred.X;
	position[1] = PointLa_Pred.Y;
	position[2] = PointLa_Pred.Z;
	cv::Mat keypoint(src_pts.size(), 3, CV_64FC1);
	my_px_to_obj(src_pts, R0, position, keypoint, 1.2);
	
	cv::Mat coefficients(src_pts.size(), 8, CV_64FC1);

	EulerAngles[0] = Roll;
	EulerAngles[1] = Pitch;
	EulerAngles[2] = Yaw;

	R1 = eulerAnglesToRotationMatrix(EulerAngles);
	//********************************************

	my_prepare_data_for_lms(dst_pts, keypoint, R1, coefficients);
	cv::Vec3f dxdydz;
	my_construct_matrices_lsm(coefficients, dxdydz);

	PointLa_Curr.X = PointLa_Pred.X - dxdydz(0);
	PointLa_Curr.Y = PointLa_Pred.Y - dxdydz(1);
	PointLa_Curr.Z = PointLa_Pred.Z - dxdydz(2);
	PointLa_Curr.XRoll = Roll;
	PointLa_Curr.YPitch = Pitch;
	PointLa_Curr.ZYaw = Yaw;
	PointLa_Curr.m_time = time2;

	int delta_time = (time2 - time1);
	PointLa_Curr.vx = (double)CLOCKS_PER_SEC*dxdydz(0) / delta_time;
	PointLa_Curr.vy = (double)CLOCKS_PER_SEC*dxdydz(1) / delta_time;
	PointLa_Curr.vz = (double)CLOCKS_PER_SEC*dxdydz(2) / delta_time;
	PointLa_Curr.wXRoll = (double)CLOCKS_PER_SEC*wRoll / delta_time;
	PointLa_Curr.wYPitch = (double)CLOCKS_PER_SEC*wPitch / delta_time;
	PointLa_Curr.wZYaw = (double)CLOCKS_PER_SEC*wYaw / delta_time;

	return PointLa_Curr;
}
cv::Mat TVisualNavigation::eulerAnglesToRotationMatrix(cv::Vec3f& theta)
{
	// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// Calculate rotation about y axis
	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);

	// Combined rotation matrix
	cv::Mat R = R_z * R_y * R_x;
	return R;
}
cv::Mat TVisualNavigation::selectRotationMat2(cv::Mat R1, cv::Mat R2, int& num)
{
	cv::Vec3f A;
	double Pitch_1, Roll_1, Yaw_1;
	GetRotation(Yaw_1, Pitch_1, Roll_1, R1);
	A[0] = Roll_1;
	A[1] = Pitch_1;
	A[2] = Yaw_1;
	R1 = eulerAnglesToRotationMatrix(A);
	
	double Pitch_2, Roll_2, Yaw_2;
	GetRotation(Yaw_2, Pitch_2, Roll_2, R2);
	A[0] = Roll_2;
	A[1] = Pitch_2;
	A[2] = Yaw_2;
	R2 = eulerAnglesToRotationMatrix(A);

	double Roll_Pred = PointLa_Pred.XRoll;
	double Pitch_Pred = PointLa_Pred.YPitch;
	double Yaw_Pred = PointLa_Pred.ZYaw;

	if ((Yaw_Pred > M_PI / 2) && (Yaw_Pred < 3 * M_PI / 2))
	{
		if ((Yaw_1 > (-M_PI)) && (Yaw_1 < 0))
		{
			Yaw_1 = Yaw_1 + 2 * M_PI;
		}
		if ((Yaw_2 > (-M_PI)) && (Yaw_2 < 0))
		{
			Yaw_2 = Yaw_2 + 2 * M_PI;
		}
	}

	double d1, d2;
	double e1, e2, e3;
	e1 = Pitch_1-Pitch_Pred;
	e2 = Roll_1-Roll_Pred;
	e3 = Yaw_1-Yaw_Pred;
	d1 = sqrt(e1 * e1 + e2 * e2 + e3 * e3);

	e1 = Pitch_2-Pitch_Pred;
	e2 = Roll_2-Pitch_Pred;
	e3 = Yaw_2-Yaw_Pred;
	d2 = sqrt(e1 * e1 + e2 * e2 + e3 * e3);

	if (d1 < d2)
	{
		num = 1;
		return R1;
	}
	else
	{
		num = 2;
		return R2;
	}
}
cv::Mat TVisualNavigation::selectRotationMat(cv::Mat R1, cv::Mat R2, int& num)
{
	cv::Vec3f A;
	double Pitch_1, Roll_1, Yaw_1;
	GetRotation(Yaw_1, Pitch_1, Roll_1, R1);
	A[0] = Roll_1;
	A[1] = Pitch_1;
	A[2] = Yaw_1;
	R1 = eulerAnglesToRotationMatrix(A);

	double Pitch_2, Roll_2, Yaw_2;
	GetRotation(Yaw_2, Pitch_2, Roll_2, R2);
	A[0] = Roll_2;
	A[1] = Pitch_2;
	A[2] = Yaw_2;
	R2 = eulerAnglesToRotationMatrix(A);

	double d1, d2;
	double e1, e2, e3;
	e1 = Pitch_1;
	e2 = Roll_1;
	e3 = Yaw_1;
	d1 = sqrt(e1 * e1 + e2 * e2 + e3 * e3);

	e1 = Pitch_2;
	e2 = Roll_2;
	e3 = Yaw_2;
	d2 = sqrt(e1 * e1 + e2 * e2 + e3 * e3);

	if (d1 < d2)
	{
		num = 1;
		return R1;
	}
	else
	{
		num = 2;
		return R2;
	}
}
void TVisualNavigation::GetRotation(double& ZYaw, double& YPitch, double& XRoll, cv::Mat M)
{
	double e = 0.00000001;
	float sy = sqrt(M.at<double>(2, 1) * M.at<double>(2, 1) + M.at<double>(2, 2) * M.at<double>(2, 2));

	if (fabs(1.0 - M.at<double>(2, 0)) < e)
	{
		ZYaw = -atan2(M.at<double>(1, 2), M.at<double>(1, 1));
		YPitch = -M_PI / 2;
		XRoll = 0.0;
	}
	else if ((fabs(-1.0 - M.at<double>(2, 0)) < e))
	{
		ZYaw = atan2(M.at<double>(1, 2), M.at<double>(1, 1));
		YPitch = M_PI / 2;
		XRoll = 0.0;
	}
	else
	{
		XRoll = atan2(M.at<double>(2, 1), M.at<double>(2, 2));	// тангаж X
		YPitch = -atan2(M.at<double>(2, 0), sy);	//крен Y
		ZYaw = atan2(M.at<double>(1, 0), M.at<double>(0, 0));	// рысканье Z
	}
}
cv::Mat TVisualNavigation::checking_unit_matrix(cv::Mat M, double e)
{
	double a00, a11, a22;
	a00 = fabs(M.at<double>(0, 0));
	a11 = fabs(M.at<double>(1, 1));
	a22 = fabs(M.at<double>(2, 2));
	if ((1.0-a00)<e && (1.0-a11)<e && (1.0-a22)<e)
	{
		cv::Vec3f theta;
		theta[0] = 0.0;
		theta[1] = 0.0;
		double a = atan2(M.at<double>(1, 0), M.at<double>(0, 0));
		theta[2] = a;
		M = eulerAnglesToRotationMatrix(theta);
	}
	return M;
}
int TVisualNavigation::my_px_to_obj(std::vector<cv::Point2f> keypointsUV, cv::Mat r,
									cv::Vec3f position, cv::Mat& keypointsXYZ, double H)
{
	double u0, v0;
	double a1, a2, a3;
	u0 = CameraMatrix.at<double>(0, 2);
	v0 = CameraMatrix.at<double>(1, 2);
	double fx, fy;
	fx = CameraMatrix.at<double>(0, 0);
	fy = CameraMatrix.at<double>(1, 1);
	for (int i = 0; i < keypointsUV.size(); i++)
	{
		keypointsXYZ.at<double>(i, 2) = H;
		a1 = r.at<double>(0, 0) * (keypointsUV[i].x - u0) + r.at<double>(0, 1) * (keypointsUV[i].y - v0) - r.at<double>(0, 2) * fx;
		a2 = r.at<double>(1, 0) * (keypointsUV[i].x - u0) + r.at<double>(1, 1) * (keypointsUV[i].y - v0) - r.at<double>(1, 2) * fy;

		a3 = r.at<double>(2, 0) * (keypointsUV[i].x - u0) + r.at<double>(2, 1) * (keypointsUV[i].y - v0) - r.at<double>(2, 2) * fx;
		keypointsXYZ.at<double>(i, 0) = keypointsXYZ.at<double>(i, 2) * a1 / a3;
		a3 = r.at<double>(2, 0) * (keypointsUV[i].x - u0) + r.at<double>(2, 1) * (keypointsUV[i].y - v0) - r.at<double>(2, 2) * fy;
		keypointsXYZ.at<double>(i, 1) = keypointsXYZ.at<double>(i, 2) * a2 / a3;
	}
	return 0;
}
int TVisualNavigation::my_prepare_data_for_lms(std::vector<cv::Point2f> dst_pts, cv::Mat keypointsXYZ,
	cv::Mat r, 	cv::Mat& coefficients)
{
	double u0, v0;
	double cx1, cy1, cx2, cy2;
	double a1, a2, a3, b1, b2, b3;
	u0 = CameraMatrix.at<double>(0, 2);
	v0 = CameraMatrix.at<double>(1, 2);
	for (int i = 0; i < dst_pts.size(); i++)
	{
		cx1 = (u0 - dst_pts[i].x);
		cy1 = (v0 - dst_pts[i].y);

		a1 = cx1 * r.at<double>(0, 2) - CameraMatrix.at<double>(0, 0) * r.at<double>(0, 0);
		a2 = cx1 * r.at<double>(1, 2) - CameraMatrix.at<double>(0, 0) * r.at<double>(1, 0);
		a3 = cx1 * r.at<double>(2, 2) - CameraMatrix.at<double>(0, 0) * r.at<double>(2, 0);
		b1 = cy1 * r.at<double>(0, 2) - CameraMatrix.at<double>(1, 1) * r.at<double>(0, 1);
		b2 = cy1 * r.at<double>(1, 2) - CameraMatrix.at<double>(1, 1) * r.at<double>(1, 1);
		b3 = cy1 * r.at<double>(2, 2) - CameraMatrix.at<double>(1, 1) * r.at<double>(2, 1);

		cx2 = keypointsXYZ.at<double>(i, 0) * a1 +
			keypointsXYZ.at<double>(i, 1) * a2 +
			keypointsXYZ.at<double>(i, 2) * a3;
		cy2 = keypointsXYZ.at<double>(i, 0) * b1 +
			keypointsXYZ.at<double>(i, 1) * b2 +
			keypointsXYZ.at<double>(i, 2) * b3;

		coefficients.at<double>(i, 0) = a1;
		coefficients.at<double>(i, 1) = a2;
		coefficients.at<double>(i, 2) = a3;
		coefficients.at<double>(i, 3) = cx2;
		coefficients.at<double>(i, 4) = b1;
		coefficients.at<double>(i, 5) = b2;
		coefficients.at<double>(i, 6) = b3;
		coefficients.at<double>(i, 7) = cy2;
	}
	return 0;
}
int TVisualNavigation::my_construct_matrices_lsm(cv::Mat coefficients, cv::Vec3f& position)
{
	int count = coefficients.rows;
	cv::Mat A(count * 2, 3, CV_64FC1);
	cv::Mat B(count * 2, 1, CV_64FC1);

	for (int i = 0; i < count; i++)
	{
		A.at<double>(i * 2, 0) = coefficients.at<double>(i, 0);
		A.at<double>(i * 2, 1) = coefficients.at<double>(i, 1);
		A.at<double>(i * 2, 2) = coefficients.at<double>(i, 2);
		B.at<double>(i * 2, 0) = coefficients.at<double>(i, 3);
		A.at<double>(i * 2 + 1, 0) = coefficients.at<double>(i, 4);
		A.at<double>(i * 2 + 1, 1) = coefficients.at<double>(i, 5);
		A.at<double>(i * 2 + 1, 2) = coefficients.at<double>(i, 6);
		B.at<double>(i * 2 + 1, 0) = coefficients.at<double>(i, 7);
	}

	cv::Mat At, AtA, AtA_invers, res;
	cv::transpose(A, At);
	AtA = At * A;
	AtA_invers = AtA.inv();
	res = AtA_invers * At * B;
	position[0] = res.at<double>(0);
	position[1] = res.at<double>(1);
	position[2] = res.at<double>(2);
	return 0;
}
void TVisualNavigation::SaveFileVector(const char* filename)
{
	std::ofstream fout;
	fout.open(filename, std::ios::app);

	fout << std::fixed << std::setprecision(8);
	// углы ориентации
	fout << PointLa_Pred.XRoll * 180 / M_PI << '\t' << PointLa_Pred.YPitch * 180 / M_PI << '\t' << PointLa_Pred.ZYaw * 180 / M_PI << '\t';
	// положение		
	fout << PointLa_Pred.X << '\t' << PointLa_Pred.Y << '\t' << PointLa_Pred.Z << '\t';
	fout << PointLa_Pred.vx << '\t' << PointLa_Pred.vy << '\t' << PointLa_Pred.vz << '\t';
	fout << PointLa_Pred.wXRoll << '\t' << PointLa_Pred.wYPitch << '\t' << PointLa_Pred.wZYaw << '\t';
	fout << PointLa_Pred.m_time << '\n';
	
	fout.close();
}
void TVisualNavigation::CorrectDistortion(std::vector<cv::Point2f>& src_pts, std::vector<cv::Point2f>& dst_pts)
{
	double u, v, k1, k2, k3, p1, p2, r2, s;
	k1 = DistCoeff.at<double>(0);
	k2 = DistCoeff.at<double>(1);
	k3 = DistCoeff.at<double>(4);
	p1 = DistCoeff.at<double>(2);
	p2 = DistCoeff.at<double>(3);
	for (int i = 0; i < src_pts.size(); i++)
	{
		u = src_pts[i].x;
		v = src_pts[i].y;
		r2 = u * u + v * v;
		s = (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
		src_pts[i].x = u * s + 2 * p1 * u * v + p2 * (r2 + 2 * u * u);
		src_pts[i].y = v * s + p1 * (r2 + 2 * v * v) + 2 * p2 * u * v;

		u = dst_pts[i].x;
		v = dst_pts[i].y;
		r2 = u * u + v * v;
		s = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
		dst_pts[i].x = u * s + 2 * p1 * u * v + p2 * (r2 + 2 * u * u);
		dst_pts[i].y = v * s + p1 * (r2 + 2 * v * v) + 2 * p2 * u * v;
	}
}