#include "visual_navigation.h"



int main()
{
    Eigen::MatrixXd A_k_minus_1(12, 12);
    Eigen::VectorXd process_noise_v_k_minus_1(12);
    Eigen::VectorXd k1(12);
    Eigen::MatrixXd H_k(12, 12);
    Eigen::VectorXd k(12);
    Eigen::VectorXd sensor_noise_w_k(12);
    Eigen::VectorXd state_estimate_k_minus_1(12);
    Eigen::VectorXd control_vector_k_minus_1(6);
    Eigen::VectorXd k2(12);
    Eigen::VectorXd temp(12);
    Eigen::VectorXd z_k_observation_vector(12);
    Eigen::VectorXd d_k(3);
    Eigen::MatrixXd rez(12, 10000);

    TVisualNavigation VisualNav1;
    PointLa La_Curr;// вектор состояния текущий
    PointLa La_Pred;
    std::string filename = "rs_capture.avi";
    cv::VideoCapture cap;
    cap.open(filename.c_str());
    cv::Mat frame1, frame2;
    int time1, time2;

    mat_set(A_k_minus_1, process_noise_v_k_minus_1, k1, H_k, k, sensor_noise_w_k, state_estimate_k_minus_1, control_vector_k_minus_1,
        k2, 0);
    my_ekf ekf1(A_k_minus_1, process_noise_v_k_minus_1, k1, H_k, k, sensor_noise_w_k, state_estimate_k_minus_1, k2,
        control_vector_k_minus_1);
    if (!cap.isOpened())
        return -1;
    // пропускаем несколько кадров
    cap >> frame1; 	cap >> frame1; 	cap >> frame1; 	cap >> frame1;
    time1 = clock();
    // приводим изображение к нужному формату
    cvtColor(frame1, frame1, cv::COLOR_BGR2GRAY);
    // приводим изображение к нужному размеру
    resize(frame1, frame1, cv::Size(1080, 720), 0, 0, cv::INTER_LINEAR);
    int num = 0, l = 0;
    // цикл по всем кадрам
    for (; ; )
    {
        num++;
        std::cout << "Num kadr = " << num << std::endl;
        cap >> frame2;
        time2 = clock();
        if (frame2.empty())
            break;
        cvtColor(frame2, frame2, cv::COLOR_BGR2GRAY);
        resize(frame2, frame2, cv::Size(1080, 720), 0, 0, cv::INTER_LINEAR);
        std::vector<cv::Point2f> src_pts, dst_pts;
        int res = 0;
        // расчет оптического потока по двум кадрам
        res = VisualNav1.fun_build_flow_correlation(frame1, frame2, src_pts, dst_pts);
        if (res == 0)
        {
            double tec_time;
            // корректировка дисторсии
            VisualNav1.CorrectDistortion(src_pts, dst_pts);
            // расчет вектора смещения и матрицы поворота
            La_Curr = VisualNav1.funRealPoints(src_pts, dst_pts, time1, time2);
            La_Pred = VisualNav1.GetPointLaPred();
            tec_time = (La_Curr.m_time - La_Pred.m_time) / CLOCKS_PER_SEC;
            mat_set(A_k_minus_1, process_noise_v_k_minus_1, k1, H_k, k, sensor_noise_w_k, state_estimate_k_minus_1, control_vector_k_minus_1,
                k2, tec_time);
            set_z(z_k_observation_vector, d_k, La_Curr, La_Pred);

            ekf1.process(rez, z_k_observation_vector, d_k, l);
            ekf1.save_ekf(rez, l);
            l++;

            frame1 = frame2.clone();
            time1 = time2;
            La_Curr.Z = 0;
            // ЗДЕСЬ ДОЛЖНА БЫТЬ КОРРЕКТИРОВНА УГЛОВ КРЕНА И ТАНГАЖА ОТ БИНС
            if (La_Curr.YPitch > M_PI / 6)
                La_Curr.YPitch = M_PI / 6;
            if (La_Curr.YPitch < -M_PI / 6)
                La_Curr.YPitch = -M_PI / 6;
            if (La_Curr.XRoll > M_PI / 6)
                La_Curr.XRoll = M_PI / 6;
            if (La_Curr.XRoll < -M_PI / 6)
                La_Curr.XRoll = -M_PI / 6;
            VisualNav1.SetPointLaPred(La_Curr);
            VisualNav1.SaveFileVector("result.txt");
            std::cout << "OK \n";
        }
        else
        {
            if (res == 1) std::cout << "Skipping a frame: No moving \n";
            else {
                if (res == 2)
                {
                    // корректировка дисторсии
                    VisualNav1.CorrectDistortion(src_pts, dst_pts);
                    // расчет вектора смещения и матрицы поворота
                    La_Curr = VisualNav1.funRealPoints(src_pts, dst_pts, time1, time2);
                    frame1 = frame2.clone();
                    time1 = time2;
                    VisualNav1.SetPointLaPred(La_Curr);
                }
                else
                {
                    std::cout << "Skipping a frame: Count point <= 18 \n";
                    frame1 = frame2.clone();
                    time1 = time2;
                    VisualNav1.SetPointLaPred(VisualNav1.GetPointLaCurr());
                    // попробывать вариант с ключевыми точками.
                }
            }
        }
    }
    system("pause");
    return 0;
    
}