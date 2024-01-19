#include <opencv2/opencv.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "Kalm.hpp"

void Kalm::kalm() {

    cv::VideoCapture cap("../video/orange1.mp4");
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return;
    }

    double dt = 1.0 / 30; // 假设视频帧率是 30 FPS

    // 定义状态向量和测量向量的大小
    const int stateSize = 4;
    const int measSize = 4;

    // 定义状态向量、测量向量和各种矩阵
    Eigen::VectorXd state(stateSize);
    Eigen::VectorXd meas(measSize);

    Eigen::MatrixXd A(stateSize, stateSize); // 状态转移矩阵
    Eigen::MatrixXd H(measSize, stateSize);  // 测量矩阵
    Eigen::MatrixXd Q(stateSize, stateSize); // 过程噪声协方差矩阵
    Eigen::MatrixXd R(measSize, measSize);   // 测量噪声协方差矩阵
    Eigen::MatrixXd P(stateSize, stateSize); // 估计误差协方差矩阵

    // 初始化矩阵
    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1,  0,
         0, 0, 0,  1;

    H << 1, 0, 0, 0,            // 关于测量矩阵：（仅限于 “n倍预测“ 版本）               
         0, 1, 0, 0,            // 加入测量vx & vy，可以使得运动停止时的回归更迅速，
         0, 0, 1, 0,            // 如果仅观测x & y，预测不会改变，但是运动停止时无法回归
         0, 0, 0, 1;            //

    Q.setIdentity();            // 单位阵
    R.setIdentity();            // 单位阵
    P.setIdentity();            // 单位阵

    Q *= 0.1;                   // 过程噪声协方差调节
    //R *= 10;                  // 测量调节
    
    double previous_x = 0, previous_y = 0;


    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        double detected_x, detected_y;


        // 检测获取物体的坐标
        cv::Mat gray,hsv;
        cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(18,50,50),cv::Scalar(25,255,255),gray);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray,contours,0,2);
        float r;
        for(int i = 0; i < contours.size(); i++) {
            double area =cv::contourArea(contours[i]);
            if(area > 10000){
            cv::Point2f center;                           // 测量坐标
            cv::minEnclosingCircle(contours[i],center,r);
            circle(frame,center,r,cv::Scalar(255,255,0),2);
            detected_x = center.x;
            detected_y = center.y;
            }
        }

        // 计算速度
        double vx = (detected_x - previous_x) / dt;
        double vy = (detected_y - previous_y) / dt;

        // 测量更新
        meas << detected_x, detected_y, vx, vy;

        // 卡尔曼滤波器预测
        state = A * state;
        P = A * P * A.transpose() + Q;

        // 卡尔曼滤波器更新
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        state = state + K * (meas - H * state);                             // 后验
        P = (Eigen::MatrixXd::Identity(stateSize, stateSize) - K * H) * P;  // 更新过程噪声协方差矩阵

        //cv::circle(frame, cv::Point(state(0),state(1)),r,cv::Scalar(0,255,0),2);

        // 预测 t 帧后的状态
        int t = 5;
        Eigen::VectorXd futureState = state;
        for (int i = 0; i < t; ++i) {
            futureState = A * futureState;
        }

        // 获取预测的位置
        double predicted_x = futureState(0);
        double predicted_y = futureState(1);
        cv::circle(frame, cv::Point(predicted_x, predicted_y), r, cv::Scalar(255, 0, 0), 2);


        cv::imshow("Tracking", frame);
        if (cv::waitKey(30) >= 0) break;

        previous_x = detected_x;
        previous_y = detected_y;
    }

    cap.release();
    cv::destroyAllWindows();
}
