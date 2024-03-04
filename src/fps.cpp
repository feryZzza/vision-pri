#include "fps.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

void ShowFps::fpsshow(){
    // 打开视频文件
    cv::VideoCapture cap("../video/orange1.mp4");

    if (!cap.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
    }

    cv::Mat frame;
    double fps = 0.0;
    cv::TickMeter tm;

    while (true) {
        bool isSuccess = cap.read(frame); // 读取新的帧

        if (!isSuccess) {
            std::cerr << "Can't read frames from the video" << std::endl;
            break;
        }

        tm.start(); // 开始计时

        // 添加 FPS 文本到帧上
        std::ostringstream fpsTextStream;
        fpsTextStream << "FPS: " << fps;
        cv::putText(frame, fpsTextStream.str(), cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        tm.stop(); // 停止计时

        // 计算 FPS
        double time_per_frame = tm.getTimeSec() / tm.getCounter();
        fps = 0.008 / time_per_frame;

        // 重置计时器
        tm.reset();

        cv::imshow("Video", frame);

        if (cv::waitKey(30) >= 0) break;
    }

    cap.release();
    cv::destroyAllWindows();

}
