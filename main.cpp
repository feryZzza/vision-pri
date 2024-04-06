#include <iostream>
#include <opencv2/opencv.hpp>

#include "Calibration.hpp"
#include "fps.hpp"
#include "Kalm.hpp"
#include "AirResist.hpp"
#include "imagetest.hpp"

/*
file: main.cpp
date：2023-12-12
author: feryZzza
TODO: learning about cv
*/

int main() {

    std::cout << "Enter '1' -> camera calibration " << "\n";
    std::cout << "Enter '2' -> show fps " << "\n";   
    std::cout << "Enter '3' -> kalm filter " << "\n";  
    std::cout << "Enter '4' -> show airresistsolve " << "\n";
    std::cout << "Enter '5' -> RGB param test " << "\n";
    std::cout << "Enter '0' -> exit " << "\n";
    int input;
    std::cin >> input;
    
    switch (input)
    {
    case 1:
        CameraCalibration calib;
        calib.performCalibration();        
        break;
    case 2:
        ShowFps fps;
        fps.fpsshow();        
        break;
    case 3:
        Kalm kf;
        kf.kalm();
        break;
    case 4:{
        float a,b;
        double kv;
        std::cout << "please input the target coordinates(m) (use: a,b):";
        scanf("%f,%f",&a,&b);
        std::cout << "please input the bullet's speed(m/s):";
        std::cin >> kv;
        AirResist ar;
        cv::Point2f point_a(a,b);
        cv::Vec2f optimal_theta_deg = ar.AirResistSolve(point_a, kv); // 调用优化函数
        std::cout << "Optimal launch angle: " << optimal_theta_deg << std::endl;
        cv::Vec2f parabol_deg = ar.ParabolSolve(point_a, kv);
        std::cout << "Parabol angle: " << parabol_deg << std::endl;
        break;
    }// case里面的赋值操作需要大括号框起来，以避免在跳转到case或default标签时跳过变量的初始化
    case 5:
        Test ts;
        ts.test();
        break;
    case 0:
        break;
    default:
        break;
    }
    return 0;
}
