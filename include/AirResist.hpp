#ifndef AIRRESIST_HPP
#define AIRRESIST_HPP

#include <vector>
#include <functional>
#include <limits>
#include <cmath>
#include <iostream>
#include <nlopt.hpp>
#include <opencv2/opencv.hpp>
#include <boost/numeric/odeint.hpp>

class AirResist{
public:
    // 微分方程模型
    void model(const std::vector<double> &z, std::vector<double> &dzdt, double t, double k1, double g);

    // Observer结构记录轨迹数据
    struct StateObserver{
        std::vector<std::vector<double>> states;
        void operator()(const std::vector<double> &z, double t);
    };

    // 目标函数
    double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);

    // 优化函数
    cv::Vec2f ParabolaSolve(cv::Point2f point_a, double kv);

    static double ObjectiveWrapper(const std::vector<double> &x, std::vector<double> &grad, void *data);

};
#endif // AIRRESIST_HPP
