#include "imagetest.hpp"

/**
 * @brief 传统视觉识别测试
 */

void Test::test(){

    cv::Mat img = cv::imread("../image/images/114514.png");
    cv::Mat roi = img;
    cv::Mat src1;
    roi.copyTo(src1);

    cv::Mat img_gray;
    std::vector<cv::Mat> rgb;

    cv::Mat hsv, img_gray11, img_gray21;
    split(roi, rgb);

    cv::Mat img_gray1 = (rgb[2] - rgb[0]);
    cv::Mat img_gray2 = (rgb[0] - rgb[2]);
    cv::Mat img_gray3 = (rgb[1]);
    
    cv::threshold(img_gray1, img_gray11, 90, 255, cv::THRESH_BINARY); // 将灰度图二值化 寻找灯条解
    cv::threshold(img_gray2, img_gray21, 100, 255, cv::THRESH_BINARY);

    cv::threshold(img_gray1, img_gray1, 240, 255, cv::THRESH_BINARY); // 将灰度图二值化 寻找灯条解
    cv::threshold(img_gray2, img_gray2, 240, 255, cv::THRESH_BINARY);
    cv::threshold(img_gray3, img_gray3, 240, 255, cv::THRESH_BINARY);
    //cv::namedWindow("img_gray1", cv::WINDOW_NORMAL);
    cv::imshow("img_gray1", img_gray1);
    //cv::namedWindow("img_gray2", cv::WINDOW_NORMAL);
    cv::imshow("img_gray2", img_gray2);
    //cv::namedWindow("img_gray3", cv::WINDOW_NORMAL);
    cv::imshow("img_gray3", img_gray3);
    //cv::namedWindow("img_gray11", cv::WINDOW_NORMAL);
    cv::imshow("img_gray11", img_gray11);
    //cv::namedWindow("img_gray21", cv::WINDOW_NORMAL);
    cv::imshow("img_gray21", img_gray21);
    img_gray = ~(img_gray1 | img_gray2) & img_gray3 & (~(img_gray11 | img_gray21));
    cv::imshow("gray",img_gray);
    cv::waitKey(0);

}