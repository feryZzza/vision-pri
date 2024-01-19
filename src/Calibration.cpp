#include "Calibration.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

/*
file: Calibration.cpp
date： 2023-12-11
author: feryZzza
TODO: learn how to calibrate camera
*/

void CameraCalibration::performCalibration() {
    
    // Initialize parameters
    int numCornersHor = 11;
    int numCornersVer = 8;
    int numSquares = numCornersHor * numCornersVer;
    cv::Size boardSize = cv::Size(numCornersHor, numCornersVer);

    // Prepare object points
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < numSquares; i++){
        //obj.push_back(cv::Point3f(i / numCornersHor, i % numCornersHor, 0.0f));
        obj.push_back(cv::Point3f((i / numCornersHor) * 20, (i % numCornersHor) * 20, 0.0f));
    }
    
    // Vectors to store object points and image points from all images
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // Load and process calibration images
    for (int i = 1; i <= 20; i++) {
        cv::Mat image = cv::imread("/home/z/cv/v2/image/calibrate_images/output" + std::to_string(i) + ".jpg");
        std::vector<cv::Point2f> corners; // This will be filled by the detected corners

        // Find the chess board corners
        bool success = cv::findChessboardCorners(image, boardSize, corners);

        if (success) {
            // Improve the found corners' coordinate accuracy
            cv::Mat grayImage;
            cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            // Add object points and image points
            objectPoints.push_back(obj);
            imagePoints.push_back(corners);

            // Display the corners
            cv::drawChessboardCorners(image, boardSize, corners, success);
            cv::imshow("Calibration", image);
            
            // Wait for a space bar press to continue
            while (true) {
                int key = cv::waitKey(0);
                if (key == 32) break; // Space bar
            }
        }
    }

    // Perform camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(objectPoints, imagePoints, cv::Size(640, 480), cameraMatrix, distCoeffs, rvecs, tvecs);

    // Output the results
    std::cout << "Camera Matrix:\n" << cameraMatrix << "\nDistortion Coefficients:\n" << distCoeffs << std::endl;

}
