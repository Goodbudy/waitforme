#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat image = cv::Mat::zeros(300, 300, CV_8UC3);
    cv::circle(image, cv::Point(150,150), 100, cv::Scalar(255, 0, 0), -1);
    cv::imshow("Blue Circle", image);
    cv::waitKey(0);
    return 0;
}
