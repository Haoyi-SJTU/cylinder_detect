#include "SaveImage.h"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int SaveMatImage(const char *save_address, RVC::Image &img) {
    if (!img.IsValid()) {
        std::cout << "image is invalid!" << std::endl;
        return 1;
    }
    RVC::Size img_size = img.GetSize();
    int img_num = img_size.rows * img_size.cols;
    cv::Mat save_img;
    if (img.GetType() == RVC::ImageType::Mono8) {
        save_img = cv::Mat(img_size.rows, img_size.cols, CV_8UC1, img.GetDataPtr());
    } else {
        save_img = cv::Mat(img_size.rows, img_size.cols, CV_8UC3, img.GetDataPtr());
        if (img.GetType() == RVC::ImageType::RGB8) {
            cv::cvtColor(save_img, save_img, cv::COLOR_RGB2BGR);
        }
    }
    cv::imwrite(save_address, save_img);

    return 0;
}