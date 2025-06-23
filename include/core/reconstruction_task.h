#pragma once

#include <opencv2/opencv.hpp>

#include "task.h"
#include "system_params.h"

class ReconstructionTask : public Task
{
public:
    ReconstructionTask(int id);

private:
    void run() override;

    void findBilData();
    void reconstruction();
    void extractLaser(const HObject &ho_image, HObject *ho_laser_image, const HTuple &hv_min_threshold);
    void resizeImage(const HObject &ho_image_part, const HTuple &hv_width,
                     const HTuple &hv_height, HObject *ho_resized_image,
                     const HTuple &hv_roi_row1, const HTuple &hv_roi_column1);
    std::string getTime();

    void preprocess();
    void setRoi(const cv::Mat &src_mat);
    void setThreshold(const cv::Mat &src_mat);
    void matToHObject(const cv::Mat &src_mat, HObject *dst_hobj);

    bool drawing_box_ = false;
    cv::Rect box_{0, 0, 0, 0};

    int temp_threshold_ = 0;
    cv::Mat src_mat_{};
    cv::Mat dst_mat_{};
};
