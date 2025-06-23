#pragma once

#include <opencv2/opencv.hpp>

#include "task.h"
#include "system_params.h"

class CalibrationTask : public Task
{
public:
    CalibrationTask(int id);

private:
    void run() override;

    void calibrate();
    void calibrateCamera(HTuple &hv_calibration_data_id);
    void calibrateLightPlane(HTuple &hv_calibration_data_id);
    void calibrateMovement(HTuple &hv_calibration_data_id);
    void saveToFile();

    int setThreshold(const cv::Mat &src_mat);
    void compute3dCoordinatesOfLightLine(HObject ho_profile_image, HTuple hv_min_gray,
                                         HTuple hv_camera_parameters, HTuple hv_local_camera_pose, HTuple hv_reference_camera_pose,
                                         HTuple *hv_x, HTuple *hv_y, HTuple *hv_z);

    void fit3dPlaneXyz(HTuple hv_x, HTuple hv_y, HTuple hv_z, HTuple *hv_ox, HTuple *hv_oy, HTuple *hv_oz,
                       HTuple *hv_nx, HTuple *hv_ny, HTuple *hv_nz, HTuple *hv_mean_residual);

    void getLightPlanePose(HTuple hv_origin_x, HTuple hv_origin_y, HTuple hv_origin_z,
                           HTuple hv_normal_vector_x, HTuple hv_normal_vector_y, HTuple hv_normal_vector_z,
                           HTuple *hv_light_plane_pose);
};