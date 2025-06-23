#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <HalconCpp.h>

#include "nlohmann/json.hpp"

using namespace HalconCpp;
using json = nlohmann::json;

struct hvProgramParams
{
    // Data path
    HTuple hv_data_root;
    HTuple hv_calibration_images_path;
    HTuple hv_laser1_path;
    HTuple hv_laser2_path;
    HTuple hv_movement1_path;
    HTuple hv_movement2_path;
    HTuple hv_caltab_description_path;
    HTuple hv_output_cloud_root;
    std::vector<HTuple> hv_output_cloud_paths;

    // Calibration parameters
    HTuple hv_calibration_images_num;
    HTuple hv_calibration_min_threshold;
    HTuple hv_calibration_steps_num;
    HTuple hv_calibration_thickness;
    HTuple hv_calibration_max_light_plane_error;

    // Reconstruction parameters
    HTuple hv_reconstruction_min_threshold;
    HTuple hv_reconstruction_roi_row1;
    HTuple hv_reconstruction_roi_col1;
    HTuple hv_reconstruction_roi_row2;
    HTuple hv_reconstruction_roi_col2;
};

struct hvSystemPoses
{
    HTuple hv_start_params;
    HTuple hv_camera_params;
    HTuple hv_camera_pose;
    HTuple hv_light_plane_poses;
    HTuple hv_movement_poses;
};

enum class Path
{
    DataRoot,
    CaltabDescription,
    OutputModelRoot,
    CameraParameters,
    CameraPose,
    LightPlanePose,
    MovementPose,
    All
};

class SystemParams
{
public:
    static SystemParams &instance();
    void reload(const std::string &new_config_path);
    void validatePaths(Path path) const;

    hvProgramParams hv_program_params_;
    hvSystemPoses hv_system_poses_;

    // Flag parameters
    mutable bool calibration_flag_ = true;
    bool save_cloud_flag_;
    bool set_roi_flag_;
    bool select_entire_frame_;
    bool set_threshold_flag_ = true;

    std::string config_path_;
    int img_width_;
    int img_height_;
    int img_size_;

    int total_valid_files_ = 0;
    std::vector<std::string> valid_files_;
    std::vector<int> frame_count_;

private:
    explicit SystemParams(const std::string &config_path = "config.json");
    SystemParams(const SystemParams &) = delete;
    SystemParams &operator=(const SystemParams &) = delete;

    void internalInitialize();
    void loadConfigFile();
    void loadPosesFiles();

    json config_;
    std::mutex mtx_;
    static inline std::unique_ptr<SystemParams> instance_ = nullptr;
    static inline std::once_flag init_flag_;
};

enum class TaskStatus
{
    Idle,
    Running,
    Finished,
    Error
};

class GuiDispParams
{
public:
    static GuiDispParams &instance();

    void reload();
    void setConfigFilePath();
    void setDataRoot();
    void setCaltabDescription();
    void setOutputCloudRoot();
    void setCalibrationImagesNum();
    void setCalibrationMinThreshold();
    void setCalibrationFrameNum();
    void setCalibrationThickness();
    void setCalibrationMaxLightPlaneError();
    void setReconstructionMinThreshold();
    void setReconstructionRoiRow1();
    void setReconstructionRoiCol1();
    void setReconstructionRoiRow2();
    void setReconstructionRoiCol2();
    void setSaveCloudFlag();
    void setSetRoiFlag();
    void setSelectEntireFrame();

    // For Display
    bool disabled_ = true;

    char config_file_path_disp_[256] = {'\0'};
    char data_root_disp_[256] = {'\0'};
    char caltab_description_disp_[256] = {'\0'};
    char output_cloud_root_disp_[256] = {'\0'};

    int calibration_images_num_disp_ = 0;
    int calibration_min_threshold_disp_ = 0;
    int calibration_frames_num_disp_ = 0;
    float calibration_thickness_disp_ = 0.0;
    float calibration_max_light_plane_error_disp_ = 0.0;

    int reconstruction_min_threshold_disp_ = 0;
    int reconstruction_roi_row1_disp_ = 0;
    int reconstruction_roi_col1_disp_ = 0;
    int reconstruction_roi_row2_disp_ = 0;
    int reconstruction_roi_col2_disp_ = 0;

    bool save_cloud_flag_ = false;
    bool set_roi_flag_ = false;
    bool select_entire_frame_ = false;

    // For Task
    int calibration_processed_images_num_ = 0;
    int calibration_progress_bar_right_images_num_ = 0;

    int reconstruction_processed_file_num_ = 0;
    int reconstruction_processed_frame_num_ = 0;
    int reconstruction_progress_bar_right_valid_files_num_ = 0;
    int reconstruction_progress_bar_right_frame_num_ = 0;

private:
    GuiDispParams();
    GuiDispParams(const GuiDispParams &) = delete;
    GuiDispParams &operator=(const GuiDispParams &) = delete;
};
