#include "system_params.h"

#include "logger.h"

SystemParams::SystemParams(const std::string &config_path)
{
    reload(config_path);
}

SystemParams &SystemParams::instance()
{
    static SystemParams instance;
    return instance;
}

void SystemParams::reload(const std::string &new_config_path)
{
    std::lock_guard<std::mutex> lock(mtx_);
    config_path_ = new_config_path;
    internalInitialize();
}

void SystemParams::internalInitialize()
{
    try
    {
        loadConfigFile();
        validatePaths(Path::All);
        loadPosesFiles();
        Logger::instance().log(LogLevel::Info, "[SystemParams] Load successfully!");
    }
    catch (const std::exception &e)
    {
        std::string msg = "[SystemParams] Reload failed: " + std::string(e.what());
        Logger::instance().log(LogLevel::Error, msg);
    }
    catch (const HException &e)
    {
        std::string msg = "[SystemParams] Reload failed: " + std::string(e.ErrorMessage());
        Logger::instance().log(LogLevel::Error, msg);
    }
}

void SystemParams::loadConfigFile()
{
    // Check if the config file exists
    if (!std::filesystem::exists(config_path_))
    {
        throw std::runtime_error("Config file missing: " + config_path_);
    }

    std::ifstream f(config_path_);
    config_ = json::parse(f);

    // Load the program parameters
    hv_program_params_.hv_data_root = config_["params"]["dataRoot"].get<std::string>().c_str();
    hv_program_params_.hv_caltab_description_path = config_["params"]["caltabDescription"].get<std::string>().c_str();
    hv_program_params_.hv_output_cloud_root = config_["params"]["outputCloudRoot"].get<std::string>().c_str();

    hv_program_params_.hv_calibration_images_num = config_["params"]["calibrationImagesNumber"].get<int>();
    hv_program_params_.hv_calibration_min_threshold = config_["params"]["calibrationMinThreshold"].get<int>();
    hv_program_params_.hv_calibration_thickness = config_["params"]["calibrationThickness"].get<float>();
    hv_program_params_.hv_calibration_steps_num = config_["params"]["calibrationStepsNumber"].get<int>();
    hv_program_params_.hv_calibration_max_light_plane_error = config_["params"]["calibrationMaxLightPlaneError"].get<float>();

    hv_program_params_.hv_reconstruction_min_threshold = config_["params"]["reconstructionMinThreshold"].get<int>();
    hv_program_params_.hv_reconstruction_roi_row1 = config_["params"]["reconstructionRoiRow1"].get<int>();
    hv_program_params_.hv_reconstruction_roi_col1 = config_["params"]["reconstructionRoiCol1"].get<int>();
    hv_program_params_.hv_reconstruction_roi_row2 = config_["params"]["reconstructionRoiRow2"].get<int>();
    hv_program_params_.hv_reconstruction_roi_col2 = config_["params"]["reconstructionRoiCol2"].get<int>();

    img_width_ = config_["params"]["imgWidth"].get<int>();
    img_height_ = config_["params"]["imgHeight"].get<int>();
    img_size_ = img_width_ * img_height_;

    this->save_cloud_flag_ = config_["mode"]["saveCloudFlag"].get<bool>();
    this->set_roi_flag_ = config_["mode"]["setRoiFlag"].get<bool>();
    this->select_entire_frame_ = config_["mode"]["selectEntireFrame"].get<bool>();

    const std::filesystem::path data_root(hv_program_params_.hv_data_root[0].S().Text());
    hv_program_params_.hv_calibration_images_path = (data_root / "caltab" / "caltab").string().c_str();
    hv_program_params_.hv_laser1_path = (data_root / "caltab" / "laser01").string().c_str();
    hv_program_params_.hv_laser2_path = (data_root / "caltab" / "laser02").string().c_str();
    hv_program_params_.hv_movement1_path = (data_root / "caltab" / "caltab_at_position_1").string().c_str();
    hv_program_params_.hv_movement2_path = (data_root / "caltab" / "caltab_at_position_2").string().c_str();
}

void SystemParams::validatePaths(Path path) const
{
    auto CheckPath = [this](const HTuple &path, const std::string &name)
    {
        if (!std::filesystem::exists(path[0].S().Text()))
        {
            this->calibration_flag_ = true;
            throw std::runtime_error(name + " path invalid " + path[0].S().Text());
        }
    };

    switch (path)
    {
    case Path::DataRoot:
    {
        CheckPath(hv_program_params_.hv_data_root, "Data root");
    }
    break;

    case Path::CaltabDescription:
    {
        CheckPath(hv_program_params_.hv_caltab_description_path, "Caltab description");
    }
    break;

    case Path::OutputModelRoot:
    {
        CheckPath(hv_program_params_.hv_output_cloud_root, "Output model root");
    }
    break;

    case Path::CameraParameters:
    {
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/camera_parameters.dat", "CameraParameters");
    }
    break;

    case Path::CameraPose:
    {
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/camera_pose.dat", "CameraPose");
    }
    break;

    case Path::LightPlanePose:
    {
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/light_plane_pose.dat", "LightPlanePose");
    }
    break;

    case Path::MovementPose:
    {
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/movement_pose.dat", "MovementPose");
    }
    break;

    case Path::All:
    default:
    {
        CheckPath(hv_program_params_.hv_data_root, "Data root");
        CheckPath(hv_program_params_.hv_caltab_description_path, "Caltab description");
        CheckPath(hv_program_params_.hv_output_cloud_root, "Output model root");
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/camera_parameters.dat", "CameraParameters");
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/camera_pose.dat", "CameraPose");
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/light_plane_pose.dat", "LightPlanePose");
        CheckPath(hv_program_params_.hv_data_root + "/saved poses/movement_pose.dat", "MovementPose");
    }
    break;
    }
}

void SystemParams::loadPosesFiles()
{
    ReadCamPar(hv_program_params_.hv_data_root + "/saved poses/camera_parameters.dat", &hv_system_poses_.hv_camera_params);
    ReadPose(hv_program_params_.hv_data_root + "/saved poses/camera_pose.dat", &hv_system_poses_.hv_camera_pose);
    ReadPose(hv_program_params_.hv_data_root + "/saved poses/light_plane_pose.dat", &hv_system_poses_.hv_light_plane_poses);
    ReadPose(hv_program_params_.hv_data_root + "/saved poses/movement_pose.dat", &hv_system_poses_.hv_movement_poses);
    calibration_flag_ = false;
}

GuiDispParams::GuiDispParams()
{
    strncpy(config_file_path_disp_, SystemParams::instance().config_path_.c_str(), sizeof(config_file_path_disp_) - 1);
    config_file_path_disp_[sizeof(config_file_path_disp_) - 1] = '\0';

    strncpy(data_root_disp_, SystemParams::instance().hv_program_params_.hv_data_root[0].S().Text(), sizeof(data_root_disp_) - 1);
    data_root_disp_[sizeof(data_root_disp_) - 1] = '\0';

    strncpy(caltab_description_disp_, SystemParams::instance().hv_program_params_.hv_caltab_description_path[0].S().Text(), sizeof(caltab_description_disp_) - 1);
    caltab_description_disp_[sizeof(caltab_description_disp_) - 1] = '\0';

    strncpy(output_cloud_root_disp_, SystemParams::instance().hv_program_params_.hv_output_cloud_root[0].S().Text(), sizeof(output_cloud_root_disp_) - 1);
    output_cloud_root_disp_[sizeof(output_cloud_root_disp_) - 1] = '\0';

    this->calibration_images_num_disp_ = SystemParams::instance().hv_program_params_.hv_calibration_images_num[0].I();
    this->calibration_min_threshold_disp_ = SystemParams::instance().hv_program_params_.hv_calibration_min_threshold[0].I();
    this->calibration_frames_num_disp_ = SystemParams::instance().hv_program_params_.hv_calibration_steps_num[0].I();
    this->calibration_thickness_disp_ = SystemParams::instance().hv_program_params_.hv_calibration_thickness[0].D();
    this->calibration_max_light_plane_error_disp_ = SystemParams::instance().hv_program_params_.hv_calibration_max_light_plane_error[0].D();

    this->reconstruction_min_threshold_disp_ = SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold[0].I();
    this->reconstruction_roi_row1_disp_ = SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1[0].I();
    this->reconstruction_roi_col1_disp_ = SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1[0].I();
    this->reconstruction_roi_row2_disp_ = SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2[0].I();
    this->reconstruction_roi_col2_disp_ = SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2[0].I();

    this->save_cloud_flag_ = SystemParams::instance().save_cloud_flag_;
    this->set_roi_flag_ = SystemParams::instance().set_roi_flag_;
    this->select_entire_frame_ = SystemParams::instance().select_entire_frame_;
}

GuiDispParams &GuiDispParams::instance()
{
    static GuiDispParams instance;
    return instance;
}

void GuiDispParams::setConfigFilePath()
{
    SystemParams::instance().config_path_ = this->config_file_path_disp_;
    SystemParams::instance().reload(this->config_file_path_disp_);

    // Update Gui
    this->reload();
}

void GuiDispParams::reload()
{
    this->setDataRoot();
    this->setCaltabDescription();
    this->setOutputCloudRoot();
    this->setCalibrationImagesNum();
    this->setCalibrationMinThreshold();
    this->setCalibrationFrameNum();
    this->setCalibrationThickness();
    this->setCalibrationMaxLightPlaneError();
    this->setReconstructionMinThreshold();
    this->setSaveCloudFlag();
    this->setSetRoiFlag();
    this->setSelectEntireFrame();
}

void GuiDispParams::setDataRoot()
{
    SystemParams::instance().hv_program_params_.hv_data_root = this->data_root_disp_;
    SystemParams::instance().validatePaths(Path::All);
}

void GuiDispParams::setCaltabDescription()
{
    SystemParams::instance().hv_program_params_.hv_caltab_description_path = this->caltab_description_disp_;
    SystemParams::instance().validatePaths(Path::CaltabDescription);
}

void GuiDispParams::setOutputCloudRoot()
{
    SystemParams::instance().hv_program_params_.hv_output_cloud_root = this->output_cloud_root_disp_;
    SystemParams::instance().validatePaths(Path::OutputModelRoot);
}

void GuiDispParams::setCalibrationImagesNum()
{
    SystemParams::instance().hv_program_params_.hv_calibration_images_num = this->calibration_images_num_disp_;
}

void GuiDispParams::setCalibrationMinThreshold()
{
    SystemParams::instance().hv_program_params_.hv_calibration_min_threshold = this->calibration_min_threshold_disp_;
}

void GuiDispParams::setCalibrationFrameNum()
{
    SystemParams::instance().hv_program_params_.hv_calibration_steps_num = this->calibration_frames_num_disp_;
}

void GuiDispParams::setCalibrationThickness()
{
    SystemParams::instance().hv_program_params_.hv_calibration_thickness = this->calibration_thickness_disp_;
}

void GuiDispParams::setCalibrationMaxLightPlaneError()
{
    SystemParams::instance().hv_program_params_.hv_calibration_max_light_plane_error = this->calibration_max_light_plane_error_disp_;
}

void GuiDispParams::setReconstructionMinThreshold()
{
    SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold = this->reconstruction_min_threshold_disp_;
}

void GuiDispParams::setReconstructionRoiRow1()
{
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1 = this->reconstruction_roi_row1_disp_;
}

void GuiDispParams::setReconstructionRoiCol1()
{
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1 = this->reconstruction_roi_col1_disp_;
}

void GuiDispParams::setReconstructionRoiRow2()
{
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2 = this->reconstruction_roi_row2_disp_;
}

void GuiDispParams::setReconstructionRoiCol2()
{
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2 = this->reconstruction_roi_col2_disp_;
}

void GuiDispParams::setSaveCloudFlag()
{
    SystemParams::instance().save_cloud_flag_ = this->save_cloud_flag_;
}

void GuiDispParams::setSetRoiFlag()
{
    SystemParams::instance().set_roi_flag_ = this->set_roi_flag_;
}

void GuiDispParams::setSelectEntireFrame()
{
    SystemParams::instance().select_entire_frame_ = this->select_entire_frame_;
}