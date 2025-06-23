#include "calibration_task.h"

#include "Halcon.h"

#include "logger.h"

CalibrationTask::CalibrationTask(int id) : Task(id)
{
}

void CalibrationTask::run()
{
    try
    {
        GuiDispParams::instance().disabled_ = true;

        // this->calibrate();
        HTuple hv_calibration_data_id;
        this->calibrateCamera(hv_calibration_data_id);
        this->calibrateLightPlane(hv_calibration_data_id);
        this->calibrateMovement(hv_calibration_data_id);
        this->saveToFile();

        GuiDispParams::instance().disabled_ = false;
    }
    catch (const std::exception &e)
    {
        std::string msg = std::string("[Error in Calibration Task] ") + e.what();
        Logger::instance().log(LogLevel::Error, msg);
        GuiDispParams::instance().disabled_ = false;
    }
    catch (HException &exception)
    {
        std::string msg = std::string("[Error in Calibration Task] ") + exception.ErrorMessage().Text();
        Logger::instance().log(LogLevel::Error, msg);
        GuiDispParams::instance().disabled_ = false;
    }
}

void CalibrationTask::calibrate()
{
    // Set calibrate threshold
    cv::Mat cv_laser_img1 = cv::imread((SystemParams::instance().hv_program_params_.hv_laser1_path + ".png")[0].S().Text());
    cv::Mat cv_laser_img2 = cv::imread((SystemParams::instance().hv_program_params_.hv_laser2_path + ".png")[0].S().Text());

    const int threshold1 = this->setThreshold(cv_laser_img1);
    const int threshold2 = this->setThreshold(cv_laser_img2);
    SystemParams::instance().hv_program_params_.hv_calibration_min_threshold = threshold1 < threshold2 ? threshold1 : threshold2;
    GuiDispParams::instance().setCalibrationMinThreshold();

    // Part 1: Perform the calibration of the camera
    // Perform some initializations

    // Initialize a calibration data model
    HTuple hv_calib_data_id;

    // Generate a camera parameter tuple for an area scan camera with distortions modeled by the polynomial model
    HTuple hv_start_parameters;
    hv_start_parameters.Clear();
    hv_start_parameters[0] = "area_scan_polynomial";
    hv_start_parameters.Append(0.008);     // Focus
    hv_start_parameters.Append(0.0);       // K1
    hv_start_parameters.Append(0.0);       // K2
    hv_start_parameters.Append(0.0);       // K3
    hv_start_parameters.Append(0.0);       // P1
    hv_start_parameters.Append(0.0);       // P2
    hv_start_parameters.Append(0.0000047); // Sx
    hv_start_parameters.Append(0.0000047); // Sy
    hv_start_parameters.Append(640);       // Cx
    hv_start_parameters.Append(400);       // Cy
    hv_start_parameters.Append(1280);      // image width
    hv_start_parameters.Append(800);       // image height

    CreateCalibData("calibration_object", 1, 1, &hv_calib_data_id);
    SetCalibDataCamParam(hv_calib_data_id, 0, HTuple(), hv_start_parameters);
    SetCalibDataCalibObject(hv_calib_data_id, 0, SystemParams::instance().hv_program_params_.hv_caltab_description_path);

    // Collect mark positions and estimated poses for all calibration images
    HObject ho_image;
    HTuple hv_idx, hv_row, hv_col, hv_idx_d;
    HTuple hv_pose, hv_camera_calibration_error;
    HTuple end_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num;
    HTuple step_idx = 1;
    int error_images_num = 0;
    GuiDispParams::instance().calibration_processed_images_num_ = 0;
    GuiDispParams::instance().calibration_progress_bar_right_images_num_ = end_idx[0].I();
    for (hv_idx = 1; hv_idx.Continue(end_idx, step_idx); hv_idx += step_idx)
    {
        GuiDispParams::instance().calibration_processed_images_num_++;
        try
        {
            ReadImage(&ho_image, SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2"));
            std::string msg = "Perform the calibration of the camera: " + std::to_string(hv_idx[0].I()) + "/" + std::to_string(SystemParams::instance().hv_program_params_.hv_calibration_images_num[0].I());
            Logger::instance().log(LogLevel::Info, msg);
            FindCalibObject(ho_image, hv_calib_data_id, 0, 0, hv_idx, HTuple(), HTuple());
            GetCalibDataObservPoints(hv_calib_data_id, 0, 0, hv_idx, &hv_row, &hv_col, &hv_idx_d, &hv_pose);
        }
        catch (HException &exception)
        {
            error_images_num++;
            std::string msg1 = "Error " + std::to_string(exception.ErrorCode()) + " in " + exception.ProcName().Text() + ": " + exception.ErrorMessage().Text();
            std::string msg2 = std::string("Skip ") + (SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2")).S().Text();
            Logger::instance().log(LogLevel::Error, msg1);
            Logger::instance().log(LogLevel::Warn, msg2);
        }
    }

    CalibrateCameras(hv_calib_data_id, &hv_camera_calibration_error);
    Logger::instance().log(LogLevel::Info, "The camera calibration has been performed successfully!");
    Logger::instance().log(LogLevel::Info, "Number of error images: " + std::to_string(error_images_num));
    Logger::instance().log(LogLevel::Info, "Calibration error: " + std::to_string(hv_camera_calibration_error[0].D()));

    // Part 2: Calibrate the orientation of the light plane with respect to the world coordinate system
    // Definition of the world coordinate system (WCS):
    // Here, the WCS is defined implicitly by choosing one
    // specific calibration image. In order to take the thickness
    // of the calibration table into account, we shift the origin
    // of the pose that corresponds to the chosen calibration image
    HObject ho_caltab_image1, ho_caltab_image2;
    HTuple hv_caltab_pose;
    hv_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num - 1;
    HTuple ho_caltab_image1_path = SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2");
    GetCalibData(hv_calib_data_id, "calib_obj_pose", HTuple(0).TupleConcat(hv_idx), "pose", &hv_caltab_pose);
    SetOriginPose(hv_caltab_pose, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &SystemParams::instance().hv_system_poses_.hv_camera_pose);
    ReadImage(&ho_caltab_image1, ho_caltab_image1_path);
    GetCalibData(hv_calib_data_id, "camera", 0, "params", &SystemParams::instance().hv_system_poses_.hv_camera_params);

    // Definition of a temporary coordinate system (TCS):
    // The TCS is also defined implicitly by choosing another
    // calibration image. Here again we shift the origin of
    // the coordinate system in order to take the thickness
    // of the calibration table into account.
    HTuple hv_tmp_camera_pose;
    hv_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num;
    HTuple ho_caltab_image2_path = SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2");
    GetCalibData(hv_calib_data_id, "calib_obj_pose", HTuple(0).TupleConcat(hv_idx), "pose", &hv_caltab_pose);
    SetOriginPose(hv_caltab_pose, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_tmp_camera_pose);
    ReadImage(&ho_caltab_image2, ho_caltab_image2_path);

    // Compute the 3D coordinates of the light line points in the plane z=0 of the WCS lightline_corrected_01
    HObject ho_profile_img1;
    HTuple hv_x_line1, hv_y_line1, hv_z_line1;
    ReadImage(&ho_profile_img1, SystemParams::instance().hv_program_params_.hv_laser1_path);
    this->compute3dCoordinatesOfLightLine(ho_profile_img1, SystemParams::instance().hv_program_params_.hv_calibration_min_threshold,
                                          SystemParams::instance().hv_system_poses_.hv_camera_params, HTuple(),
                                          SystemParams::instance().hv_system_poses_.hv_camera_pose,
                                          &hv_x_line1, &hv_y_line1, &hv_z_line1);
    if (HTuple(HTuple(hv_x_line1.TupleLength() == 0).TupleOr(hv_y_line1.TupleLength() == 0)).TupleOr(hv_z_line1.TupleLength() == 0) != 0)
    {
        std::string msg = SystemParams::instance().hv_program_params_.hv_laser1_path[0].S().Text() + std::string(" MUST be oriented horizontally for successfully processing!");
        throw std::runtime_error(msg);
    }

    // Comput the 3D coordinates of the light line points in the plane z = 0 of the TCS
    HObject ho_profile_img2;
    HTuple hv_x_line2, hv_y_line2, hv_z_line2;
    ReadImage(&ho_profile_img2, SystemParams::instance().hv_program_params_.hv_laser2_path);
    this->compute3dCoordinatesOfLightLine(ho_caltab_image2, SystemParams::instance().hv_program_params_.hv_calibration_min_threshold,
                                          SystemParams::instance().hv_system_poses_.hv_camera_params, hv_tmp_camera_pose,
                                          SystemParams::instance().hv_system_poses_.hv_camera_pose,
                                          &hv_x_line2, &hv_y_line2, &hv_z_line2);
    if (HTuple(HTuple(hv_x_line2.TupleLength() == 0).TupleOr(hv_y_line2.TupleLength() == 0)).TupleOr(hv_z_line2.TupleLength() == 0) != 0)
    {
        std::string msg = SystemParams::instance().hv_program_params_.hv_laser2_path[0].S().Text() + std::string(" MUST be oriented horizontally for successfully processing!");
        throw std::runtime_error(msg);
    }

    // Fit the light plane in the 3D coordinates of the line points computed previously.
    // Note that this requires nearly coplanar points.
    // We must provide line points  recorded at -at least- two different heights, in order to get an unambiguous solution.
    // To obtain stable and accurate results, acquire the light line points at the bottom and at the top of the measurement volume.
    HTuple hv_ox, hv_oy, hv_oz, hv_nx, hv_ny, hv_nz, hv_mean_residual;
    this->fit3dPlaneXyz(hv_x_line1.TupleConcat(hv_x_line2), hv_y_line1.TupleConcat(hv_y_line2), hv_z_line1.TupleConcat(hv_z_line2),
                        &hv_ox, &hv_oy, &hv_oz, &hv_nx, &hv_ny, &hv_nz, &hv_mean_residual);
    if (HTuple(HTuple(hv_nx.TupleLength() == 0).TupleOr(hv_ny.TupleLength() == 0)).TupleOr(hv_nz.TupleLength() == 0) != 0)
    {

        std::string msg = "Too few 3d points have been provided to fit the light plane,or the points are (nearly) collinear!";
        throw std::runtime_error(msg);
    }

    if (hv_mean_residual > SystemParams::instance().hv_program_params_.hv_calibration_min_threshold)
    {
        std::string msg = "The error in light plane calibration: " + std::to_string(hv_mean_residual[0].D()) + ", the light plane could not be fitted accurately!";
        throw std::runtime_error(msg);
    }

    // Compute the light plane pose: this pose must be oriented such that the plane defined by z = 0 coincides with the light plane.
    this->getLightPlanePose(hv_ox, hv_oy, hv_oz, hv_nx, hv_ny, hv_nz, &SystemParams::instance().hv_system_poses_.hv_light_plane_poses);
    if ((SystemParams::instance().hv_system_poses_.hv_light_plane_poses.TupleLength() != 7) != 0)
    {
        std::string msg = "The pose of the light plane could not be determined. Please verify that the vector passed at input of the procedure getLightPlanePose() is not null!";
        throw std::runtime_error(msg);
    }

    Logger::instance().log(LogLevel::Info, "The light plane calibration has been performed successfully!");
    Logger::instance().log(LogLevel::Info, "Calibration error: " + std::to_string(hv_mean_residual[0].D()));

    // Part 3: Calibration of the movement of the object between the acquisition of two successive profiles

    // In order to determine the movement of the object between two successive profile images, we acquire two images of a calibration table which describe the same movement.
    // In order to get a good accuracy, we usually move the calibration table by more than one step.
    HObject ho_caltab_image_pos1, ho_caltab_image_pos2;
    ReadImage(&ho_caltab_image_pos1, SystemParams::instance().hv_program_params_.hv_movement1_path);
    ReadImage(&ho_caltab_image_pos2, SystemParams::instance().hv_program_params_.hv_movement2_path);

    // Set the optimized camera parameter as new start camera parameters for the calibration data model to extract the following poses using these calibrated parameters.
    SetCalibDataCamParam(hv_calib_data_id, 0, HTuple(), SystemParams::instance().hv_system_poses_.hv_camera_params);

    // Compute the pose of the calibration table in each image
    FindCalibObject(ho_caltab_image_pos1, hv_calib_data_id, 0, 0,
                    SystemParams::instance().hv_program_params_.hv_calibration_images_num + 1, HTuple(), HTuple());

    // Extract the unoptimized pose from the calibration data model
    HTuple hv_row1, hv_col1, hv_idx1, hv_camera_pose_pos1, hv_camera_pose_pos2;
    GetCalibDataObservPoints(hv_calib_data_id, 0, 0, SystemParams::instance().hv_program_params_.hv_calibration_images_num + 1,
                             &hv_row1, &hv_col1, &hv_idx1, &hv_camera_pose_pos1);
    FindCalibObject(ho_caltab_image_pos2, hv_calib_data_id, 0, 0,
                    SystemParams::instance().hv_program_params_.hv_calibration_images_num + 2, HTuple(), HTuple());
    GetCalibDataObservPoints(hv_calib_data_id, 0, 0, SystemParams::instance().hv_program_params_.hv_calibration_images_num + 2,
                             &hv_row1, &hv_col1, &hv_idx1, &hv_camera_pose_pos2);

    // Compute the coordinates of the origin of the calibration table in the two positions with respect to the world coordinate system and determine the coordinates of the corresponding translation vector
    HTuple hv_hom_mat_3d_pos1_to_camera, hv_hom_mat_3d_pos2_to_camera;
    HTuple hv_hom_mat_3d_world_to_camera, hv_hom_mat_3d_camera_to_world;
    HTuple hv_hom_mat_3d_pos1_to_world, hv_hom_mat_3d_pos2_to_world;
    HTuple hv_start_x, hv_start_y, hv_start_z;
    HTuple hv_end_x, hv_end_y, hv_end_z;
    HTuple hv_movement_pose_nsteps;
    SetOriginPose(hv_camera_pose_pos1, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_camera_pose_pos1);
    SetOriginPose(hv_camera_pose_pos2, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_camera_pose_pos2);
    PoseToHomMat3d(hv_camera_pose_pos1, &hv_hom_mat_3d_pos1_to_camera);
    PoseToHomMat3d(hv_camera_pose_pos2, &hv_hom_mat_3d_pos2_to_camera);
    PoseToHomMat3d(SystemParams::instance().hv_system_poses_.hv_camera_pose, &hv_hom_mat_3d_world_to_camera);
    HomMat3dInvert(hv_hom_mat_3d_world_to_camera, &hv_hom_mat_3d_camera_to_world);
    HomMat3dCompose(hv_hom_mat_3d_camera_to_world, hv_hom_mat_3d_pos1_to_camera, &hv_hom_mat_3d_pos1_to_world);
    HomMat3dCompose(hv_hom_mat_3d_camera_to_world, hv_hom_mat_3d_pos2_to_camera, &hv_hom_mat_3d_pos2_to_world);
    AffineTransPoint3d(hv_hom_mat_3d_pos1_to_world, 0, 0, 0, &hv_start_x, &hv_start_y, &hv_start_z);
    AffineTransPoint3d(hv_hom_mat_3d_pos2_to_world, 0, 0, 0, &hv_end_x, &hv_end_y, &hv_end_z);
    CreatePose(hv_end_x - hv_start_x, hv_end_y - hv_start_y, hv_end_z - hv_start_z, 0, 0, 0, "Rp+T", "gba", "point", &hv_movement_pose_nsteps);
    SystemParams::instance().hv_system_poses_.hv_movement_poses = hv_movement_pose_nsteps / SystemParams::instance().hv_program_params_.hv_calibration_steps_num;
    Logger::instance().log(LogLevel::Info, "The light movement pose has been performed successfully!");
}

void CalibrationTask::calibrateCamera(HTuple &hv_calibration_data_id)
{
    // Generate a camera parameter tuple for an area scan camera with distortions modeled by the polynomial model
    HTuple hv_start_parameters;
    hv_start_parameters.Clear();
    hv_start_parameters[0] = "area_scan_polynomial";
    hv_start_parameters.Append(0.008);     // Focus
    hv_start_parameters.Append(0.0);       // K1
    hv_start_parameters.Append(0.0);       // K2
    hv_start_parameters.Append(0.0);       // K3
    hv_start_parameters.Append(0.0);       // P1
    hv_start_parameters.Append(0.0);       // P2
    hv_start_parameters.Append(0.0000047); // Sx
    hv_start_parameters.Append(0.0000047); // Sy
    hv_start_parameters.Append(640);       // Cx
    hv_start_parameters.Append(400);       // Cy
    hv_start_parameters.Append(1280);      // image width
    hv_start_parameters.Append(800);       // image height

    CreateCalibData("calibration_object", 1, 1, &hv_calibration_data_id);
    SetCalibDataCamParam(hv_calibration_data_id, 0, HTuple(), hv_start_parameters);
    SetCalibDataCalibObject(hv_calibration_data_id, 0, SystemParams::instance().hv_program_params_.hv_caltab_description_path);

    // Collect mark positions and estimated poses for all calibration images
    HObject ho_image;
    HTuple hv_idx, hv_row, hv_col, hv_idx_d;
    HTuple hv_pose, hv_camera_calibration_error;
    HTuple end_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num;
    HTuple step_idx = 1;
    int error_images_num = 0;
    GuiDispParams::instance().calibration_processed_images_num_ = 0;
    GuiDispParams::instance().calibration_progress_bar_right_images_num_ = end_idx[0].I();
    for (hv_idx = 1; hv_idx.Continue(end_idx, step_idx); hv_idx += step_idx)
    {
        GuiDispParams::instance().calibration_processed_images_num_++;
        try
        {
            ReadImage(&ho_image, SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2"));
            std::string msg = "Perform the calibration of the camera: " + std::to_string(hv_idx[0].I()) + "/" + std::to_string(SystemParams::instance().hv_program_params_.hv_calibration_images_num[0].I());
            Logger::instance().log(LogLevel::Info, msg);
            FindCalibObject(ho_image, hv_calibration_data_id, 0, 0, hv_idx, HTuple(), HTuple());
            GetCalibDataObservPoints(hv_calibration_data_id, 0, 0, hv_idx, &hv_row, &hv_col, &hv_idx_d, &hv_pose);
        }
        catch (HException &exception)
        {
            error_images_num++;
            std::string msg1 = "Error " + std::to_string(exception.ErrorCode()) + " in " + exception.ProcName().Text() + ": " + exception.ErrorMessage().Text();
            std::string msg2 = std::string("Skip ") + (SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2")).S().Text();
            Logger::instance().log(LogLevel::Error, msg1);
            Logger::instance().log(LogLevel::Warn, msg2);
        }
    }

    CalibrateCameras(hv_calibration_data_id, &hv_camera_calibration_error);
    Logger::instance().log(LogLevel::Info, "The camera calibration has been performed successfully!");
    Logger::instance().log(LogLevel::Info, "Number of error images: " + std::to_string(error_images_num));
    Logger::instance().log(LogLevel::Info, "Calibration error: " + std::to_string(hv_camera_calibration_error[0].D()));
}

void CalibrationTask::calibrateLightPlane(HTuple &hv_calibration_data_id)
{
    cv::Mat cv_laser_img1 = cv::imread((SystemParams::instance().hv_program_params_.hv_laser1_path + ".png")[0].S().Text());
    cv::Mat cv_laser_img2 = cv::imread((SystemParams::instance().hv_program_params_.hv_laser2_path + ".png")[0].S().Text());

    if (cv_laser_img1.empty() || cv_laser_img2.empty())
    {
        std::string msg = "Error reading laser images";
        throw std::runtime_error(msg);
    }

    const int threshold1 = this->setThreshold(cv_laser_img1);
    const int threshold2 = this->setThreshold(cv_laser_img2);
    SystemParams::instance().hv_program_params_.hv_calibration_min_threshold = threshold1 < threshold2 ? threshold1 : threshold2;
    GuiDispParams::instance().setCalibrationMinThreshold();

    // Definition of the world coordinate system (WCS)
    HObject ho_caltab_image1, ho_caltab_image2;
    HTuple hv_caltab_pose;
    HTuple hv_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num - 1;
    HTuple ho_caltab_image1_path = SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2");
    GetCalibData(hv_calibration_data_id, "calib_obj_pose", HTuple(0).TupleConcat(hv_idx), "pose", &hv_caltab_pose);
    SetOriginPose(hv_caltab_pose, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &SystemParams::instance().hv_system_poses_.hv_camera_pose);
    ReadImage(&ho_caltab_image1, ho_caltab_image1_path);
    GetCalibData(hv_calibration_data_id, "camera", 0, "params", &SystemParams::instance().hv_system_poses_.hv_camera_params);

    // Definition of a temporary coordinate system (TCS)
    HTuple hv_tmp_camera_pose;
    hv_idx = SystemParams::instance().hv_program_params_.hv_calibration_images_num;
    HTuple ho_caltab_image2_path = SystemParams::instance().hv_program_params_.hv_calibration_images_path + hv_idx.TupleString(".2");
    GetCalibData(hv_calibration_data_id, "calib_obj_pose", HTuple(0).TupleConcat(hv_idx), "pose", &hv_caltab_pose);
    SetOriginPose(hv_caltab_pose, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_tmp_camera_pose);
    ReadImage(&ho_caltab_image2, ho_caltab_image2_path);

    // Compute the 3D coordinates of the light line points in the plane z = 0 of the WCS
    HObject ho_profile_img1;
    HTuple hv_x_line1, hv_y_line1, hv_z_line1;
    ReadImage(&ho_profile_img1, SystemParams::instance().hv_program_params_.hv_laser1_path);
    this->compute3dCoordinatesOfLightLine(ho_profile_img1, SystemParams::instance().hv_program_params_.hv_calibration_min_threshold,
                                          SystemParams::instance().hv_system_poses_.hv_camera_params, HTuple(),
                                          SystemParams::instance().hv_system_poses_.hv_camera_pose,
                                          &hv_x_line1, &hv_y_line1, &hv_z_line1);
    if (HTuple(HTuple(hv_x_line1.TupleLength() == 0).TupleOr(hv_y_line1.TupleLength() == 0)).TupleOr(hv_z_line1.TupleLength() == 0) != 0)
    {
        std::string msg = SystemParams::instance().hv_program_params_.hv_laser1_path[0].S().Text() + std::string(" MUST be oriented horizontally for successfully processing!");
        throw std::runtime_error(msg);
    }

    // Comput the 3D coordinates of the light line points in the plane z = 0 of the TCS
    HObject ho_profile_img2;
    HTuple hv_x_line2, hv_y_line2, hv_z_line2;
    ReadImage(&ho_profile_img2, SystemParams::instance().hv_program_params_.hv_laser2_path);
    this->compute3dCoordinatesOfLightLine(ho_profile_img2, SystemParams::instance().hv_program_params_.hv_calibration_min_threshold,
                                          SystemParams::instance().hv_system_poses_.hv_camera_params, hv_tmp_camera_pose,
                                          SystemParams::instance().hv_system_poses_.hv_camera_pose,
                                          &hv_x_line2, &hv_y_line2, &hv_z_line2);
    if (HTuple(HTuple(hv_x_line2.TupleLength() == 0).TupleOr(hv_y_line2.TupleLength() == 0)).TupleOr(hv_z_line2.TupleLength() == 0) != 0)
    {
        std::string msg = SystemParams::instance().hv_program_params_.hv_laser2_path[0].S().Text() + std::string(" MUST be oriented horizontally for successfully processing!");
        throw std::runtime_error(msg);
    }

    // Fit the light plane in the 3D coordinates of the line points computed previously.
    // Note that this requires nearly coplanar points.
    // We must provide line points  recorded at -at least- two different heights, in order to get an unambiguous solution.
    // To obtain stable and accurate results, acquire the light line points at the bottom and at the top of the measurement volume.
    HTuple hv_ox, hv_oy, hv_oz, hv_nx, hv_ny, hv_nz, hv_mean_residual;
    this->fit3dPlaneXyz(hv_x_line1.TupleConcat(hv_x_line2), hv_y_line1.TupleConcat(hv_y_line2), hv_z_line1.TupleConcat(hv_z_line2),
                        &hv_ox, &hv_oy, &hv_oz, &hv_nx, &hv_ny, &hv_nz, &hv_mean_residual);
    if (HTuple(HTuple(hv_nx.TupleLength() == 0).TupleOr(hv_ny.TupleLength() == 0)).TupleOr(hv_nz.TupleLength() == 0) != 0)
    {

        std::string msg = "Too few 3d points have been provided to fit the light plane,or the points are (nearly) collinear!";
        throw std::runtime_error(msg);
    }

    if (hv_mean_residual > SystemParams::instance().hv_program_params_.hv_calibration_min_threshold)
    {
        std::string msg = "The error in light plane calibration: " + std::to_string(hv_mean_residual[0].D()) + ", the light plane could not be fitted accurately!";
        throw std::runtime_error(msg);
    }

    // Compute the light plane pose: this pose must be oriented such that the plane defined by z = 0 coincides with the light plane.
    this->getLightPlanePose(hv_ox, hv_oy, hv_oz, hv_nx, hv_ny, hv_nz, &SystemParams::instance().hv_system_poses_.hv_light_plane_poses);
    if ((SystemParams::instance().hv_system_poses_.hv_light_plane_poses.TupleLength() != 7) != 0)
    {
        std::string msg = "The pose of the light plane could not be determined. Please verify that the vector passed at input of the procedure getLightPlanePose() is not null!";
        throw std::runtime_error(msg);
    }

    Logger::instance().log(LogLevel::Info, "The light plane calibration has been performed successfully!");
    Logger::instance().log(LogLevel::Info, "Calibration error: " + std::to_string(hv_mean_residual[0].D()));
}

void CalibrationTask::calibrateMovement(HTuple &hv_calibration_data_id)
{
    // In order to determine the movement of the object between two successive profile images, we acquire two images of a calibration table which describe the same movement.
    // In order to get a good accuracy, we usually move the calibration table by more than one step.
    HObject ho_caltab_image_pos1, ho_caltab_image_pos2;
    ReadImage(&ho_caltab_image_pos1, SystemParams::instance().hv_program_params_.hv_movement1_path);
    ReadImage(&ho_caltab_image_pos2, SystemParams::instance().hv_program_params_.hv_movement2_path);

    // Set the optimized camera parameter as new start camera parameters for the calibration data model to extract the following poses using these calibrated parameters.
    SetCalibDataCamParam(hv_calibration_data_id, 0, HTuple(), SystemParams::instance().hv_system_poses_.hv_camera_params);

    // Compute the pose of the calibration table in each image
    FindCalibObject(ho_caltab_image_pos1, hv_calibration_data_id, 0, 0,
                    SystemParams::instance().hv_program_params_.hv_calibration_images_num + 1, HTuple(), HTuple());

    // Extract the unoptimized pose from the calibration data model
    HTuple hv_row1, hv_col1, hv_idx1, hv_camera_pose_pos1, hv_camera_pose_pos2;
    GetCalibDataObservPoints(hv_calibration_data_id, 0, 0, SystemParams::instance().hv_program_params_.hv_calibration_images_num + 1,
                             &hv_row1, &hv_col1, &hv_idx1, &hv_camera_pose_pos1);
    FindCalibObject(ho_caltab_image_pos2, hv_calibration_data_id, 0, 0,
                    SystemParams::instance().hv_program_params_.hv_calibration_images_num + 2, HTuple(), HTuple());
    GetCalibDataObservPoints(hv_calibration_data_id, 0, 0, SystemParams::instance().hv_program_params_.hv_calibration_images_num + 2,
                             &hv_row1, &hv_col1, &hv_idx1, &hv_camera_pose_pos2);

    // Compute the coordinates of the origin of the calibration table in the two positions with respect to the world coordinate system and determine the coordinates of the corresponding translation vector
    HTuple hv_hom_mat_3d_pos1_to_camera, hv_hom_mat_3d_pos2_to_camera;
    HTuple hv_hom_mat_3d_world_to_camera, hv_hom_mat_3d_camera_to_world;
    HTuple hv_hom_mat_3d_pos1_to_world, hv_hom_mat_3d_pos2_to_world;
    HTuple hv_start_x, hv_start_y, hv_start_z;
    HTuple hv_end_x, hv_end_y, hv_end_z;
    HTuple hv_movement_pose_nsteps;
    SetOriginPose(hv_camera_pose_pos1, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_camera_pose_pos1);
    SetOriginPose(hv_camera_pose_pos2, 0.0, 0.0, SystemParams::instance().hv_program_params_.hv_calibration_thickness, &hv_camera_pose_pos2);
    PoseToHomMat3d(hv_camera_pose_pos1, &hv_hom_mat_3d_pos1_to_camera);
    PoseToHomMat3d(hv_camera_pose_pos2, &hv_hom_mat_3d_pos2_to_camera);
    PoseToHomMat3d(SystemParams::instance().hv_system_poses_.hv_camera_pose, &hv_hom_mat_3d_world_to_camera);
    HomMat3dInvert(hv_hom_mat_3d_world_to_camera, &hv_hom_mat_3d_camera_to_world);
    HomMat3dCompose(hv_hom_mat_3d_camera_to_world, hv_hom_mat_3d_pos1_to_camera, &hv_hom_mat_3d_pos1_to_world);
    HomMat3dCompose(hv_hom_mat_3d_camera_to_world, hv_hom_mat_3d_pos2_to_camera, &hv_hom_mat_3d_pos2_to_world);
    AffineTransPoint3d(hv_hom_mat_3d_pos1_to_world, 0, 0, 0, &hv_start_x, &hv_start_y, &hv_start_z);
    AffineTransPoint3d(hv_hom_mat_3d_pos2_to_world, 0, 0, 0, &hv_end_x, &hv_end_y, &hv_end_z);
    CreatePose(hv_end_x - hv_start_x, hv_end_y - hv_start_y, hv_end_z - hv_start_z, 0, 0, 0, "Rp+T", "gba", "point", &hv_movement_pose_nsteps);
    SystemParams::instance().hv_system_poses_.hv_movement_poses = hv_movement_pose_nsteps / SystemParams::instance().hv_program_params_.hv_calibration_steps_num;
    Logger::instance().log(LogLevel::Info, "The light movement pose has been performed successfully!");
}

void CalibrationTask::saveToFile()
{
    // Save the calibration parameters
    // Create "Saved Poses" directory if it don't exist.
    HTuple hv_saved_poses_dir = SystemParams::instance().hv_program_params_.hv_data_root + "/saved poses";
    if (!std::filesystem::exists(hv_saved_poses_dir[0].S().Text()))
    {
        std::filesystem::create_directories(hv_saved_poses_dir[0].S().Text());
    }
    else
    {
        WriteCamPar(SystemParams::instance().hv_system_poses_.hv_camera_params, hv_saved_poses_dir + "/camera_parameters.dat");
        WritePose(SystemParams::instance().hv_system_poses_.hv_camera_pose, hv_saved_poses_dir + "/camera_pose.dat");
        WritePose(SystemParams::instance().hv_system_poses_.hv_light_plane_poses, hv_saved_poses_dir + "/light_plane_pose.dat");
        WritePose(SystemParams::instance().hv_system_poses_.hv_movement_poses, hv_saved_poses_dir + "/movement_pose.dat");

        SystemParams::instance().calibration_flag_ = false;
        Logger::instance().log(LogLevel::Info, "The calibration parameters has been saved successfully!");
    }
}

int CalibrationTask::setThreshold(const cv::Mat &src_mat)
{
    double max_size = 700;
    int window_width, window_height;
    double aspect_ratio = static_cast<double>(src_mat.cols) / static_cast<double>(src_mat.rows);

    if (src_mat.cols > src_mat.rows)
    {
#ifndef WIN32
        window_width = std::min(src_mat.cols, static_cast<int>(max_size));
#else
        window_width = min(src_mat.cols, static_cast<int>(max_size));
#endif
        window_height = static_cast<int>(window_width / aspect_ratio);
    }
    else
    {
#ifndef WIN32
        window_height = std::min(src_mat.rows, static_cast<int>(max_size));
#else
        window_height = min(src_mat.rows, static_cast<int>(max_size));
#endif
        window_width = static_cast<int>(window_height * aspect_ratio);
    }

    cv::namedWindow("Setting threshold", cv::WINDOW_NORMAL);
    cv::resizeWindow("Setting threshold", window_width, window_height);

    int temp_threshold = SystemParams::instance().hv_program_params_.hv_calibration_min_threshold[0].I();
    struct ThresholdContext
    {
        cv::Mat src;
        cv::Mat dst;
        int *threshold;
    };
    ThresholdContext context{src_mat.clone(), cv::Mat(), &temp_threshold};

    auto on_trackbar = [](int pos, void *userdata) mutable
    {
        auto *ctx = static_cast<ThresholdContext *>(userdata);
        *(ctx->threshold) = pos;
        cv::threshold(ctx->src, ctx->dst, pos, 255, cv::THRESH_BINARY);
        cv::imshow("Setting threshold", ctx->dst);
    };

    cv::createTrackbar("Threshold value", "Setting threshold", nullptr, 255, on_trackbar, &context);
    cv::setTrackbarPos("Threshold value", "Setting threshold", temp_threshold);
    on_trackbar(temp_threshold, &context);

    while (1)
    {
        if (char(cv::waitKey(1)) == 13)
        {
            break;
        }
    }
    cv::destroyAllWindows();

    SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold = temp_threshold;
    GuiDispParams::instance().reconstruction_min_threshold_disp_ = temp_threshold;

    return temp_threshold;
}

void CalibrationTask::compute3dCoordinatesOfLightLine(HObject ho_profile_image, HTuple hv_min_gray, HTuple hv_camera_parameters, HTuple hv_local_camera_pose, HTuple hv_reference_camera_pose, HTuple *hv_x, HTuple *hv_y, HTuple *hv_z)
{
    // Local iconic variables
    HObject ho_region, ho_region_dilation, ho_profile_region;
    HObject ho_domain, ho_disparity, ho_disparity_domain;

    // Local control variables
    HTuple hv_do_transform, hv_hom_mat3d_local_to_cam;
    HTuple hv_hom_mat3d_reference_to_cam, hv_hom_mat3d_cam_to_reference, hv_hom_mat3d_local_to_reference;
    HTuple hv_phi, hv_row1, hv_col1, hv_row2, hv_col2;
    HTuple hv_sheet_of_light_handle;
    HTuple hv_rows, hv_cols, hv_disparities;

    // WARNING: if the profile is not oriented roughly horizontal empty tuples are returned for X, Y and Z.
    (*hv_x) = HTuple();
    (*hv_y) = HTuple();
    (*hv_z) = HTuple();

    if ((int(hv_local_camera_pose == HTuple())) != 0)
    {
        hv_do_transform = 0;
        hv_local_camera_pose = hv_reference_camera_pose;
    }
    else if ((int(hv_local_camera_pose == hv_reference_camera_pose)) != 0)
    {
        hv_do_transform = 0;
    }
    else
    {
        hv_do_transform = 1;
    }

    // Compute the pose for the projection of the local coordinate system to the plane z=0
    PoseToHomMat3d(hv_local_camera_pose, &hv_hom_mat3d_local_to_cam);

    // Compute the homography which transform the 3D-coordinates of points from the local coordinate system to the reference coordinate system
    if (hv_do_transform != 0)
    {
        PoseToHomMat3d(hv_reference_camera_pose, &hv_hom_mat3d_reference_to_cam);
        HomMat3dInvert(hv_hom_mat3d_reference_to_cam, &hv_hom_mat3d_cam_to_reference);
        HomMat3dCompose(hv_hom_mat3d_cam_to_reference, hv_hom_mat3d_local_to_cam, &hv_hom_mat3d_local_to_reference);
    }

    // Determine the profile region and test if the profile is oriented roughly horizontal
    Threshold(ho_profile_image, &ho_region, hv_min_gray, 999999999);
    OrientationRegion(ho_region, &hv_phi);
    if (0 != (HTuple(int((hv_phi.TupleCos()) > (HTuple(HTuple(135).TupleRad()).TupleCos()))).TupleAnd(int((hv_phi.TupleCos()) < (HTuple(HTuple(45).TupleRad()).TupleCos())))))
    {
        // The detected profile is NOT oriented roughly horizontal, therefore return empty tuples X, Y and Z.
        return;
    }
    DilationCircle(ho_region, &ho_region_dilation, 5.5);
    SmallestRectangle1(ho_region_dilation, &hv_row1, &hv_col1, &hv_row2, &hv_col2);
    GenRectangle1(&ho_profile_region, hv_row1, hv_col1, hv_row2, hv_col2);
    GetDomain(ho_profile_image, &ho_domain);
    Intersection(ho_profile_region, ho_domain, &ho_profile_region);

    // Get the 2D-coordinates of the points on the light line
    CreateSheetOfLightModel(ho_profile_region, (HTuple("min_gray").Append("num_profiles")), hv_min_gray.TupleConcat(1), &hv_sheet_of_light_handle);
    MeasureProfileSheetOfLight(ho_profile_image, hv_sheet_of_light_handle, HTuple());
    GetSheetOfLightResult(&ho_disparity, hv_sheet_of_light_handle, "disparity");

    // Get the 3D-coordinates of the points on the light line in the coordinate system defined by ReferenceCameraPose
    GetDomain(ho_disparity, &ho_disparity_domain);
    GetRegionPoints(ho_disparity_domain, &hv_rows, &hv_cols);
    GetGrayval(ho_disparity, hv_rows, hv_cols, &hv_disparities);

    // Project those points to the z=0 plane of the WCS
    ImagePointsToWorldPlane(hv_camera_parameters, hv_local_camera_pose, hv_disparities, hv_cols + hv_col1, 1.0, &(*hv_x), &(*hv_y));
    TupleGenConst(hv_cols.TupleLength(), 0.0, &(*hv_z));

    // If necessary, transform the 3D-coordinates from the local coordinate system to the reference coordinate system
    if (hv_do_transform != 0)
    {
        AffineTransPoint3d(hv_hom_mat3d_local_to_reference, (*hv_x), (*hv_y), (*hv_z), &(*hv_x), &(*hv_y), &(*hv_z));
    }
    return;
}

void CalibrationTask::fit3dPlaneXyz(HTuple hv_x, HTuple hv_y, HTuple hv_z, HTuple *hv_ox, HTuple *hv_oy, HTuple *hv_oz, HTuple *hv_nx, HTuple *hv_ny, HTuple *hv_nz, HTuple *hv_mean_residual)
{
    // Local control variables
    HTuple hv_size, hv_matrix_id_mt, hv_matrix_id_m, hv_matrix_id_u, hv_matrix_id_s, hv_matrix_id_v;
    HTuple hv_singular_values_of_m, hv_indices, hv_n, hv_matrix_id_n, hv_matrix_id_mn, hv_distances;

    // This procedure fits a 3D-plane into a set of 3D-points.
    // The procedure returns the coordinates [ox, oy, oz] of the centroid of the provided input coordinates, and the coordinates [nx, ny,n,z] of the normal vector to the fitted plane.
    // WARNING: If the system of equations is under-determined (i.e. if it has too few input coordinates in X, Y, Z), it cannot be solved and the procedure returns empty tuples for X, Y, and Z
    // Perform some initializations
    (*hv_ox) = HTuple();
    (*hv_oy) = HTuple();
    (*hv_oz) = HTuple();
    (*hv_nx) = HTuple();
    (*hv_ny) = HTuple();
    (*hv_nz) = HTuple();
    (*hv_mean_residual) = HTuple();

    // Test the size of x, y and z, and return if necessary
    hv_size = hv_x.TupleLength();
    if ((HTuple(HTuple(int(hv_size < 3)).TupleOr(int(hv_size != (hv_y.TupleLength())))).TupleOr(int(hv_size != (hv_z.TupleLength())))) != 0)
    {
        return;
    }

    // Compute the coordinates of the centroid point
    TupleMean(hv_x, &(*hv_ox));
    TupleMean(hv_y, &(*hv_oy));
    TupleMean(hv_z, &(*hv_oz));

    // Setup the equation system as a matrix M and compute its singular value decomposition.
    // The singular vector of M corresponding to its smallest singular value provides the coordinates of the normal vector of the fitted plane.
    CreateMatrix(3, hv_x.TupleLength(), ((hv_x - (*hv_ox)).TupleConcat(hv_y - (*hv_oy))).TupleConcat(hv_z - (*hv_oz)), &hv_matrix_id_mt);
    TransposeMatrix(hv_matrix_id_mt, &hv_matrix_id_m);
    SvdMatrix(hv_matrix_id_m, "reduced", "right", &hv_matrix_id_u, &hv_matrix_id_s, &hv_matrix_id_v);
    GetValueMatrix(hv_matrix_id_s, ((HTuple(0).Append(1)).Append(2)), ((HTuple(0).Append(1)).Append(2)),
                   &hv_singular_values_of_m);
    TupleSortIndex(hv_singular_values_of_m, &hv_indices);

    // Test if more than one singular value of M is (nearly) equal to zero.
    // This indicates that the provided 3d points are inappropriate to fit the plane (e.g. they are nearly collinear or reduce to a single point).
    if ((HTuple(int(HTuple(hv_singular_values_of_m[HTuple(hv_indices[0])]) < 1e-9)).TupleAnd(int(HTuple(hv_singular_values_of_m[HTuple(hv_indices[1])]) < 1e-9))) != 0)
    {
        return;
    }

    // Get coordinates of the normal vector to the fitted plane
    GetValueMatrix(hv_matrix_id_v, ((HTuple(0).Append(1)).Append(2)), (HTuple(hv_indices[0]).TupleConcat(HTuple(hv_indices[0]))).TupleConcat(HTuple(hv_indices[0])), &hv_n);
    CreateMatrix(3, 1, hv_n, &hv_matrix_id_n);
    (*hv_nx) = ((const HTuple &)hv_n)[0];
    (*hv_ny) = ((const HTuple &)hv_n)[1];
    (*hv_nz) = ((const HTuple &)hv_n)[2];

    // Compute the mean residual distance between the 3d points
    // and the fitted plane, in order to guess the quality of
    // the fitted plane:
    MultMatrix(hv_matrix_id_m, hv_matrix_id_n, "AB", &hv_matrix_id_mn);
    GetFullMatrix(hv_matrix_id_mn, &hv_distances);
    hv_distances = hv_distances.TupleAbs();
    (*hv_mean_residual) = (hv_distances.TupleSum()) / hv_size;

    return;
}

void CalibrationTask::getLightPlanePose(HTuple hv_origin_x, HTuple hv_origin_y, HTuple hv_origin_z, HTuple hv_normal_vector_x, HTuple hv_normal_vector_y, HTuple hv_normal_vector_z, HTuple *hv_light_plane_pose)
{
    // Local control variables
    HTuple hv_norm, hv_hom_mat3d_identity, hv_alpha;
    HTuple hv_hom_mat3d_rotate_alpha, hv_n1x, hv_n1y, hv_n1z;
    HTuple hv_beta, hv_hom_mat3d_rotate_beta, hv_hom_mat3d_translate;
    HTuple hv_hom_mat3d_tmp, hv_hom_mat3d_world_to_light_plane;

    // This procedure determines a lightplane pose, e.g. a pose whose plane defined by z=0 coincides with the physical light plane.
    // Test that the vector passed at input is not null
    (*hv_light_plane_pose) = HTuple();
    hv_norm = ((hv_normal_vector_x * hv_normal_vector_x) + (hv_normal_vector_y * hv_normal_vector_y)) + (hv_normal_vector_z * hv_normal_vector_z);
    if ((int((hv_norm.TupleAbs()) < 1e-8)) != 0)
    {
        return;
    }

    // In order to compute a light-plane pose, we determine two rotations which align the unit vector of the z-axis to the normal vector of the light plane, when applied successively.
    // For example, we can compute the anglesAlpha (rotation around the x-axis) and Beta (subsequentrotation around the y-axis) in this successive order.
    // (The rotation around the z-axis is arbitrarily set to zero).

    // Determine the value of the angle Alpha and rotate the normal vector to the plane y=0.
    // This provides the vector N1.
    HomMat3dIdentity(&hv_hom_mat3d_identity);
    TupleAtan2(hv_normal_vector_y, hv_normal_vector_z, &hv_alpha);
    HomMat3dRotate(hv_hom_mat3d_identity, hv_alpha, "x", 0, 0, 0, &hv_hom_mat3d_rotate_alpha);
    AffineTransPoint3d(hv_hom_mat3d_rotate_alpha, hv_normal_vector_x, hv_normal_vector_y, hv_normal_vector_z, &hv_n1x, &hv_n1y, &hv_n1z);

    // Determine the value of the angle Beta by using the coordinates of N1.
    // Note that the rotation around the y-axis with angle Beta is carried out in the counter trigonometric direction, therefore we apply -Beta.
    TupleAtan2(hv_n1x, hv_n1z, &hv_beta);
    HomMat3dRotate(hv_hom_mat3d_identity, -hv_beta, "y", 0, 0, 0, &hv_hom_mat3d_rotate_beta);

    // Create the LightPlanePose
    HomMat3dTranslate(hv_hom_mat3d_identity, -hv_origin_x, -hv_origin_y, -hv_origin_z, &hv_hom_mat3d_translate);
    HomMat3dCompose(hv_hom_mat3d_rotate_alpha, hv_hom_mat3d_translate, &hv_hom_mat3d_tmp);
    HomMat3dCompose(hv_hom_mat3d_rotate_beta, hv_hom_mat3d_tmp, &hv_hom_mat3d_world_to_light_plane);
    HomMat3dToPose(hv_hom_mat3d_world_to_light_plane, &(*hv_light_plane_pose));
    return;
}
