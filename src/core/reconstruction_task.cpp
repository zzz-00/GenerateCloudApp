#include "reconstruction_task.h"

#include <filesystem>
#include <iostream>

#include "logger.h"

ReconstructionTask::ReconstructionTask(int id) : Task(id)
{
}

void ReconstructionTask::run()
{
    try
    {
        GuiDispParams::instance().disabled_ = true;

        if (SystemParams::instance().calibration_flag_)
        {
            Logger::instance().log(LogLevel::Warn, "Please calibrate first!");
            return;
        }

        this->findBilData();

        if (SystemParams::instance().valid_files_.empty())
        {
            Logger::instance().log(LogLevel::Error, "No valid .bil files found.");
            GuiDispParams::instance().disabled_ = false;
            return;
        }

        this->preprocess();
        this->reconstruction();
        GuiDispParams::instance().disabled_ = false;
    }
    catch (const std::exception &e)
    {
        std::string msg = std::string("[Error in Reconstruction Task] ") + e.what();
        Logger::instance().log(LogLevel::Error, msg);
        GuiDispParams::instance().disabled_ = false;
    }
}

void ReconstructionTask::findBilData()
{
    const std::streampos img_size = SystemParams::instance().img_width_ * SystemParams::instance().img_height_;
    namespace fs = std::filesystem;

    size_t file_count = 0;
    std::uintmax_t total_size = 0;
    size_t total_frame_counts = 0;
    std::string bil_data_dir = std::string(SystemParams::instance().hv_program_params_.hv_data_root[0].S().Text()) + "/bil";

    try
    {
        Logger::instance().log(LogLevel::Info, "Start to find .bil files");
        if (!SystemParams::instance().valid_files_.empty())
        {
            SystemParams::instance().valid_files_.clear();
        }
        for (const auto &entry : fs::recursive_directory_iterator(bil_data_dir))
        {
            if (running_.load())
            {
                if (!entry.is_regular_file())
                {
                    continue;
                }

                const fs::path &path = entry.path();
                if (path.extension() != ".bil")
                {
                    continue;
                }
                std::string normalized_path = path.string();
                std::replace(normalized_path.begin(), normalized_path.end(), '\\', '/');
                SystemParams::instance().valid_files_.emplace_back(std::move(normalized_path));

                std::uintmax_t file_size = fs::file_size(path);
                if (file_size % img_size != 0)
                {
                    std::string msg = "File " + normalized_path + " size is not divisible by image size.";
                    Logger::instance().log(LogLevel::Warn, msg);
                    // continue;
                }

                int frame_counts = static_cast<int>(file_size / img_size);
                SystemParams::instance().frame_count_.emplace_back(frame_counts);

                file_count++;
                total_size += file_size;
                total_frame_counts += frame_counts;

                std::string msg = "[BIL FILE] " + path.filename().string() + " | Size: " + std::to_string(file_size) + " bytes | Frames: " + std::to_string(frame_counts);
                Logger::instance().log(LogLevel::Info, msg);
            }
        }

        SystemParams::instance().total_valid_files_ = file_count;
        GuiDispParams::instance().reconstruction_progress_bar_right_valid_files_num_ = file_count;
        Logger::instance().log(LogLevel::Info, "[BIL FILE] Total .bil files: " + std::to_string(file_count));
        Logger::instance().log(LogLevel::Info, "[BIL FILE] Total size (bytes): " + std::to_string(total_size));
        Logger::instance().log(LogLevel::Info, "[BIL FILE] Total frames: " + std::to_string(total_frame_counts));
    }
    catch (const std::exception &e)
    {
        std::string msg = std::string("Error in findBilData: ") + e.what();
        throw std::runtime_error(msg);
        Logger::instance().log(LogLevel::Error, msg);
    }
}

void ReconstructionTask::reconstruction()
{
    try
    {
        const int img_width = SystemParams::instance().img_width_;
        const int img_height = SystemParams::instance().img_height_;
        const size_t img_size = SystemParams::instance().img_size_;

        HObject ho_profile_image;
        HTuple hv_width, hv_height;
        this->matToHObject(src_mat_, &ho_profile_image);
        GetImageSize(ho_profile_image, &hv_width, &hv_height);

        HObject ho_profile_region;
        GenRectangle1(&ho_profile_region,
                      SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1,
                      SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1,
                      SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2,
                      SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2);

        GuiDispParams::instance().reconstruction_processed_file_num_ = 0;
        for (size_t file_idx = 0; file_idx < SystemParams::instance().valid_files_.size(); file_idx++)
        {
            try
            { // Local iconic variables
                HObject ho_image;

                // Local iconic variables
                HTuple hv_window_handle;
                HTuple hv_sheet_of_light_model_id, hv_object_model_3d_id;

                HTuple frame_nums;
                frame_nums[0] = SystemParams::instance().frame_count_[file_idx];

                std::ifstream bil_file(SystemParams::instance().valid_files_[file_idx], std::ios::binary);
                if (!bil_file.is_open())
                {
                    std::string msg = "Can't open " + SystemParams::instance().valid_files_[file_idx];
                    Logger::instance().log(LogLevel::Error, msg);
                    continue;
                }
                bil_file.seekg(0, std::ios::beg);

                CreateSheetOfLightModel(ho_profile_region,
                                        ((HTuple("min_gray").Append("num_profiles")).Append("ambiguity_solving")),
                                        (SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold.TupleConcat(frame_nums)).TupleConcat("first"),
                                        &hv_sheet_of_light_model_id);
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "calibration", "xyz");
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "scale", "mm");
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "camera_parameter", SystemParams::instance().hv_system_poses_.hv_camera_params);
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "camera_pose", SystemParams::instance().hv_system_poses_.hv_camera_pose);
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "lightplane_pose", SystemParams::instance().hv_system_poses_.hv_light_plane_poses);
                SetSheetOfLightParam(hv_sheet_of_light_model_id, "movement_pose", SystemParams::instance().hv_system_poses_.hv_movement_poses);

                HObject ho_final_image;
                HObject ho_image_part, ho_gray_image, ho_laser_image;
                CropRectangle1(ho_profile_image, &ho_image_part,
                               SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1,
                               SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1,
                               SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2,
                               SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2);

                GuiDispParams::instance().reconstruction_processed_file_num_++;
                GuiDispParams::instance().reconstruction_processed_frame_num_ = 0;
                GuiDispParams::instance().reconstruction_progress_bar_right_frame_num_ = SystemParams::instance().frame_count_[file_idx];
                for (size_t frame = 0; frame < SystemParams::instance().frame_count_[file_idx]; frame++)
                {
                    if (running_.load())
                    {
                        std::streampos offset = frame * img_size;
                        bil_file.seekg(offset, std::ios::beg);
                        std::vector<uint8_t> img_data(img_size);
                        bil_file.read(reinterpret_cast<char *>(img_data.data()), img_size);
                        if (bil_file.gcount() < img_size)
                        {
                            std::string msg = "Error reading frame " + std::to_string(frame) + " from " + SystemParams::instance().valid_files_[file_idx];
                            Logger::instance().log(LogLevel::Warn, msg);
                            break;
                        }
                        cv::Mat temp_img(img_height, img_width, CV_8UC1, img_data.data());
                        this->matToHObject(temp_img, &ho_image);
                        CropRectangle1(ho_image, &ho_image_part,
                                       SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1,
                                       SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1,
                                       SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2,
                                       SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2);
                        this->extractLaser(ho_image_part, &ho_laser_image, SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold);

                        // Putting the cropped ROI area back into the original image
                        this->resizeImage(ho_laser_image, hv_width, hv_height, &ho_final_image,
                                          SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1,
                                          SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1);
                        MeasureProfileSheetOfLight(ho_final_image, hv_sheet_of_light_model_id, HTuple());

                        GuiDispParams::instance().reconstruction_processed_frame_num_++;
                    }
                }

                bil_file.close();

                // Get the resulting images and close the sheet-of-light handle
                GetSheetOfLightResultObjectModel3d(hv_sheet_of_light_model_id, &hv_object_model_3d_id);

                // from wcs to camera cs
                HTuple hv_object_model_affine_trans, hv_pose_trans;
                hv_pose_trans = SystemParams::instance().hv_system_poses_.hv_camera_pose;

                // form m to mm
                hv_pose_trans[0] = HTuple(SystemParams::instance().hv_system_poses_.hv_camera_pose[0]) * 1000;
                hv_pose_trans[1] = HTuple(SystemParams::instance().hv_system_poses_.hv_camera_pose[1]) * 1000;
                hv_pose_trans[2] = HTuple(SystemParams::instance().hv_system_poses_.hv_camera_pose[2]) * 1000;
                RigidTransObjectModel3d(hv_object_model_3d_id, hv_pose_trans, &hv_object_model_affine_trans);

                // Save the model
                if (SystemParams::instance().save_cloud_flag_)
                {
                    std::string processed_time = this->getTime();
                    std::filesystem::path file_path = SystemParams::instance().valid_files_[file_idx];
                    HTuple save_path = SystemParams::instance().hv_program_params_.hv_output_cloud_root + "/" + file_path.stem().string().c_str() + "_" + processed_time.c_str();
                    WriteObjectModel3d(hv_object_model_affine_trans, "obj", save_path, HTuple(), HTuple());

                    std::string msg = "Save to " + std::string(save_path[0].S().Text());
                    Logger::instance().log(LogLevel::Info, msg);
                }

                if (!running_.load())
                {
                    return;
                }
            }
            catch (const HException &exception)
            {
                std::string msg = "Error in " + SystemParams::instance().valid_files_[file_idx] + " file: " + exception.ErrorMessage().Text();
                Logger::instance().log(LogLevel::Error, msg);
            }
            catch (const std::exception &e)
            {
                std::string msg = "Error in " + SystemParams::instance().valid_files_[file_idx] + " file: " + e.what();
                Logger::instance().log(LogLevel::Error, msg);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::string msg = std::string("Error in reconstruction: ") + e.what();
        Logger::instance().log(LogLevel::Error, msg);
    }
}

void ReconstructionTask::extractLaser(const HObject &ho_image, HObject *ho_laser_image, const HTuple &hv_min_threshold)
{
    // Local iconic variables
    HObject ho_gamma_image, ho_image_median, ho_region;
    HObject ho_image_1, ho_rectangle, ho_image_part_1, ho_region_1;
    HObject ho_bin_image, ho_region_2, ho_connected_regions;
    HObject ho_region_union, ho_bin_image1, ho_selected_regions;

    // Local control variables
    HTuple hv_width, hv_height, hv_sub_image_nums, hv_used_threshold;
    HTuple hv_width_1, hv_rows, hv_cols, hv_histo_percent, hv_index;
    HTuple hv_absolute_histo, hv_relative_histo, hv_s, hv_index_1;
    HTuple hv_thres, hv_grayvals, hv_cols_1, hv_window_handle;
    HTuple hv_region_area, hv_region_row, hv_region_col;

    // Get image size
    GetImageSize(ho_image, &hv_width, &hv_height);

    // Gamma, median filter
    GammaImage(ho_image, &ho_gamma_image, 1.316667, 0, 0.41308, 255, "true");
    MedianImage(ho_gamma_image, &ho_image_median, "circle", 3, "mirrored");

    // Region required to generate a grayscale histogram
    Threshold(ho_image_median, &ho_region, 0, 255);

    // Number of image slices
    hv_sub_image_nums = 10;
    hv_width_1 = hv_width / hv_sub_image_nums;

    // Create a blank image of the corresponding size
    GenImageConst(&ho_image_1, "byte", hv_width, hv_height);

    // Create a rectangular area and get all the coordinates in the area.
    GenRectangle1(&ho_rectangle, 0, 0, hv_height - 1, hv_width_1 - 1);
    GetRegionPoints(ho_rectangle, &hv_rows, &hv_cols);

    // Setting the percentage threshold for the first graph
    HTuple end_val15 = hv_sub_image_nums - 1;
    HTuple step_val15 = 1;
    for (hv_index = 0; hv_index.Continue(end_val15, step_val15); hv_index += step_val15)
    {
        // Slicing the image
        CropPart(ho_image_median, &ho_image_part_1, 0,
                 hv_index * hv_width_1, hv_width_1, hv_height);
        Threshold(ho_image_part_1, &ho_region_1, hv_min_threshold, 255);

        // Fill the slice to the corresponding area of the blank map
        RegionToBin(ho_region_1, &ho_bin_image, 255, 0, hv_width_1, hv_height);
        GetGrayval(ho_bin_image, hv_rows, hv_cols, &hv_grayvals);
        hv_cols_1 = hv_cols + (hv_index * hv_width_1);
        SetGrayval(ho_image_1, hv_rows, hv_cols_1, hv_grayvals);
    }

    // Binarization, Connected component selection
    Threshold(ho_image_1, &ho_region_2, 128, 255);
    AreaCenter(ho_region_2, &hv_region_area, &hv_region_row, &hv_region_col);
    if (hv_region_area <= 0)
    {
        std::string msg = "threshold is too large, no region is found.";
        throw std::runtime_error(msg);
    }
    Connection(ho_region_2, &ho_connected_regions);
    SelectShape(ho_connected_regions, &ho_selected_regions, "area", "and", 40, 99999);

    // Union, turn region to image
    Union1(ho_selected_regions, &ho_region_union);
    AreaCenter(ho_region_union, &hv_region_area, &hv_region_row, &hv_region_col);
    if (hv_region_area <= 0)
    {
        std::string msg = "connected region area threshold error, no region is found.";
        throw std::runtime_error(msg);
    }
    RegionToBin(ho_region_union, ho_laser_image, 255, 0, hv_width, hv_height);
}

void ReconstructionTask::resizeImage(const HObject &ho_image_part, const HTuple &hv_width,
                                     const HTuple &hv_height, HObject *ho_resized_image,
                                     const HTuple &hv_roi_row1, const HTuple &hv_roi_column1)
{
    HObject ho_image_1, ho_rectangle;
    HTuple hv_width_1, hv_height_1, hv_rows, hv_cols, hv_grayvals;
    GenImageConst(&ho_image_1, "byte", hv_width, hv_height);
    GetImageSize(ho_image_part, &hv_width_1, &hv_height_1);
    GenRectangle1(&ho_rectangle, 0, 0, hv_height_1 - 1, hv_width_1 - 1);
    GetRegionPoints(ho_rectangle, &hv_rows, &hv_cols);
    GetGrayval(ho_image_part, hv_rows, hv_cols, &hv_grayvals);

    // Add offsets to all pixel coordinates
    hv_rows += hv_roi_row1;
    hv_cols += hv_roi_column1;

    // Place the figure in the specified area of a blank image
    SetGrayval(ho_image_1, hv_rows, hv_cols, hv_grayvals);
    CopyImage(ho_image_1, ho_resized_image);
}

std::string ReconstructionTask::getTime()
{
#ifdef _WIN32
    // Get the current time
    auto now_time = std::chrono::system_clock::now();

    // Convert to local time
    auto local_time = std::chrono::system_clock::to_time_t(now_time);

    // Format the timestamp as a string
    std::stringstream string_time;
    std::tm struct_time{};
    localtime_s(&struct_time, &local_time);
    string_time << std::put_time(&struct_time, "%Y%m%d%H%M%S");
    return string_time.str();
#elif __linux__
    std::time_t result = std::time(nullptr);
    struct std::tm *struct_tm = std::localtime(&result);
    std::string time_stamp;
    std::stringstream ss;
    ss << (struct_tm->tm_year + 1900);
    ss << std::setw(2) << std::setfill('0') << (struct_tm->tm_mon + 1);
    ss << std::setw(2) << std::setfill('0') << (struct_tm->tm_mday);
    ss << std::setw(2) << std::setfill('0') << (struct_tm->tm_hour);
    ss << std::setw(2) << std::setfill('0') << (struct_tm->tm_min);
    ss << std::setw(2) << std::setfill('0') << (struct_tm->tm_sec);
    ss >> time_stamp;
    return time_stamp;
#endif
}

void ReconstructionTask::preprocess()
{
    std::string first_bil_file = SystemParams::instance().valid_files_[0];
    std::ifstream f(first_bil_file, std::ios::binary);
    if (!f.is_open())
    {
        throw std::runtime_error("can't open .bil file");
    }

    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> first_frame_data(SystemParams::instance().img_width_ * SystemParams::instance().img_width_);
    f.read(reinterpret_cast<char *>(first_frame_data.data()), first_frame_data.size());
    f.seekg(0, std::ios::beg);
    f.close();

    const cv::Mat first_frame(800, 1280, CV_8UC1, first_frame_data.data());
    if (!first_frame.data)
    {
        throw std::runtime_error("error opening image!");
    }
    first_frame.copyTo(src_mat_);

    if (SystemParams::instance().set_roi_flag_ && !SystemParams::instance().select_entire_frame_)
    {
        setRoi(first_frame);
    }
    else
    {
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1 = 0;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1 = 0;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2 = SystemParams::instance().img_height_ - 1;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2 = SystemParams::instance().img_width_ - 1;

        GuiDispParams::instance().reconstruction_roi_row1_disp_ = 0;
        GuiDispParams::instance().reconstruction_roi_col1_disp_ = 0;
        GuiDispParams::instance().reconstruction_roi_row2_disp_ = SystemParams::instance().img_height_ - 1;
        GuiDispParams::instance().reconstruction_roi_col2_disp_ = SystemParams::instance().img_width_ - 1;
    }

    if (SystemParams::instance().set_threshold_flag_)
    {
        setThreshold(first_frame);
    }
}

void ReconstructionTask::setRoi(const cv::Mat &src_mat)
{

    Logger::instance().log(LogLevel::Info, "Please select the ROI region");
    double max_size = 700;
    int window_width, window_height;
    double aspect_ratio = static_cast<double>(src_mat.cols) / static_cast<double>(src_mat.rows);

    if (src_mat.cols > src_mat.rows)
    {
#ifndef _WIN32
        window_width = std::min(src_mat.cols, static_cast<int>(max_size));
#else
        window_width = min(src_mat.cols, static_cast<int>(max_size));
#endif
        window_height = static_cast<int>(window_width / aspect_ratio);
    }
    else
    {
#ifndef _WIN32
        window_height = std::min(src_mat.rows, static_cast<int>(max_size));
#else
        window_height = min(src_mat.rows, static_cast<int>(max_size));
#endif
        window_width = static_cast<int>(window_height * aspect_ratio);
    }

    cv::namedWindow("Set Roi", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Set Roi", [](int event, int x, int y, int flags, void *userdata)
                         {
                             ReconstructionTask *task = static_cast<ReconstructionTask *>(userdata);

                             switch (event)
                             {
                             case cv::EVENT_MOUSEMOVE:
                                 if (task->drawing_box_)
                                 {
                                     task->box_.width = x - task->box_.x;
                                     task->box_.height = y - task->box_.y;
                                 }
                                 break;
                             case cv::EVENT_LBUTTONDOWN:
                                 task->drawing_box_ = true;
                                 task->box_ = cv::Rect(x, y, 0, 0);
                                 break;
                             case cv::EVENT_LBUTTONUP:
                                 task->drawing_box_ = false;
                                 if (task->box_.width < 0)
                                 {
                                     task->box_.x += task->box_.width;
                                     task->box_.width *= -1;
                                 }
                                 if (task->box_.height < 0)
                                 {
                                     task->box_.y += task->box_.height;
                                     task->box_.height *= -1;
                                 }
                                 break;
                             } }, this);

    cv::resizeWindow("Set Roi", window_width, window_height);

    cv::Mat temp;
    src_mat.copyTo(temp);

    bool roi_selected = false;

    while (true)
    {
        if (drawing_box_)
        {
            src_mat.copyTo(temp);
            cv::rectangle(temp, box_, cv::Scalar(0, 255, 0), 8);
        }

        try
        {
            cv::imshow("Set Roi", temp);
        }
        catch (const cv::Exception &e)
        {
            Logger::instance().log(LogLevel::Info, "Window closed or unavailable. ROI set to default.");
            roi_selected = false;
            break;
        }

        int key = cv::waitKey(15);

        if (key == 13)
        {
            roi_selected = true;
            break;
        }

        if (key == 27)
        {
            roi_selected = false;
            Logger::instance().log(LogLevel::Info, "ESC pressed. ROI set to default.");
            break;
        }
    }

    cv::destroyAllWindows();

    if (roi_selected)
    {
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1 = box_.y;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1 = box_.x;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2 = box_.y + box_.height;
        SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2 = box_.x + box_.width;

        GuiDispParams::instance().reconstruction_roi_row1_disp_ = box_.y;
        GuiDispParams::instance().reconstruction_roi_col1_disp_ = box_.x;
        GuiDispParams::instance().reconstruction_roi_row2_disp_ = box_.y + box_.height;
        GuiDispParams::instance().reconstruction_roi_col2_disp_ = box_.x + box_.width;
        return;
    }

    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row1 = 0;
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col1 = 0;
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_row2 = SystemParams::instance().img_height_ - 1;
    SystemParams::instance().hv_program_params_.hv_reconstruction_roi_col2 = SystemParams::instance().img_width_ - 1;

    GuiDispParams::instance().reconstruction_roi_row1_disp_ = 0;
    GuiDispParams::instance().reconstruction_roi_col1_disp_ = 0;
    GuiDispParams::instance().reconstruction_roi_row2_disp_ = SystemParams::instance().img_height_ - 1;
    GuiDispParams::instance().reconstruction_roi_col2_disp_ = SystemParams::instance().img_width_ - 1;
}

void ReconstructionTask::setThreshold(const cv::Mat &src_mat)
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

    temp_threshold_ = SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold[0].I();

    auto on_trackbar = [](int pos, void *userdata)
    {
        auto *task = static_cast<ReconstructionTask *>(userdata);
        task->temp_threshold_ = pos;
        cv::threshold(task->src_mat_, task->dst_mat_, pos, 255, cv::THRESH_BINARY);
        cv::imshow("Setting threshold", task->dst_mat_);
    };

    cv::createTrackbar("Threshold value", "Setting threshold", NULL, 255, on_trackbar, this);
    cv::setTrackbarPos("Threshold value", "Setting threshold", temp_threshold_);
    on_trackbar(temp_threshold_, this);

    while (1)
    {
        if (char(cv::waitKey(1)) == 13)
        {
            break;
        }
    }
    cv::destroyAllWindows();

    SystemParams::instance().hv_program_params_.hv_reconstruction_min_threshold = this->temp_threshold_;
    GuiDispParams::instance().reconstruction_min_threshold_disp_ = this->temp_threshold_;
}

void ReconstructionTask::matToHObject(const cv::Mat &src_mat, HalconCpp::HObject *dst_hobj)
{
    int hgt = src_mat.rows;
    int wid = src_mat.cols;
    //  CV_8UC3
    if (src_mat.type() == CV_8UC3)
    {
        std::vector<cv::Mat> img_channel;
        split(src_mat, img_channel);
        cv::Mat img_b = img_channel[0];
        cv::Mat img_g = img_channel[1];
        cv::Mat img_r = img_channel[2];
        try
        {
            auto *data_r = new uchar[hgt * wid];
            auto *data_g = new uchar[hgt * wid];
            auto *data_b = new uchar[hgt * wid];
            for (int i = 0; i < hgt; i++)
            {
                // step: number of pixels per row
                memcpy(data_r + wid * i, img_r.data + img_r.step * i, wid);
                memcpy(data_g + wid * i, img_g.data + img_g.step * i, wid);
                memcpy(data_b + wid * i, img_b.data + img_b.step * i, wid);
            }
            GenImage3(dst_hobj, "byte", wid, hgt, reinterpret_cast<Hlong>(data_r), reinterpret_cast<Hlong>(data_g),
                      reinterpret_cast<Hlong>(data_b));
            delete[] data_r;
            delete[] data_g;
            delete[] data_b;
            data_r = nullptr;
            data_g = nullptr;
            data_b = nullptr;
        }
        catch (std::bad_alloc)
        {
            std::cout << "bad_alloc, Mat to Hobj error!" << std::endl;
        }
    }
    //  CV_8UCU1
    else if (src_mat.type() == CV_8UC1)
    {
        try
        {
            auto *data = new uchar[hgt * wid];
            for (int i = 0; i < hgt; i++)
            {
                memcpy(data + wid * i, src_mat.data + src_mat.step * i, wid);
            }
            GenImage1(dst_hobj, "byte", wid, hgt, reinterpret_cast<Hlong>(data));
            delete[] data;
            data = nullptr;
        }
        catch (std::bad_alloc)
        {
            std::cout << "bad_alloc, Mat to Hobj error!" << std::endl;
        }
    }
}