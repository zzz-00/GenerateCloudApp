#include "params_panel.h"

#include "logger.h"
#include "system_params.h"
#include "disabled_block.h"

void ParamsPanel::render()
{
    ImGui::Begin("Params Panel");

    try
    {
        ImGuiDisabledBlock disabled_block(GuiDispParams::instance().disabled_);
        ImGui::SeparatorText("Paths");
        {
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Config File");
            ImGui::SameLine(120);
            if (ImGui::InputText("##Config File", GuiDispParams::instance().config_file_path_disp_, IM_ARRAYSIZE(GuiDispParams::instance().config_file_path_disp_), ImGuiInputTextFlags_EnterReturnsTrue))
            {
                GuiDispParams::instance().setConfigFilePath();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Data Root");
            ImGui::SameLine(120);
            if (ImGui::InputText("##Data Root", GuiDispParams::instance().data_root_disp_, IM_ARRAYSIZE(GuiDispParams::instance().data_root_disp_), ImGuiInputTextFlags_EnterReturnsTrue))
            {
                GuiDispParams::instance().setDataRoot();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Caltab Descr");
            ImGui::SameLine(120);
            if (ImGui::InputText("##Caltab Description", GuiDispParams::instance().caltab_description_disp_, IM_ARRAYSIZE(GuiDispParams::instance().caltab_description_disp_), ImGuiInputTextFlags_EnterReturnsTrue))
            {
                GuiDispParams::instance().setCaltabDescription();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Save Root");
            ImGui::SameLine(120);
            if (ImGui::InputText("##Output Root", GuiDispParams::instance().output_cloud_root_disp_, IM_ARRAYSIZE(GuiDispParams::instance().output_cloud_root_disp_), ImGuiInputTextFlags_EnterReturnsTrue))
            {
                GuiDispParams::instance().setOutputCloudRoot();
            }
        }

        ImGui::SeparatorText("Calibration Params");
        {
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Images Num");
            ImGui::SameLine(120);
            ImGui::SliderInt("##Calibration Images Num", &GuiDispParams::instance().calibration_images_num_disp_, 1, 500, "%d", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified calibration images num to " + std::to_string(GuiDispParams::instance().calibration_images_num_disp_));
                GuiDispParams::instance().setCalibrationImagesNum();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Min Threshold");
            ImGui::SameLine(120);
            ImGui::SliderInt("##Calibration Min Threshold", &GuiDispParams::instance().calibration_min_threshold_disp_, 0, 255, "%d", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified calibration min threshold to " + std::to_string(GuiDispParams::instance().calibration_min_threshold_disp_));
                GuiDispParams::instance().setCalibrationMinThreshold();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Frames Num");
            ImGui::SameLine(120);
            ImGui::SliderInt("##Calibration Frames Num", &GuiDispParams::instance().calibration_frames_num_disp_, 1, 6000, "%d", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified calibration frames num to " + std::to_string(GuiDispParams::instance().calibration_frames_num_disp_));
                GuiDispParams::instance().setCalibrationFrameNum();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Thickness");
            ImGui::SameLine(120);
            ImGui::SliderFloat("##Calibration Thickness", &GuiDispParams::instance().calibration_thickness_disp_, 0, 1, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified calibration thickness to " + std::to_string(GuiDispParams::instance().calibration_thickness_disp_));
                GuiDispParams::instance().setCalibrationThickness();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Max Error");
            ImGui::SameLine(120);
            ImGui::SliderFloat("##Calibration Max Light Plane Error", &GuiDispParams::instance().calibration_max_light_plane_error_disp_, 0, 1, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified max light plane error to " + std::to_string(GuiDispParams::instance().calibration_max_light_plane_error_disp_));
                GuiDispParams::instance().setCalibrationMaxLightPlaneError();
            }
        }

        ImGui::SeparatorText("Reconstruction Params");
        {
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Min Threshold");
            ImGui::SameLine(120);
            ImGui::SliderInt("##Reconstruction Min Threshold", &GuiDispParams::instance().reconstruction_min_threshold_disp_, 0, 255, "%d", ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                Logger::instance().log(LogLevel::Info, "Modified reconstruction min threshold to " + std::to_string(GuiDispParams::instance().reconstruction_min_threshold_disp_));
                GuiDispParams::instance().setReconstructionMinThreshold();
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Roi Area");
            ImGui::SameLine(120);
            if (GuiDispParams::instance().select_entire_frame_)
            {
                ImGui::TextDisabled(std::string(std::to_string(GuiDispParams::instance().reconstruction_roi_row1_disp_) + ", " +
                                                std::to_string(GuiDispParams::instance().reconstruction_roi_col1_disp_) + ", " +
                                                std::to_string(GuiDispParams::instance().reconstruction_roi_row2_disp_) + ", " +
                                                std::to_string(GuiDispParams::instance().reconstruction_roi_col2_disp_))
                                        .c_str());
            }
            else
            {
                ImGui::Text(std::string(std::to_string(GuiDispParams::instance().reconstruction_roi_row1_disp_) + ", " +
                                        std::to_string(GuiDispParams::instance().reconstruction_roi_col1_disp_) + ", " +
                                        std::to_string(GuiDispParams::instance().reconstruction_roi_row2_disp_) + ", " +
                                        std::to_string(GuiDispParams::instance().reconstruction_roi_col2_disp_))
                                .c_str());
            }
        }

        ImGui::SeparatorText("Mode Settings");
        {
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Save Cloud");
            ImGui::SameLine(120);
            if (ImGui::Checkbox("##Save Cloud", &GuiDispParams::instance().save_cloud_flag_))
            {
                GuiDispParams::instance().setSaveCloudFlag();
            }

            {
                ImGuiDisabledBlock disabled_setting_roi(GuiDispParams::instance().select_entire_frame_);
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Setting Roi");
                ImGui::SameLine(120);
                if (ImGui::Checkbox("##Setting Roi", &GuiDispParams::instance().set_roi_flag_))
                {
                    GuiDispParams::instance().setSetRoiFlag();
                }
            }

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Entire Frame");
            ImGui::SameLine(120);
            if (ImGui::Checkbox("##Entire Frame", &GuiDispParams::instance().select_entire_frame_))
            {
                GuiDispParams::instance().setSelectEntireFrame();
            }
        }
    }
    catch (const std::exception &e)
    {
        std::string msg = "[Setting Params] " + std::string(e.what());
        Logger::instance().log(LogLevel::Error, msg);
    }

    ImGui::End();
}