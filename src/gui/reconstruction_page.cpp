#include "reconstruction_page.h"

#include "imgui.h"

#include "task_manager_impl.h"
#include "reconstruction_task.h"

void ReconstructionPage::draw()
{
    char file_num_buf[32] = {'\0'};
    char frame_num_buf[32] = {'\0'};
    static float file_num_progress = 0.0f;
    static float frame_num_progress = 0.0f;

    if (GuiDispParams::instance().reconstruction_progress_bar_right_valid_files_num_ == 0)
    {
        sprintf(file_num_buf, "NaN");
    }
    else
    {
        file_num_progress = GuiDispParams::instance().reconstruction_processed_file_num_ * 1.0f / GuiDispParams::instance().reconstruction_progress_bar_right_valid_files_num_;
        sprintf(file_num_buf, "%d/%d", GuiDispParams::instance().reconstruction_processed_file_num_, GuiDispParams::instance().reconstruction_progress_bar_right_valid_files_num_);
    }

    if (GuiDispParams::instance().reconstruction_progress_bar_right_frame_num_ == 0)
    {
        sprintf(frame_num_buf, "NaN");
    }
    else
    {
        frame_num_progress = GuiDispParams::instance().reconstruction_processed_frame_num_ * 1.0f / GuiDispParams::instance().reconstruction_progress_bar_right_frame_num_;
        sprintf(frame_num_buf, "%d/%d", GuiDispParams::instance().reconstruction_processed_frame_num_, GuiDispParams::instance().reconstruction_progress_bar_right_frame_num_);
    }

    ImGui::ProgressBar(file_num_progress, ImVec2(0.f, 0.f), file_num_buf);
    ImGui::ProgressBar(frame_num_progress, ImVec2(0.f, 0.f), frame_num_buf);

    if (ImGui::Button("Reconstruction"))
    {
        auto reconstruction_task = new ReconstructionTask(2);
        TaskManagerImpl::instance().startTask(reconstruction_task->getId(), reconstruction_task);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        TaskManagerImpl::instance().stopTask(2);
    }
}