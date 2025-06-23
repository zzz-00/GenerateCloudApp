#include "calibration_page.h"

#include "imgui.h"

#include "task_manager_impl.h"
#include "calibration_task.h"

void CalibrationPage::draw()
{
    char images_num_buf[32] = {'\0'};
    static float images_num_progress = 0.0f;

    if (GuiDispParams::instance().calibration_progress_bar_right_images_num_ == 0)
    {
        sprintf(images_num_buf, "NaN");
    }
    else
    {
        images_num_progress = GuiDispParams::instance().calibration_processed_images_num_ * 1.0f / GuiDispParams::instance().calibration_progress_bar_right_images_num_;
        sprintf(images_num_buf, "%d/%d", GuiDispParams::instance().calibration_processed_images_num_, GuiDispParams::instance().calibration_progress_bar_right_images_num_);
    }

    ImGui::ProgressBar(images_num_progress, ImVec2(0.f, 0.f), images_num_buf);

    if (ImGui::Button("Calibration"))
    {
        auto calibration_task = new CalibrationTask(3);
        TaskManagerImpl::instance().startTask(calibration_task->getId(), calibration_task);
    }
}