#include "task_panel.h"

#include "imgui.h"

TaskPanel::TaskPanel()
{
    reconstruction_page_ = std::make_unique<ReconstructionPage>();
}

void TaskPanel::render()
{
    ImGui::Begin("Task Panel");

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("Task Panel Tab Bar", tab_bar_flags))
    {
        if (ImGui::BeginTabItem("Reconstruction"))
        {
            reconstruction_page_->draw();
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Calibration"))
        {
            calibration_page_->draw();
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}