#include "log_panel.h"

#include "imgui.h"

LogPanel::LogPanel()
{
    logs_.reserve(1000);
}

void LogPanel::onLogReceived(const LogMessage &msg)
{
    logs_.push_back(msg);
    if (logs_.size() > 1000)
    {
        logs_.erase(logs_.begin());
    }
}

void LogPanel::render()
{
    ImGui::Begin("Log Panel");

    ImVec2 logWindowSize = ImVec2(0, 0);
    ImGui::BeginChild("LogScrollRegion", logWindowSize, false, ImGuiWindowFlags_AlwaysVerticalScrollbar);

    static size_t lastLogCount = 0;
    size_t currentLogCount = logs_.size();
    bool scrollToBottom = currentLogCount > lastLogCount;
    lastLogCount = currentLogCount;

    for (const auto &msg : logs_)
    {
        ImVec4 color;
        switch (msg.level)
        {
        case LogLevel::Info:
            color = ImVec4(1, 1, 1, 1);
            break;
        case LogLevel::Warn:
            color = ImVec4(1, 1, 0, 1);
            break;
        case LogLevel::Error:
            color = ImVec4(1, 0, 0, 1);
            break;
        default:
            color = ImVec4(0.5, 0.5, 0.5, 1);
            break;
        }

        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", msg.message.c_str());
        ImGui::PopStyleColor();
    }

    if (scrollToBottom)
    {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::End();
}
