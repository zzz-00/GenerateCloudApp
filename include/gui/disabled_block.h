#include "imgui.h"

class ImGuiDisabledBlock
{
public:
    ImGuiDisabledBlock(bool condition)
        : active_(condition)
    {
        if (active_)
        {
            ImGui::BeginDisabled();
        }
    }

    ~ImGuiDisabledBlock()
    {
        if (active_)
        {
            ImGui::EndDisabled();
        }
    }

private:
    bool active_;
};
