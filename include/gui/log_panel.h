#pragma once

#include <vector>

#include "imgui.h"

#include "logger.h"

class LogPanel
{
public:
    LogPanel();

    void onLogReceived(const LogMessage &msg);

    void render();

private:
    std::vector<LogMessage> logs_;
};
