#pragma once

#include <memory>

#include "imgui.h"
#include "log_panel.h"
#include "params_panel.h"
#include "task_panel.h"

class MainWindow
{
public:
    MainWindow();
    ~MainWindow();

    void render();

private:
    std::unique_ptr<LogPanel> log_panel_ = nullptr;
    std::unique_ptr<ParamsPanel> params_panel_ = nullptr;
    std::unique_ptr<TaskPanel> task_panel_ = nullptr;
};