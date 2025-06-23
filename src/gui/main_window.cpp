#include "main_window.h"

#include "logger.h"

#include "log_panel.h"
#include "params_panel.h"
#include "task_panel.h"

MainWindow::MainWindow()
{
    log_panel_ = std::make_unique<LogPanel>();
    params_panel_ = std::make_unique<ParamsPanel>();
    task_panel_ = std::make_unique<TaskPanel>();

    Logger::instance().registerCallback(
        [this](const LogMessage &msg)
        {
            log_panel_->onLogReceived(msg);
        });

    Logger::instance().log(LogLevel::Info, "MainWindow initialized!");
}

MainWindow::~MainWindow() = default;

void MainWindow::render()
{
    log_panel_->render();
    params_panel_->render();
    task_panel_->render();
}
