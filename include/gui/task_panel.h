#pragma once

#include <memory>

#include "reconstruction_page.h"
#include "calibration_page.h"

class TaskPanel
{
public:
    TaskPanel();

    void render();

private:
    std::unique_ptr<ReconstructionPage> reconstruction_page_ = nullptr;
    std::unique_ptr<CalibrationPage> calibration_page_ = nullptr;
};
