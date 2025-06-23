#pragma once

#include "task.h"

class ParamsInitTask : public Task
{
public:
    ParamsInitTask(int id);

private:
    void run() override;
};
