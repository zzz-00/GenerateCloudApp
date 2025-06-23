#include "params_init_task.h"

#include "system_params.h"

ParamsInitTask::ParamsInitTask(int id) : Task(id)
{
}

void ParamsInitTask::run()
{
    SystemParams::instance();
    GuiDispParams::instance();
    GuiDispParams::instance().disabled_ = false;
}