#include <fstream>
#include <iostream>

#include "cmdline.h"
#include "nlohmann/json.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "glfw3.h"

#include "system_params.h"
#include "task_manager.h"
#include "task_manager_impl.h"

#include "app.h"

void runUiMode()
{
    std::cout << "[UI] Run in UI mode..." << std::endl;
    TaskManagerImpl::instance();

    App app;
    app.run();
}

void runCliMode(const cmdline::parser &parser)
{
    std::cout << "[CLI] Run in CLI mode..." << std::endl;
    std::string json_config_path = parser.get<std::string>("json-config");

    TaskManager &task_manager = TaskManagerImpl::instance();
}

int main(int argc, char *argv[])
{
    cmdline::parser parser;
    parser.add("ui", 'u', "Run in UI mode");
    parser.add("cli", 'c', "Run in CLI mode");
    parser.add<std::string>("json-config", 'j', "Config path", false, "config.json");

    parser.parse_check(argc, argv);

    if (parser.exist("cli"))
    {
        runCliMode(parser);
    }
    else
    {
        runUiMode();
    }

    return 0;
}