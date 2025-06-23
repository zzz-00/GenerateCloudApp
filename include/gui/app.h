#pragma once

#include <string>
#include <memory>

#include "imgui.h"
#include "main_window.h"

struct GLFWwindow;

class App
{
public:
    App(int window_width = 1280, int window_height = 720, std::string title = "GenerateCloud");
    ~App();

    void run();

private:
    void init();
    void cleanup();
    void render();

    GLFWwindow *window_ = nullptr;
    std::unique_ptr<MainWindow> main_window_;

    ImGuiIO *io_ = nullptr;

    int window_width_;
    int window_height_;
    std::string title_;
    ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
};
