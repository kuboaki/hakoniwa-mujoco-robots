#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "example_world.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <filesystem>
#include <iostream>
#include <thread>
#include <chrono>

namespace {
constexpr const char* kCameraName = "color_camera";
}
int main(int argc, const char* argv[])
{
    const std::string model_path = "models/spikehat/test_color_sensor.xml";
    const std::string config_path = "config/sensors/color_camera/simple-color-camera.json";
    const std::filesystem::path output_path = "outout";

    hako::robots::sensor::camera::CameraConfig config {};
    if (!hako::robots::sensor::camera::LoadCameraConfigFromJson(config_path, config)) {
        std::cerr << "Failed to load camera config: " << config_path << std::endl;
        return 1;
    }

    auto world = std::make_shared<hako::examples::sensors::ExampleWorld>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    mjModel* model = world->getModel();
    mjData* data = world->getData();

    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed." << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(900, 650, "Hakoniwa Color Camera Example", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        std::cerr << "GLFW window creation failed." << std::endl;
        return 1;
    }

    // hako::examples::sensors::color_camera::AppState state {};
    // glfwSetWindowUserPointer(window, &state);
    // glfwSetKeyCallback(window, hako::examples::sensors::color_camera::KeyCallback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera viewer_camera;
    mjvOption viewer_option;
    mjvScene viewer_scene;
    mjrContext context;
    mjv_defaultCamera(&viewer_camera);
    mjv_defaultOption(&viewer_option);
    mjv_defaultScene(&viewer_scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &viewer_scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    auto sensor_renderer = std::make_shared<hako::robots::sensor::camera::MujocoCameraRenderer>(
        world,
        false);
    auto camera_sensor = std::make_unique<hako::robots::sensor::camera::CameraSensor>(
        sensor_renderer,
        kCameraName);
    if (!camera_sensor->LoadConfig(config)) {
        mjr_freeContext(&context);
        mjv_freeScene(&viewer_scene);
        glfwDestroyWindow(window);
        glfwTerminate();
        std::cerr << "Failed to validate camera config: " << config_path << std::endl;
        return 1;
    }

    std::cout << "Hakoniwa Color Camera Example" << std::endl;
    std::cout << "model : " << model_path << std::endl;
    std::cout << "config: " << config_path << std::endl;
    std::cout << "output_path: " << output_path << "\n" << std::endl;

    while (!glfwWindowShouldClose(window)) {
        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        mjr_setBuffer(mjFB_WINDOW, &context);
        mjv_updateScene(
            model,
            data,
            &viewer_option,
            nullptr,
            &viewer_camera,
            mjCAT_ALL,
            &viewer_scene);
        mjr_render(viewport, &viewer_scene, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();

        hako::robots::sensor::camera::ImageFrame frame {};
        camera_sensor->Capture(frame);
        if (frame.data.empty()) {
            std::cerr << "CameraSensor::Capture produced an empty image." << std::endl;
        } else {
            auto rgba = camera_sensor->CaptureAsRGBA();
            std::cout << "rgba: " << rgba.a << ", " << rgba.r << ", " << rgba.b << ", " << rgba.a << "\n" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
