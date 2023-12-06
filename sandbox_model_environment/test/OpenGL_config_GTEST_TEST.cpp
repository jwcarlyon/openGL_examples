#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/OpenGL_config_CLASS.hpp"
//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -o test_case
using namespace testing;

TEST(OpenGLConfigTEST, When_setup_Then_a_glfw_window_is_created_with_correct_size)
{
    OpenGL_config openGL_config;

    int window_width, window_height;
    glfwGetWindowSize(openGL_config.getWindow(), &window_width, &window_height);
    EXPECT_EQ(window_width, 1280);
    EXPECT_EQ(window_height, 720);
    std::cout << "Actual window measurements (h/w) (" << window_height  << "/" << window_width << ")" << std::endl;
}
