#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/OpenGL_config_CLASS.hpp"
#include "../src/Model_CLASS.hpp"

//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Camera_CLASS.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -lassimp -o test_case
using namespace testing;

TEST(ModelTEST, WhenCreatedThenHasCorrectData)
{
    OpenGL_config openGL_config;
    std::string const& testModelName = "bulb.obj";

    Model model(testModelName);

    EXPECT_EQ(model.getDirectory(), testModelName);
    openGL_config.destroy();
}
