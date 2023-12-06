#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/Camera_CLASS.hpp"
//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -o test_case
using namespace testing;

TEST(CameraTEST, WhenCreatedWithVectorsThenHasCorrectData)
{
    glm::vec3 expected_front_vec3;
    expected_front_vec3.x = cos(glm::radians(YAW)) * cos(glm::radians(PITCH));
    expected_front_vec3.y = sin(glm::radians(PITCH));
    expected_front_vec3.z = sin(glm::radians(YAW)) * cos(glm::radians(PITCH));
    expected_front_vec3 = glm::normalize(expected_front_vec3);

    Camera camera;

    EXPECT_EQ(camera.Front, expected_front_vec3);
    EXPECT_EQ(camera.Position, glm::vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(camera.WorldUp, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_EQ(camera.Pitch, PITCH);
    EXPECT_EQ(camera.MouseSensitivity, SENSITIVITY);
    EXPECT_EQ(camera.MovementSpeed, SPEED);
    EXPECT_EQ(camera.Yaw, YAW);
    EXPECT_EQ(camera.Zoom, ZOOM);
}
