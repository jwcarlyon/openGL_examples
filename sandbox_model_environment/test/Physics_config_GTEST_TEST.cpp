#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/OpenGL_config_CLASS.hpp"
#include "../src/Physics_config_CLASS.hpp"
#include "../src/Shader_CLASS.hpp"

//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Physics_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Camera_CLASS.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Physics_config_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -I../lib/bullet3/src/ -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -lassimp -lBulletDynamics -lBulletCollision -lLinearMath -o test_case
using namespace testing;

TEST(PhysicsConfigTEST, WhenCreatedThenHasCorrectData)
{
    OpenGL_config openGL_config;
    Shader mockShader("mock_vertex_shader.vs", "mock_fragment_shader.fs");
    MyGuiHelper guiHelper(&openGL_config, mockShader);

    Physics_config physics(&guiHelper);
    physics.initPhysics();

    EXPECT_EQ(physics.getDynamicsWorld()->getGravity().getY(), -10.f);
}
