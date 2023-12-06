#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/Shader_CLASS.hpp"
#include "../src/OpenGL_config_CLASS.hpp"

//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -o test_case

// using ::testing::Return;
// using ::testing::Throw;
using namespace testing;
namespace {
    class ShaderTest : public ::Test {

    protected:
        OpenGL_config* openGL_config;

        ShaderTest() {}

        virtual ~ShaderTest() {}

        virtual void SetUp()
        {
            openGL_config = new OpenGL_config();
        }

        virtual void TearDown()
        {
            delete openGL_config;
        }
    };

    TEST_F(ShaderTest, WhenShaderCreatedThenFragmentShaderGetsGLSLData)
    {
        Shader shader("mock_vertex_shader.vs", "mock_fragment_shader.fs");
        EXPECT_EQ(shader.getVertexShader(), "#version 330 core\nvoid main()\n{\n}\n");
        EXPECT_EQ(shader.getFragmentShader(), "#version 330 core\nvoid main()\n{\n}\n");
    }

    TEST_F(ShaderTest, WhenFragmentShaderCreatedAndFileNotFoundThenThrowsException)
    {
        const char* vertex_shader_filename = "mock_vertex_shader.vs";
        const char* invalid_filename = "nonexistent.txt";

        EXPECT_THROW(Shader shader(vertex_shader_filename, invalid_filename), std::ifstream::failure);
    }

    TEST_F(ShaderTest, WhenVertexShaderCreatedAndFileNotFoundThenThrowsException)
    {
        const char* fragment_shader_filename = "mock_fragment_shader.fs";
        const char* invalid_filename = "nonexistent.txt";

        EXPECT_THROW(Shader shader(invalid_filename, fragment_shader_filename), std::ifstream::failure);
    }
}
