#define BOOST_TEST_MODULE shader_test
#include <boost/test/included/unit_test.hpp>

#include "../src/Shader_CLASS.hpp"

//g++ -I /usr/local/lib/boost_1_81_0/ Shader_BOOST_TEST.cpp ../src/Shader_CLASS.cpp glad.c -ldl -o run_test_case

BOOST_AUTO_TEST_CASE(When_Shader_is_created_Then_glsl_is_loaded_as_a_string) {
    const char* fragment_path_ptr = "mock_fragment_shader.fs";
    const char* vertex_path_ptr = "mock_vertex_shader.vs";

    Shader shader_one(fragment_path_ptr, vertex_path_ptr);

    int shaderIsSuccessfullyCompiled;
    BOOST_CHECK(shaderIsSuccessfullyCompiled != GL_TRUE);
    BOOST_CHECK(shader_one.getVertexShader() == "#version 330 core\nvoid main()\n{\n}\n");
}

BOOST_AUTO_TEST_CASE(WhenFragmentShaderCreatedAndFileNotFoundThenThrowsException)
{
    const char* vertex_shader_filename = "mock_vertex_shader.vs";
    const char* invalid_filename = "nonexistent.txt";

    BOOST_CHECK_THROW(Shader shader(vertex_shader_filename, invalid_filename), std::ifstream::failure);
}

BOOST_AUTO_TEST_CASE(WhenVertexShaderCreatedAndFileNotFoundThenThrowsException)
{
    const char* fragment_shader_filename = "mock_fragment_shader.fs";
    const char* invalid_filename = "nonexistent.txt";

    BOOST_CHECK_THROW(Shader shader(invalid_filename, fragment_shader_filename), std::ifstream::failure);
}
