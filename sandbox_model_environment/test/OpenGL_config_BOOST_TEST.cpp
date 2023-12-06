#define BOOST_TEST_MODULE opengl_config_test
#include <boost/test/included/unit_test.hpp>
#include "../src/OpenGL_config_CLASS.hpp"
// g++ -I /usr/local/lib/boost_1_81_0/ OpenGL_config_TEST.cpp ../src/OpenGL_config_CLASS.cpp ../src/glad.c -lglfw -ldl -o run_test_case

BOOST_AUTO_TEST_CASE(When_setup_Then_a_glfw_window_is_created_with_correct_size) {
    OpenGL_config openGL_config;

    int window_width, window_height;
    glfwGetWindowSize(openGL_config.get_window(), &window_width, &window_height);
    BOOST_CHECK(window_width == 1280);
    BOOST_CHECK(window_height == 720);
    std::cout << "Actual window measurements (h/w) (" << window_height  << "/" << window_width << ")";
}
