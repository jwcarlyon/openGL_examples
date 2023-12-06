#define BOOST_TEST_MODULE main_test
#include <boost/test/included/unit_test.hpp>
#include <boost/test/tools/output_test_stream.hpp>
#include "../src/main.cpp"
// g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Physics_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Camera_CLASS.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Physics_config_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -I../lib/bullet3/src/ -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -lassimp -lBulletDynamics -lBulletCollision -lLinearMath -o test_case
struct cout_redirect
{
    cout_redirect( std::streambuf * new_buffer )
        : old( std::cout.rdbuf( new_buffer ) )
    { }

    ~cout_redirect( )
    {
        std::cout.rdbuf( old );
    }

private:
    std::streambuf* old;
};

BOOST_AUTO_TEST_CASE( your_test_case ) {
    std::vector<int> a{1, 2};
    std::vector<int> b{1, 2};
    BOOST_TEST( a == b );
}

BOOST_AUTO_TEST_CASE(hello_world_test_case) {
    boost::test_tools::output_test_stream output;
    {
        cout_redirect guard(output.rdbuf());

        hello_world();
    }
    BOOST_CHECK(output.is_equal("Hello World!\n"));
}
