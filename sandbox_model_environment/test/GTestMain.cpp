#include <stdio.h>
#include <gtest/gtest.h>

GTEST_API_ int main(int argc, char **argv)
{
    printf("Running all tests in main() playground_model_environment\n");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
