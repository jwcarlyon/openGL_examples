cmake_minimum_required(VERSION 3.10)
project(PlayGround VERSION 1.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(PROJ_NAME PlayGround)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
include(ExternalProject)
include_directories(
                "../lib/bullet3/src/"
)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_executable(${PROJ_NAME}
              main.cpp
              glad.c
              Camera_CLASS.cpp
              Mesh_CLASS.cpp
              Model_CLASS.cpp
              OpenGL_config_CLASS.cpp
              Physics_config_CLASS.cpp
              Shader_CLASS.cpp
)
target_include_directories(${PROJ_NAME} PUBLIC
                           "${PROJECT_BINARY_DIR}"
)
target_link_libraries(${PROJ_NAME} PUBLIC
                      glfw3
                      dl
                      X11
                      assimp
                      pthread
                      BulletDynamics
                      BulletCollision
                      LinearMath
)
