# Solar System Graphic Simulation

## Explanation

This repo contains a cmake build which will create a small OpenGL based sim  
using C++ and bullet3. The sim shows a sun and planet rotating in black space  
with an automated camera which keeps the two in frame.

## System Requirements

To build the simulation, the following libraries are required to be installed  
locally on your machine:  
- glad
- glfw3
- glm
- assimp
- cmake
- C++14

Also, download a copy of bullet3 from github and place it in the src/ folder

## Build Instructions

Using a terminal, change directory to the build folder. Then type the following  
command: `cmake ../src`  
If that command results in an output:
`-- Build files have been written to: home/.../solar_system/build`  
Follow up by typing `cmake --build .`  
Once built, type `./solar_system` to run the executable created and see the simulation
