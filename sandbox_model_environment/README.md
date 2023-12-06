# Solar System Graphic Simulation

## Explanation

This repo contains a cmake build which will create a small 3D simulation
using C++ and bullet3 with OpenGL commands. The sim shows a skybox and twin circle tracks with a functional vehicle the user can drive around.
The camera is static but moveable with the WASD keys until the user hits the 'home' key to enable the 'chase' camera which follows the vehicle. The Bullet3 implemetation and the OpenGL renderer operate in separate threads to avoid slowdown during computation-heavy events.

## System Requirements

To build the simulation, the following libraries are required to be installed  
locally on your machine:  
- glad
- glfw3
- glm
- assimp
- cmake
- C++14

Also, download a copy of bullet3 from github and place it in the lib/ folder. The binaries must be created to function, please check the readme in the Bullet folder.

## Build Instructions

Using a terminal, change directory to the build folder. Then type the following  
command: `cmake ../src`  
If that command results in an output:
`-- Build files have been written to: home/.../sandbox_model_environment/build`
Follow up by typing `cmake --build .`  
Once built, type `./Playground` to run the executable created and see the simulation
