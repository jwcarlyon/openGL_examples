#ifndef OPENGL_CONFIG_CLASS_HPP_INCLUDED
#define OPENGL_CONFIG_CLASS_HPP_INCLUDED

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <cstdio>
#include <iostream>

#include "Camera_CLASS.hpp"
#include "Model_CLASS.hpp"
#include "ThreadPool.h"
#include "TrackLayout.h"
#include "VehicleInterface.h"
#include "WireframeCollector.h"
// #include "Physics_config_CLASS.hpp"



class OpenGL_config
{
public:
    OpenGL_config();
  	~OpenGL_config();

    const unsigned int SCR_WIDTH = 1920;// * 2 / 3;//1100;
    const unsigned int SCR_HEIGHT = 1080;// * 2 / 3;//825;
    int* screenCaptureBuffer = new int[ SCR_WIDTH * SCR_HEIGHT ];
    glm::vec3 vehicleAxis_vec3;
    glm::vec3 vehiclePosition_vec3;
    float vehicleAngle_fl;

    void cameraChaseVehicle();
    void destroy();
    void drawLines(
      const float* linePointArray_floatPtr,
      const int linePointArraySize_int,
      const float* colorArray_floatPtr
    );
    Camera getCamera();
    GLFWwindow* getWindow();
    glm::mat4* getGroundMatrix_ptr();
    glm::mat4* getLeftFrontMatrix_ptr();
    glm::mat4* getLeftRearMatrix_ptr();
    glm::mat4* getRightFrontMatrix_ptr();
    glm::mat4* getRightRearMatrix_ptr();
    glm::mat4* getVehicleMatrix_ptr();
    void groundShaderUpdate(Shader& objectShader_ref, Model& groundModel_ref);
    void lightBulbShaderUpdate(Shader& objectShader_ref, Model& lightBulbModel_ref);
    void processInput(GLFWwindow* window);
    bool shouldWindowClose();
    void updateWindow(VehicleInterface& vehiclePosition_ref);
    void lowerCircleShaderUpdate(Shader& objectShader_ref, Model& trackModel_ref);
    void setLowerCircleMatrices(std::vector<glm::mat4>& matrix_vector);
    void setStartingCircleMatrices(std::vector<glm::mat4>& matrix_vector);
    void startingCircleShaderUpdate(Shader& objectShader_ref, Model& trackModel_ref);
    void updateMatrixArrays(
      ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
      ThreadSafe_matrix4& leftRearWheelPosition_mat4,
      TrackLayout& lowerCircle_trackLayout,
      ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
      ThreadSafe_matrix4& rightRearWheelPosition_mat4,
      TrackLayout& startingCircle_trackLayout,
      ThreadSafe_matrix4& vehiclePosition_mat4
    );
    void updatePlaneMatrixArrays(
      ThreadSafe_matrix4& groundPosition_mat4,
      ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
      ThreadSafe_matrix4& leftRearWheelPosition_mat4,
      ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
      ThreadSafe_matrix4& rightRearWheelPosition_mat4,
      ThreadSafe_matrix4& vehiclePosition_mat4
    );
    void skyboxShaderUpdate(SkyboxShader& skyboxShader_ref);
    void vehicleShaderUpdate(
      Shader& vehicleShader_ref,
      Model& vehicleModel_ref
    );
    void wheelShaderUpdate(
      Shader& wheelShader_ref,
      Model& wheelModel_ref
    );
    void screenCapture();
private:
    GLFWwindow* window;
    Camera camera;
    FILE* ffmpeg;

    LineShader* lineShader;
    std::vector<Line> wireframeLineCacheVector;
    std::vector<glm::mat4> lowerCircle_mat4Vector;
    std::vector<glm::mat4>  startingCircle_mat4Vector;
    glm::vec3 cameraPosition_vec3, cameraUp_vec3, centerPos_vec3, chassisWorldSpacePositionOffset_vec3,
      groundPosition_vec3, lightColor_vec3, lightSource_vec3;
    glm::mat4 ground_m4, light_m4, model_m4, projection_m4, vehiclePosition_m4, view_m4,
      leftFront_m4, rightFront_m4, leftRear_m4, rightRear_m4;
    glm::mat4* groundMatrix_ptr, *leftFrontMatrix_ptr,
      *rightFrontMatrix_ptr, *leftRearMatrix_ptr, *rightRearMatrix_ptr, *vehicleMatrix_ptr;

    void setup();
};
//---------------------------------------------------------


#endif // OPENGL_CONFIG_CLASS_HPP_INCLUDED
