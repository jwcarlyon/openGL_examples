#include <iostream>
#include <cmath>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>

#include "../lib/bullet3/src/btBulletDynamicsCommon.h"

#include "OpenGL_config_CLASS.hpp"
#define RECORD_SESSION 0

static bool brakePedal, isChaseCamera, gasPedal;
static int steeringWheel;

bool isFirstMouse;
float mouseLastX_fl, mouseLastY_fl, mousePitch_fl, mouseYaw_fl;
glm::vec3 cameraFront_vec3;

OpenGL_config::OpenGL_config()
{
    std::cout << "OpenGL_config_CLASS::constructor debug, hello world!" << std::endl;

    isFirstMouse = true;
    mouseLastX_fl = SCR_WIDTH / 2.0f;
    mouseLastY_fl = SCR_HEIGHT / 2.0f;
    mouseYaw_fl = -90.0f;
    mousePitch_fl = 0.0f;

    brakePedal = false;
    isChaseCamera = false;
    gasPedal = false;
    steeringWheel = 0;

    cameraPosition_vec3 = glm::vec3(0.0f, 10.0f, 24.0f);
    camera = Camera(cameraPosition_vec3);
    cameraUp_vec3 = glm::vec3(0.0f, 1.0f, 0.0f);
    centerPos_vec3 = glm::vec3(0.67f, 0.5f, -6.0f);
    chassisWorldSpacePositionOffset_vec3 = glm::vec3(0.f, 0.2f, -0.2f);
    groundPosition_vec3 = glm::vec3(0.67f, .5f, -6.0f);
    ground_m4 = glm::mat4(0.0f); //set by bullet physics
    lightColor_vec3 = glm::vec3(1.0f, 1.0f, 1.0f);
    lightSource_vec3 = glm::vec3(1.0f, 5.0f, -0.5f);

    lowerCircle_mat4Vector.push_back(glm::mat4(0.0f));
    lowerCircle_mat4Vector.push_back(glm::mat4(0.0f));
    lowerCircle_mat4Vector.push_back(glm::mat4(0.0f));
    lowerCircle_mat4Vector.push_back(glm::mat4(0.0f));
    startingCircle_mat4Vector.push_back(glm::mat4(0.0f));
    startingCircle_mat4Vector.push_back(glm::mat4(0.0f));
    startingCircle_mat4Vector.push_back(glm::mat4(0.0f));
    startingCircle_mat4Vector.push_back(glm::mat4(0.0f));
    groundMatrix_ptr = &ground_m4;
    leftFront_m4 = glm::mat4(0.0f); //set by bullet physics
    leftFrontMatrix_ptr = &leftFront_m4;
    rightFront_m4 = glm::mat4(0.0f); //set by bullet physics
    rightFrontMatrix_ptr = &rightFront_m4;
    leftRear_m4 = glm::mat4(0.0f); //set by bullet physics
    leftRearMatrix_ptr = &leftRear_m4;
    rightRear_m4 = glm::mat4(0.0f); //set by bullet physics
    rightRearMatrix_ptr = &rightRear_m4;
    vehiclePosition_m4 = glm::mat4(0.0f); //set by bullet physics
    vehicleMatrix_ptr = &vehiclePosition_m4;

    setup();
}

OpenGL_config::~OpenGL_config()
{
    destroy();
    pclose(ffmpeg);
}

void OpenGL_config::cameraChaseVehicle()
{
    float chaseDistance = -25.f;
    glm::vec4 chaseOffset_vec4(0.f, 5.f, chaseDistance, 0.f);
    glm::vec4 vehicle_vec4(vehiclePosition_m4[3]);
    //you can transform any vector to that coordinate space by multiplying it the matrix representing the positon - the vehicle coordinate space
    cameraPosition_vec3 = glm::vec3(vehicle_vec4 + (vehiclePosition_m4 * chaseOffset_vec4));
    view_m4 = glm::lookAt(
        cameraPosition_vec3,
        glm::vec3(vehicle_vec4),
        cameraUp_vec3
    );
}

void OpenGL_config::destroy()
{
    glfwTerminate();
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

glm::mat4* OpenGL_config::getGroundMatrix_ptr()
{
  return groundMatrix_ptr;
}

glm::mat4* OpenGL_config::getLeftFrontMatrix_ptr()
{
    return leftFrontMatrix_ptr;
}

glm::mat4* OpenGL_config::getLeftRearMatrix_ptr()
{
    return leftRearMatrix_ptr;
}

glm::mat4* OpenGL_config::getRightFrontMatrix_ptr()
{
    return rightFrontMatrix_ptr;
}

glm::mat4* OpenGL_config::getRightRearMatrix_ptr()
{
    return rightRearMatrix_ptr;
}

glm::mat4* OpenGL_config::getVehicleMatrix_ptr()
{
    return vehicleMatrix_ptr;
}

Camera OpenGL_config::getCamera()
{ return camera; }

GLFWwindow* OpenGL_config::getWindow()
{
    return window;
}

void OpenGL_config::groundShaderUpdate(Shader& objectShader_ref, Model& groundModel_ref)
{
    model_m4 = glm::translate(ground_m4, groundPosition_vec3);//this is a
    // model_m4 = glm::rotate(model_m4, float(M_PI / -20.f), glm::vec3(0.f, 0.f, 1.f));
    // model_m4 = glm::scale(ground_m4, glm::vec3(10.f)); //both the model and the btTriangleMesh get scaled this way
    // model_m4 = glm::scale(ground_m4, glm::vec3(3.f));
    // model_m4 = glm::rotate(model_m4, float(M_PI), glm::vec3(0.f, 0.f, 1.f));

    objectShader_ref.use();
    objectShader_ref.setVec3("lightColor", lightColor_vec3);
    objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
    objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
    objectShader_ref.setMat4("model", model_m4);//model position
    objectShader_ref.setMat4("projection", projection_m4);
    objectShader_ref.setMat4("view", view_m4);

    groundModel_ref.draw(objectShader_ref);
}

void OpenGL_config::lightBulbShaderUpdate(Shader& objectShader_ref, Model& lightBulbModel_ref)

{
    objectShader_ref.use();
    model_m4 = glm::translate(glm::mat4(1.0f), centerPos_vec3);
    model_m4 = glm::translate(model_m4, lightSource_vec3);
    model_m4 = glm::scale(model_m4, glm::vec3(0.00025f));// it's a bit too big for our scene, so scale it down

    objectShader_ref.setMat4("projection", projection_m4);
    objectShader_ref.setMat4("view", view_m4);
    objectShader_ref.setMat4("model", model_m4);

    lightBulbModel_ref.draw(objectShader_ref);

}

void OpenGL_config::lowerCircleShaderUpdate(
  Shader& objectShader_ref,
  Model& trackModel_ref
) {
        glm::vec3 meshScalingVector_vec3(10.f);
        {
            glm::mat4 trackModel_m4 = lowerCircle_mat4Vector.at(0);
            model_m4 = glm::scale(
              trackModel_m4,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel_m4 = lowerCircle_mat4Vector.at(1);
            model_m4 = glm::scale(
              trackModel_m4,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel_m4 = lowerCircle_mat4Vector.at(2);
            model_m4 = glm::scale(
              trackModel_m4,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel_m4 = lowerCircle_mat4Vector.at(3);
            model_m4 = glm::scale(
              trackModel_m4,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
}

void OpenGL_config::processInput(GLFWwindow* window)//window is a member, refactor method
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
        glfwSetWindowShouldClose(window, true);
    }
    if(glfwGetKey(window, GLFW_KEY_HOME) == GLFW_PRESS){
        std::cout << "OpenGL_CLASS - processInput (toggle-chase down)" << std::endl;
        isChaseCamera = !isChaseCamera;
    }
    if(!isChaseCamera)
    {
        float camera_speed_fl = 0.5f;
        if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
            cameraPosition_vec3 += camera_speed_fl * cameraFront_vec3;
    		} else if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
            cameraPosition_vec3 -= camera_speed_fl * cameraFront_vec3;
        } else if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
            cameraPosition_vec3 -= glm::normalize(glm::cross(cameraFront_vec3, cameraUp_vec3)) * camera_speed_fl;
        } else if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
            cameraPosition_vec3 += glm::normalize(glm::cross(cameraFront_vec3, cameraUp_vec3)) * camera_speed_fl;
    		}
    }
    //---------------------------------------------------------
    if(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){
      std::cout << "OpenGL_CLASS - processInput (gas down)" << std::endl;
        gasPedal = true;//gas
		} else if(gasPedal && glfwGetKey(window, GLFW_KEY_UP) == GLFW_RELEASE){
      std::cout << "OpenGL_CLASS - processInput (gas up)" << std::endl;
        gasPedal = false;//coast
    }
    if(glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS){
        std::cout << "OpenGL_CLASS - processInput (brake down)" << std::endl;
        brakePedal = true;//brake
    } else if(brakePedal && glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_RELEASE){
        std::cout << "OpenGL_CLASS - processInput (brake up)" << std::endl;
        brakePedal = false;//coast
    }
    if(glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS){
        std::cout << "OpenGL_CLASS - processInput (left down)" << std::endl;
        steeringWheel = 1;
    } else if((steeringWheel > 0) && glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_RELEASE){
        std::cout << "OpenGL_CLASS - processInput (left up)" << std::endl;
        steeringWheel = 0;
    } else if(glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS){
        std::cout << "OpenGL_CLASS - processInput (right down)" << std::endl;
        steeringWheel = -1;
    } else if((steeringWheel < 0) && glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_RELEASE){
        std::cout << "OpenGL_CLASS - processInput (right up)" << std::endl;
        steeringWheel = 0;
		}
}

void processMousePositionCallback(GLFWwindow* window, double mouseXPosition, double mouseYPosition)
{
	  if(isFirstMouse)
	  {
		    mouseLastX_fl = mouseXPosition;
		    mouseLastY_fl = mouseYPosition;
		    isFirstMouse = false;
	  }
	  float mouseXOffset = mouseXPosition - mouseLastX_fl;
	  float mouseYOffset = mouseLastY_fl - mouseYPosition; // reversed since y-coordinates go from bottom to top
	  mouseLastX_fl = mouseXPosition;
	  mouseLastY_fl = mouseYPosition;

		// make sure that when pitch is out of bounds, screen doesn't get flipped
		mouseYaw_fl += mouseXOffset;
    float newMousePitch_fl = mousePitch_fl + mouseYOffset;
		if(newMousePitch_fl > 89.0f) {
        mousePitch_fl = 89.0f;
	  } else if(newMousePitch_fl < -89.0f) {
	      mousePitch_fl = -89.0f;
		} else {
        mousePitch_fl = newMousePitch_fl;
    }
	  glm::vec3 front_vec3(
			  cos(glm::radians(mouseYaw_fl)) * cos(glm::radians(mousePitch_fl)),
			  sin(glm::radians(mousePitch_fl)),
			  sin(glm::radians(mouseYaw_fl)) * cos(glm::radians(mousePitch_fl))
		);
	  cameraFront_vec3 = glm::normalize(front_vec3);
}

void OpenGL_config::screenCapture()
{
    glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, screenCaptureBuffer);
    fwrite(screenCaptureBuffer, sizeof(int)*SCR_WIDTH*SCR_HEIGHT, 1, ffmpeg);
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    // camera.ProcessMouseScroll(yoffset);
}

void OpenGL_config::setup()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Playground", 0, 0);
    if(window == 0)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window); /*Is this line necessary or not?*/
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetCursorPosCallback(window, processMousePositionCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {   std::cout << "Failed to initialize GLAD" << std::endl; return; }
    glEnable(GL_DEPTH_TEST);

    const char* bashScreenCaptureCommand = "ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s 1920x1080 -i - "
                  "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip output.mp4";
    ffmpeg = popen(bashScreenCaptureCommand, "w");
}

void OpenGL_config::setLowerCircleMatrices(std::vector<glm::mat4>& matrix_vector)
{
    int i = 0;
    for(auto matrix : matrix_vector)
    {
        lowerCircle_mat4Vector.at(i) = matrix;
        i++;
    }
}

void OpenGL_config::setStartingCircleMatrices(std::vector<glm::mat4>& matrix_vector)
{
    int i = 0;
    for(auto matrix : matrix_vector)
    {
        startingCircle_mat4Vector.at(i) = matrix;
        i++;
    }
}

bool OpenGL_config::shouldWindowClose() { return glfwWindowShouldClose(window); }

void OpenGL_config::skyboxShaderUpdate(SkyboxShader& skyboxShader_ref)
{
    glm::mat4 view_m3asM4 = glm::mat4(glm::mat3(view_m4));
    skyboxShader_ref.draw(projection_m4, view_m3asM4);
}


void OpenGL_config::startingCircleShaderUpdate(
  Shader& objectShader_ref,
  Model& trackModel_ref
) {
        glm::vec3 meshScalingVector_vec3(10.f);
        {
            glm::mat4 trackModel = startingCircle_mat4Vector.at(0);
            model_m4 = glm::scale(
              trackModel,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel = startingCircle_mat4Vector.at(1);
            model_m4 = glm::scale(
              trackModel,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel = startingCircle_mat4Vector.at(2);
            model_m4 = glm::scale(
              trackModel,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
        {
            glm::mat4 trackModel = startingCircle_mat4Vector.at(3);
            model_m4 = glm::scale(
              trackModel,
              meshScalingVector_vec3
            ); //both the model and the btTriangleMesh get scaled this way

            objectShader_ref.use();
            objectShader_ref.setVec3("lightColor", lightColor_vec3);
            objectShader_ref.setVec3("lightPos", lightSource_vec3);//used to calculate viewspace for reflection
            objectShader_ref.setVec3("viewerPos", cameraPosition_vec3);// camera.Position);
            objectShader_ref.setMat4("model", model_m4);//model position
            objectShader_ref.setMat4("projection", projection_m4);
            objectShader_ref.setMat4("view", view_m4);

            trackModel_ref.draw(objectShader_ref);
        }
}

void OpenGL_config::updatePlaneMatrixArrays(
    ThreadSafe_matrix4& groundPosition_mat4,
    ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
    ThreadSafe_matrix4& leftRearWheelPosition_mat4,
    ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
    ThreadSafe_matrix4& rightRearWheelPosition_mat4,
    ThreadSafe_matrix4& vehiclePosition_mat4
) {
    ground_m4 = groundPosition_mat4.get();
    vehiclePosition_m4 = vehiclePosition_mat4.get();
    leftFront_m4 = leftFrontWheelPosition_mat4.get();
    rightFront_m4 = rightFrontWheelPosition_mat4.get();
    leftRear_m4 = leftRearWheelPosition_mat4.get();
    rightRear_m4 = rightRearWheelPosition_mat4.get();
}

void OpenGL_config::updateMatrixArrays(
    ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
    ThreadSafe_matrix4& leftRearWheelPosition_mat4,
    TrackLayout& lowerCircle_trackLayout,
    ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
    ThreadSafe_matrix4& rightRearWheelPosition_mat4,
    TrackLayout& startingCircle_trackLayout,
    ThreadSafe_matrix4& vehiclePosition_mat4
) {
    lowerCircle_mat4Vector.at(0) = lowerCircle_trackLayout.turnOne.get();
    lowerCircle_mat4Vector.at(1) = lowerCircle_trackLayout.turnTwo.get();
    lowerCircle_mat4Vector.at(2) = lowerCircle_trackLayout.turnThree.get();
    lowerCircle_mat4Vector.at(3) = lowerCircle_trackLayout.turnFour.get();

    startingCircle_mat4Vector.at(0) = startingCircle_trackLayout.turnOne.get();
    startingCircle_mat4Vector.at(1) = startingCircle_trackLayout.turnTwo.get();
    startingCircle_mat4Vector.at(2) = startingCircle_trackLayout.turnThree.get();
    startingCircle_mat4Vector.at(3) = startingCircle_trackLayout.turnFour.get();

    vehiclePosition_m4 = vehiclePosition_mat4.get();
    leftFront_m4 = leftFrontWheelPosition_mat4.get();
    rightFront_m4 = rightFrontWheelPosition_mat4.get();
    leftRear_m4 = leftRearWheelPosition_mat4.get();
    rightRear_m4 = rightRearWheelPosition_mat4.get();
}

void OpenGL_config::updateWindow(VehicleInterface& vehicleInterface_ref)
{
    glfwSwapBuffers(window);
    std::cout << "OpenGL_config::updateWindow" << std::endl;
#if RECORD_SESSION
    screenCapture();
#endif
    glfwPollEvents();
    processInput(window);
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //update vehicle inputs
    vehicleInterface_ref.setBrakePedal(brakePedal);
    vehicleInterface_ref.setGasPedal(gasPedal);
    vehicleInterface_ref.setSteeringWheel(steeringWheel);

    projection_m4 = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

    if(isChaseCamera)
    {
        cameraChaseVehicle();
    } else {
        view_m4 = glm::lookAt(
            cameraPosition_vec3,
            cameraPosition_vec3 + cameraFront_vec3,
            cameraUp_vec3
        );
    }
}

void OpenGL_config::vehicleShaderUpdate(
  Shader& vehicleShader_ref,
  Model& vehicleModel_ref
) {
    glm::quat vehicleDynamicRotation = glm::quat_cast(vehiclePosition_m4);
    //This line reverses the pitch and yaw of the vehicle... but wtf makes it mirror like that?
    vehicleDynamicRotation = glm::quat(vehicleDynamicRotation.w, -vehicleDynamicRotation.x,  vehicleDynamicRotation.y, -vehicleDynamicRotation.z);
    glm::mat4 rotate = glm::mat4_cast(vehicleDynamicRotation);

    model_m4 =  glm::translate(glm::mat4(1.f), glm::vec3(vehiclePosition_m4[3]));
    model_m4 = model_m4 * rotate;
    model_m4 = glm::translate(model_m4, chassisWorldSpacePositionOffset_vec3);
    model_m4 = glm::scale(model_m4, glm::vec3(.1f));

    vehicleShader_ref.use();
    vehicleShader_ref.setVec3("lightColor", lightColor_vec3);
    vehicleShader_ref.setVec3("lightPos", lightSource_vec3);
    vehicleShader_ref.setVec3("viewerPos", cameraPosition_vec3);
    vehicleShader_ref.setMat4("model", model_m4);
    vehicleShader_ref.setMat4("projection", projection_m4);
    vehicleShader_ref.setMat4("view", view_m4);
    vehicleModel_ref.draw(vehicleShader_ref);
}

void OpenGL_config::wheelShaderUpdate(
  Shader& wheelShader_ref,
  Model& wheelModel_ref
) {
    wheelShader_ref.use();
    wheelShader_ref.setVec3("lightColor", lightColor_vec3);
    wheelShader_ref.setVec3("lightPos", lightSource_vec3);
    wheelShader_ref.setVec3("viewerPos", cameraPosition_vec3);
    wheelShader_ref.setMat4("projection", projection_m4);
    wheelShader_ref.setMat4("view", view_m4);

    float rotation_fl = M_PI / 2.f;
    glm::vec3 tireScale_vec3(0.2f, 0.4f, 0.4f);
    // glm::vec3 tireScale_vec3(0.2f, 0.4f, 0.4f);//frank_grimes_deluxe
    model_m4 = glm::scale(leftFront_m4, tireScale_vec3);
    model_m4 = glm::rotate(
          model_m4,
          rotation_fl,
          glm::vec3(0.0f, 0.0f, 1.0f)
    );
    wheelShader_ref.setMat4("model", model_m4);
    wheelModel_ref.draw(wheelShader_ref);

    model_m4 = glm::scale(rightFront_m4, tireScale_vec3);
    model_m4 = glm::rotate(
          model_m4,
          rotation_fl,
          glm::vec3(0.0f, 0.0f, -1.0f)
    );
    wheelShader_ref.setMat4("model", model_m4);
    wheelModel_ref.draw(wheelShader_ref);

    model_m4 = glm::scale(leftRear_m4, tireScale_vec3);
    model_m4 = glm::rotate(
      model_m4,
      rotation_fl,
      glm::vec3(0.0f, 0.0f, -1.0f)
    );
    wheelShader_ref.setMat4("model", model_m4);
    wheelModel_ref.draw(wheelShader_ref);

    model_m4 = glm::scale(rightRear_m4, tireScale_vec3);
    model_m4 = glm::rotate(
      model_m4,
      rotation_fl,
      glm::vec3(0.0f, 0.0f, 1.0f)
    );
    wheelShader_ref.setMat4("model", model_m4);
    wheelModel_ref.draw(wheelShader_ref);
    // std::cout << "OpenGL_config_CLASS::wheelShaderUpdate complete" << std::endl;
}
