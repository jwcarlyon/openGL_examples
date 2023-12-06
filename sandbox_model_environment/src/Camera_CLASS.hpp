

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

#ifndef CAMERA_CLASS_HEADER
#define CAMERA_CLASS_HEADER

enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const float YAW         = -90.0f;
const float PITCH       =  0.0f;
const float SPEED       =  2.5f;
const float SENSITIVITY =  0.1f;
const float ZOOM        =  45.0f;


class Camera
{
public:
    Camera(glm::vec3 position_vec3 = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up_vec3 = glm::vec3(0.0f, 1.0f, 0.0f), float yaw_fl = YAW, float pitch_fl = PITCH);
    Camera(float xPosition_fl, float yPosition_fl, float zPosition_fl, float upDirectionXValue_fl, float upDirectionYValue_fl, float upDirectionZValue_fl, float yaw_fl, float pitch_fl);

    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    float Yaw; //Euler angles
    float Pitch;
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    glm::mat4 GetViewMatrix();
    void ProcessKeyboard(Camera_Movement direction, float deltaTime_fl);
    void ProcessMouseMovement(float xAxisDelta_fl, float yAxisDelta_fl, GLboolean isPitchConstrained = true);
    void ProcessMouseScroll(float verticalWheelAxisInput_fl);

private:
    void updateCameraVectors();
};
#endif
