#include "Camera_CLASS.hpp"

Camera::Camera(glm::vec3 position_vec3, glm::vec3 up_vec3, float yaw_fl, float pitch_fl) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    Position = position_vec3;
    WorldUp = up_vec3;
    Yaw = yaw_fl;
    Pitch = pitch_fl;
    updateCameraVectors();
}
// Constructor with scalar values
Camera::Camera(float xPosition_fl, float yPosition_fl, float zPosition_fl, float upDirectionXValue_fl, float upDirectionYValue_fl, float upDirectionZValue_fl, float yaw_fl, float pitch_fl) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
{
    Position = glm::vec3(xPosition_fl, yPosition_fl, zPosition_fl);
    WorldUp = glm::vec3(upDirectionXValue_fl, upDirectionYValue_fl, upDirectionZValue_fl);
    Yaw = yaw_fl;
    Pitch = pitch_fl;
    updateCameraVectors();
}

// Returns the view matrix calculated using Euler Angles and the LookAt Matrix
glm::mat4 Camera::GetViewMatrix()
{
    return glm::lookAt(Position, Position + Front, Up);
}

// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime_fl)
{
    float velocity_fl = MovementSpeed * deltaTime_fl;
    if(direction == FORWARD)
        Position += Front * velocity_fl;
    if(direction == BACKWARD)
        Position -= Front * velocity_fl;
    if(direction == LEFT)
        Position -= Right * velocity_fl;
    if(direction == RIGHT)
        Position += Right * velocity_fl;
}

// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
void Camera::ProcessMouseMovement(float xAxisDelta_fl, float yAxisDelta_fl, GLboolean isPitchConstrained)
{
    xAxisDelta_fl *= MouseSensitivity;
    yAxisDelta_fl *= MouseSensitivity;

    Yaw += xAxisDelta_fl;
    Pitch += yAxisDelta_fl;

    // Make sure that when pitch is out of bounds, screen doesn't get flipped
    if(isPitchConstrained)
    {
        if(Pitch > 89.0f) {
            Pitch = 89.0f;
        } else if(Pitch < -89.0f) {
            Pitch = -89.0f;
        }
    }

    // Update Front, Right and Up Vectors using the updated Euler angles
    updateCameraVectors();
}

// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
void Camera::ProcessMouseScroll(float verticalWheelAxisInput_fl)
{
    if(Zoom >= 1.0f && Zoom <= 45.0f)
    {   Zoom -= verticalWheelAxisInput_fl;  }

    if(Zoom <= 1.0f) {
        Zoom = 1.0f;
    } else if(Zoom >= 45.0f) {
        Zoom = 45.0f;
    }
}

// Calculates the front vector from the Camera's (updated) Euler Angles
void Camera::updateCameraVectors()
{
    // Calculate the new Front vector
    glm::vec3 front_vec3;
    front_vec3.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front_vec3.y = sin(glm::radians(Pitch));
    front_vec3.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front_vec3);
    // Also re-calculate the Right and Up vector
    Right = glm::normalize(glm::cross(Front, WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    Up = glm::normalize(glm::cross(Right, Front));
}
