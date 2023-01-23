#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include <cmath>
#include "../bullet3/examples/Vehicles/Hinge2VehicleCustom.cpp"
namespace chr = std::chrono::_V2;
using Framerate = std::chrono::duration<chr::steady_clock::rep, std::ratio<1, 60>>;

void advance_time_in_fixed_timestep();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void game_engine_loop();
void vehicle_shader_update(Shader& object_shader_ref, btTransform transformation);
void open_gl_setup();
void open_gl_destroy();
void process_input(GLFWwindow *window);
void process_mouse_position_callback(GLFWwindow* window, double mouse_x_position, double mouse_y_position);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void ground_shader_update(Shader& object_shader_ref);
void wheel_object_update(Shader& object_shader_ref, btTransform transformation, int wheel_location);

// Variables
// timing
auto next_render_clock = chr::steady_clock::now() + Framerate{1};
// settings
const unsigned int SCR_WIDTH = 1920 * 2 / 3;//1100;
const unsigned int SCR_HEIGHT = 1080 * 2 / 3;//825;
GLFWwindow* window;
// camera and mouse
glm::vec3 camera_position_vec3(0.0f, 1.0f, 3.0f);
glm::vec3 camera_front_vec3(0.0f, 0.0f, -1.0f);
glm::vec3 camera_up_vec3(0.0f, 1.0f, 0.0f);
glm::mat4 model_m4, projection_m4, view_m4;
Camera camera(camera_position_vec3);
float mouse_last_x_fl = SCR_WIDTH / 2.0f;
float mouse_last_y_fl = SCR_HEIGHT / 2.0f;
float mouse_yaw_fl = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float mouse_pitch_fl = 0.0f;
bool is_first_mouse = true;
//scene and models maths
glm::vec3 center_pos_vec3(0.67f, 0.5f, -6.0f);
glm::vec3 ground_position_vec3(center_pos_vec3 + glm::vec3(0.0f, -14.0f, -1.0f));
glm::vec3 light_color_vec3(1.0f, 1.0f, 1.0f);
glm::vec3 light_source_vec3(1.0f, 5.0f, -0.5f);
glm::vec3 vehicle_position_vec3(1.0f, 5.0f, -0.5f);
//physics
static Hinge2Vehicle* vehicle_rigid_body_base = new Hinge2Vehicle(0);

void advance_time_in_fixed_timestep()
{
  //"busy" loop - chrono just observes a pre-set time window
    while(chr::steady_clock::now() < next_render_clock);
    next_render_clock += Framerate{1}; //this is a singleton window value
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void game_engine_loop()
{
    Model vehicle_model(fs::u8path("../assets/frank_grimes_deluxe_Asset/fgd_MODEL.fbx")),
        ground_model(fs::u8path("../assets/pavement_Asset/pavement_MODEL.fbx")),
        tire_model(fs::u8path("../assets/tire_Asset/tire_MODEL.fbx"));
    Shader vehicle_shader("../assets/frank_grimes_deluxe_Asset/vehicle.vs", "../assets/frank_grimes_deluxe_Asset/vehicle.fs"),
    		ground_shader("../assets/pavement_Asset/ground.vs", "../assets/pavement_Asset/ground.fs"),
    		tire_shader("../assets/tire_Asset/tire.vs", "../assets/tire_Asset/tire.fs");
    Shader& ground_shader_ref = ground_shader;
    Shader& vehicle_shader_ref = vehicle_shader;
    Shader& tire_shader_ref = tire_shader;

    vehicle_rigid_body_base->initPhysics();
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0.f, 1.f, 0.f), 14.75f);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(
        btVector3(ground_position_vec3.x, ground_position_vec3.y, ground_position_vec3.z)
    );
    //rigidbody is dynamic if and only if groundMass_fl is non zero, otherwise static
    btScalar groundMass_fl(0.f);
    btVector3 groundInertia_btv3(0, 0, 0);
    if(groundMass_fl != 0.f) //if mass is 0, it is not dynamic
    {   groundShape->calculateLocalInertia(groundMass_fl, groundInertia_btv3);}
  	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
  	btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
  	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyInfo(
        groundMass_fl,
        groundMotionState,
        groundShape,
        groundInertia_btv3
    );
  	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyInfo);
    groundRigidBody->setRestitution(0.9f);
    vehicle_rigid_body_base->m_dynamicsWorld->addRigidBody(groundRigidBody);

    while(!glfwWindowShouldClose(window))
    {
        process_input(window);

        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    		vehicle_rigid_body_base->stepSimulation(1.f / 60.f);
        advance_time_in_fixed_timestep();
        projection_m4 = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        view_m4 = glm::lookAt(
            camera_position_vec3,
            camera_position_vec3 + camera_front_vec3,
            camera_up_vec3
        );
        for(int i = 0; i < vehicle_rigid_body_base->m_dynamicsWorld->getNumCollisionObjects(); i++)
        {
            btCollisionObject* obj = vehicle_rigid_body_base->m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);

            btTransform transform;
            if(body && body->getMotionState())
            {
                body->getMotionState()->getWorldTransform(transform);
            } else {
                transform = obj->getWorldTransform();
            }
            btVector3 origin = transform.getOrigin();
            btQuaternion rotation = transform.getRotation();
            btVector3 axis = rotation.getAxis();
            btScalar angle = rotation.getAngle();
            if (i == 0) { continue; }
            if (i == 1)
            {
                vehicle_shader_update(vehicle_shader_ref, transform);
                vehicle_model.Draw(vehicle_shader_ref);
                std::printf("Vehicle#: %d, position (%f, %f, %f) rotation axis: (%f, %f, %f), angle: (%f)\n",
                    i,
                    float(origin.getX()),
                    float(origin.getY()),
                    float(origin.getZ()),
                    float(axis.getX()),
                    float(axis.getY()),
                    float(axis.getZ()),
                    float(angle)
                );
            } else if(i < 6) {
                wheel_object_update(tire_shader_ref, transform, i);
                tire_model.Draw(tire_shader);
                std::printf("Wheel#: %d, position (%f, %f, %f)  rotation axis: (%f, %f, %f), angle: (%f)\n",
                    i,
                    float(origin.getX()),
                    float(origin.getY()),
                    float(origin.getZ()),
                    float(axis.getX()),
                    float(axis.getY()),
                    float(axis.getZ()),
                    float(angle)
                );
            } else {
              ground_shader_update(ground_shader_ref);
              ground_model.Draw(ground_shader);
              std::printf("Ground#: %d, position (%f, %f, %f) rotation axis: (%f, %f, %f), angle: (%f)\n",
                  i,
                  float(origin.getX()),
                  float(origin.getY()),
                  float(origin.getZ()),
                  float(axis.getX()),
                  float(axis.getY()),
                  float(axis.getZ()),
                  float(angle)
              );
            }
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    vehicle_rigid_body_base->exitPhysics();
}

void ground_shader_update(Shader& object_shader_ref)
{
    // glm::vec3 axis_vec3(0.2f, 0.8f, 0.0f);
    model_m4 = glm::translate(glm::mat4(1.0f), ground_position_vec3);
    // model_m4 = glm::rotate(model_m4, 3.14159f/2.f, axis_vec3);
    // model_m4 = glm::translate(glm::mat4(1.0f), (center_pos_vec3 + glm::vec3(0.0f, -16.0f, -1.0f)));
  	// model_m4 = glm::scale(model_m4, glm::vec3(0.25));

  	object_shader_ref.use();
  	object_shader_ref.setVec3("lightColor", light_color_vec3);
  	object_shader_ref.setVec3("lightPos", light_source_vec3);//used to calculate viewspace for reflection
  	object_shader_ref.setVec3("viewerPos", glm::vec3(0.0f));// camera.Position);
  	object_shader_ref.setMat4("model", model_m4);
  	object_shader_ref.setMat4("projection", projection_m4);
  	object_shader_ref.setMat4("view", view_m4);
}

void open_gl_setup()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Sandbox", 0, 0);
    if(window == 0)
    {
      std::cout << "Failed to create GLFW window" << std::endl;
      glfwTerminate(); return;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, process_mouse_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {   std::cout << "Failed to initialize GLAD" << std::endl; return; }
    glEnable(GL_DEPTH_TEST);
}

void open_gl_destroy()
{
    glfwTerminate();
}

void process_input(GLFWwindow *window)
{
		float camera_speed_fl = 0.5f;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
				glfwSetWindowShouldClose(window, true);
		} else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        camera_position_vec3 += camera_speed_fl * camera_front_vec3;
		} else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        camera_position_vec3 -= camera_speed_fl * camera_front_vec3;
    } else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        camera_position_vec3 -= glm::normalize(glm::cross(camera_front_vec3, camera_up_vec3)) * camera_speed_fl;
    } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        camera_position_vec3 += glm::normalize(glm::cross(camera_front_vec3, camera_up_vec3)) * camera_speed_fl;
		}
}

void process_mouse_position_callback(GLFWwindow* window, double mouse_x_position, double mouse_y_position)
{
	  if (is_first_mouse)
	  {
		    mouse_last_x_fl = mouse_x_position;
		    mouse_last_y_fl = mouse_y_position;
		    is_first_mouse = false;
	  }
	  float mouse_x_offset = mouse_x_position - mouse_last_x_fl;
	  float mouse_y_offset = mouse_last_y_fl - mouse_y_position; // reversed since y-coordinates go from bottom to top
	  mouse_last_x_fl = mouse_x_position;
	  mouse_last_y_fl = mouse_y_position;

		// make sure that when pitch is out of bounds, screen doesn't get flipped
		mouse_yaw_fl += mouse_x_offset;
	  mouse_pitch_fl += mouse_y_offset;
		if (mouse_pitch_fl > 89.0f)
    {
        mouse_pitch_fl = 89.0f;
	  } else if (mouse_pitch_fl < -89.0f) {
	      mouse_pitch_fl = -89.0f;
		}
	  glm::vec3 front_vec3(
			  cos(glm::radians(mouse_yaw_fl)) * cos(glm::radians(mouse_pitch_fl)),
			  sin(glm::radians(mouse_pitch_fl)),
			  sin(glm::radians(mouse_yaw_fl)) * cos(glm::radians(mouse_pitch_fl))
		);
	  camera_front_vec3 = glm::normalize(front_vec3);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

void vehicle_shader_update(Shader& object_shader_ref, btTransform transformation)
{
  btVector3 origin = transformation.getOrigin();
  float vehicle_start_rotation_fl = M_PI / 2.f;
  vehicle_position_vec3.x = origin.getX();//trans.getOrigin().getX();
  vehicle_position_vec3.y = origin.getY();//trans.getOrigin().getY();
  vehicle_position_vec3.z = origin.getZ();//trans.getOrigin().getZ();
  btQuaternion rotation = transformation.getRotation();
  btScalar angle_btScalar = rotation.getAngle();
  btVector3 axis = rotation.getAxis();
  glm::vec3 axis_vec3(axis.getX(), axis.getY(), axis.getZ());
  glm::vec3 vehicle_offset_vec3(vehicle_position_vec3 + glm::vec3(0.f, 1.f, -1.f));

  model_m4 = glm::translate(glm::mat4(1.0f), vehicle_offset_vec3);//this is a translation to light_position then rotation
  model_m4 = glm::rotate(model_m4, vehicle_start_rotation_fl, glm::vec3(-1.0f, 0.0f, 0.0f));
  model_m4 = glm::rotate(model_m4, vehicle_start_rotation_fl, glm::vec3(0.0f, 0.0f, 1.0f));
  model_m4 = glm::rotate(model_m4, float(angle_btScalar), axis_vec3);
  model_m4 = glm::scale(model_m4, glm::vec3(1.4f));

  object_shader_ref.use();
  object_shader_ref.setVec3("lightColor", light_color_vec3);
  object_shader_ref.setVec3("lightPos", light_source_vec3);//used to calculate viewspace for reflection
  object_shader_ref.setVec3("viewerPos", glm::vec3(0.0f));// camera.Position);
  object_shader_ref.setMat4("model", model_m4);//model position
  object_shader_ref.setMat4("projection", projection_m4);
  object_shader_ref.setMat4("view", view_m4);
}

void wheel_object_update(Shader& object_shader_ref, btTransform transformation, int wheel_location)
{
    float wheel_start_rotation_fl = M_PI / 2.f;
    if(wheel_location >= 4) { wheel_start_rotation_fl += M_PI; }
    btVector3 origin = transformation.getOrigin();
    btQuaternion rotation = transformation.getRotation();
    btScalar angle_btScalar = rotation.getAngle();
    btVector3 axis = rotation.getAxis();
    glm::vec3 axis_vec3(axis.getX(), axis.getY(), axis.getZ());
    glm::vec3 wheel_position_vec3(origin.getX(), origin.getY(), origin.getZ());

    model_m4 = glm::translate(glm::mat4(1.0f), wheel_position_vec3);
    model_m4 = glm::scale(model_m4, glm::vec3(1.0f, 1.0f, 0.4f));
    // model_m4 = glm::scale(model_m4, glm::vec3(2.0f, 2.0f, 1.0f));
    // model_m4 = glm::rotate(model_m4, wheel_start_rotation_fl, glm::vec3(0.0f, 1.0f, 0.0f));
    model_m4 = glm::rotate(model_m4, float(angle_btScalar), axis_vec3);
    model_m4 = glm::rotate(model_m4, wheel_start_rotation_fl, glm::vec3(-1.0f, 0.0f, 0.0f));

    object_shader_ref.use();
    object_shader_ref.setVec3("lightColor", light_color_vec3);
    object_shader_ref.setVec3("lightPos", light_source_vec3);//used to calculate viewspace for reflection
    object_shader_ref.setVec3("viewerPos", glm::vec3(0.0f));// camera.Position);
    object_shader_ref.setMat4("model", model_m4);//model position
    object_shader_ref.setMat4("projection", projection_m4);
    object_shader_ref.setMat4("view", view_m4);
}
