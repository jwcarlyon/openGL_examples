//I think this loads a camera angle which views a model
/*Successfully compiled using command $ g++ ground_lighting_env.cpp glad.c -lGL -lglfw3 -Iglad -lX11 -ldl -lpthread -lassimp
  !Update, 30May2022 added bulletAPI led me to this compile cmd:
  g++ ground_lighting_env.cpp ../glad/glad.c -lglfw -ldl -lassimp -lPhysXGpu_64
  on inteli9 10900k, asus hero xii no graphics card
  */
//https://stackoverflow.com/questions/18389211/how-to-animate-a-3d-model-mesh-in-opengl
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
//g++ ground_lighting_env.cpp ../glad/glad.c -lglfw -ldl -lassimp -I../PhysX/pxshared/include -I../PhysX/physx/include -L../PhysX/physx/bin/linux.aarch64/checked -lPhysX_static_64 -lPhysXPvdSDK_static_64 -lPhysXExtensions_static_64 -lPhysXCooking_static_64 -lPhysXCommon_static_64 -lPhysXFoundation_static_64 -lPhysXCharacterKinematic_static_64 -lPhysXVehicle_static_64 -lSnippetUtils_static_64
//g++  ground_lighting_env.cpp ../glad/glad.c -lglfw -ldl -lassimp -I../PhysX/pxshared/include/ -I../PhysX/physx/include/ ../PhysX/physx/bin/linux.aarch64/debug/libPhysXCharacterKinematic_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXCommon_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXCooking_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXExtensions_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXFoundation_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysX_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXVehicle_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libSnippetUtils_static_64.a ../PhysX/physx/bin/linux.aarch64/debug/libPhysXPvdSDK_static_64.a
//libPhysXGpu_64.so

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <OpenGL_CLASS/Camera_CLASS.h>
#include <OpenGL_CLASS/Shader_CLASS.h>
#include <OpenGL_CLASS/Model_CLASS.h>
#include <OpenGL_CLASS/Mesh_CLASS.h>
#include <iostream>
#include <chrono>
#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS

#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem; //linux install ghc in bash
#endif
namespace chr = std::chrono::_V2;//::steady_clock
using Framerate = std::chrono::duration<chr::steady_clock::rep, std::ratio<1, 60>>;

/* The following is cut from bullet physics' README:
The entire collision detection and rigid body dynamics can be executed on the GPU.

A high-end desktop GPU, such as an AMD Radeon 7970 or NVIDIA GTX 680 or better.
We succesfully tested the software under Windows, Linux and Mac OSX.
The software currently doesn't work on OpenCL CPU devices. It might run
on a laptop GPU but performance will not likely be very good. Note that
often an OpenCL drivers fails to compile a kernel.
*/

void advance_time_in_fixed_timestep();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void ground_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3& light_source_vec3_ref, Shader& object_shader_ref);
void lamp_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3 lamp_start_position_vec3, glm::vec3 center_pos_vec3, float& radial_position_fl_ref, glm::vec3& light_source_vec3_ref, Shader& lamp_shader_ref);
void process_input(GLFWwindow *window);
void process_mouse_position_callback(GLFWwindow* window, double mouse_x_position, double mouse_y_position);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void tire_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3& light_source_vec3_ref, Shader& object_shader_ref);
glm::vec3 updated_orbit_path_vec3(glm::vec3 center_pos_vec3, glm::vec3 planet_pos_vec3, float& radial_position_fl_ref);
// settings
const unsigned int SCR_WIDTH = 1920 * 2 / 3;//1100;
const unsigned int SCR_HEIGHT = 1080 * 2 / 3;//825;

// camera and mowse
glm::vec3 camera_position_vec3(0.0f, 0.0f, 3.0f);
Camera camera(camera_position_vec3);
float mouse_last_x_fl = SCR_WIDTH / 2.0f;
float mouse_last_y_fl = SCR_HEIGHT / 2.0f;
float mouse_yaw_fl   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float mouse_pitch_fl = 0.0f;
bool is_first_mouse = true;

// timing
auto next_render_clock = chr::steady_clock::now() + Framerate{1};

//scene and models
static int num_of_dummies = 1;
float radial_position_fl = 0.0f;
float& radial_position_fl_ref = radial_position_fl;
glm::mat4 light_m4 = glm::mat4(1.0f);
glm::mat4 model_m4;
glm::mat4 projection_m4, view_m4;
glm::vec3 camera_front_vec3 = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 center_pos_vec3(0.67f, -0.5f, 0.0f);
glm::vec3 camera_up_vec3 = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 light_source_vec3(0.0f);
glm::vec3 lamp_start_position_vec3(1.0f, 1.0f, -1.0f);
glm::vec3 light_color_vec3(1.0f, 1.0f, 1.0f);
glm::vec3& light_source_vec3_ref = light_source_vec3;

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "House, pavement and sandbox", NULL, NULL);
    if(window == NULL)
    {
      std::cout << "Failed to create GLFW window" << std::endl;
      glfwTerminate(); return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, process_mouse_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {std::cout << "Failed to initialize GLAD" << std::endl; return -1;}
    glEnable(GL_DEPTH_TEST);

    // scene
    Shader lamp_shader("lamp_loading.vs", "lamp_loading.fs"),
  		ground_shader("ground_shader.vs", "ground_shader.fs"),
  		tire_shader("tire_shader.vs", "tire_shader.fs");
  	Model bulb_model(fs::u8path("bulb.obj")),
  		ground_model(fs::u8path("pavement_MODEL.fbx")),
  		tire_model(fs::u8path("tire_MODEL.fbx"));
      //tire model has very weird behavior...
      //whynot bring all assets into one scene and export them piecemeal??Dummy?Huh?22May2022
    Shader& lamp_shader_ref = lamp_shader,
  					ground_shader_ref = ground_shader,
  					tire_shader_ref = tire_shader;
    // bulb_model is drawn by an unspecialized shader(lamp_shader). This shader only
    // gets the view, model and projection matricies -  reflections are drawn by
    // other shaders that track the object which is being reflected. I think this
  	// unspecialized shader just draws whatever unassigned vao is present. Is
  	// that why when an object shader bricks, the object is drawn here??

    while(!glfwWindowShouldClose(window))
    {
        process_input(window);

        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        projection_m4 = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        //view_m4 = camera.GetViewMatrix();
    		view_m4 = glm::lookAt(
    			camera_position_vec3,
    			camera_position_vec3 + camera_front_vec3,
    			camera_up_vec3
    		);

        lamp_shader_update(
    			projection_m4,
    			view_m4,
    			lamp_start_position_vec3,
          center_pos_vec3,
    			radial_position_fl_ref,
    			light_source_vec3_ref,
    			lamp_shader_ref
    		);
        bulb_model.Draw(lamp_shader);

    		ground_shader_update(projection_m4, view_m4, light_source_vec3_ref, ground_shader_ref);
    		ground_model.Draw(ground_shader);
    		tire_shader_update(projection_m4, view_m4, light_source_vec3_ref, tire_shader_ref);
    		tire_model.Draw(tire_shader);

        advance_time_in_fixed_timestep();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

glm::vec3 updated_orbit_path_vec3(glm::vec3 center_pos_vec3, glm::vec3 planet_pos_vec3, float& radial_position_fl_ref)
{
	//this function will keep obj in an orbit around center
  //rotation of the light bulb about the car
  float x_position_fl, z_position_fl, theta_delta_fl;
  theta_delta_fl = 0.005f;
  radial_position_fl_ref = radial_position_fl_ref + theta_delta_fl;
  if(radial_position_fl_ref >= (2.0f * 3.14159f))
  {radial_position_fl_ref = radial_position_fl_ref - (2.0f * 3.14159f);}

  glm::vec3 orbit_center_vec3((center_pos_vec3.x + 0.0f), center_pos_vec3.y, (center_pos_vec3.z + 0.0f));
  x_position_fl = orbit_center_vec3.x +
    (cos(radial_position_fl_ref) * (planet_pos_vec3.x - orbit_center_vec3.x)) -
    (sin(radial_position_fl_ref) * (planet_pos_vec3.z - orbit_center_vec3.z));
  z_position_fl = orbit_center_vec3.z +
    (sin(radial_position_fl_ref) * (planet_pos_vec3.x - orbit_center_vec3.x)) -
    (cos(radial_position_fl_ref) * (planet_pos_vec3.z - orbit_center_vec3.z));
  return glm::vec3(x_position_fl, 0.0f, z_position_fl);//return the rotated vector?? #notamathboy
}

void advance_time_in_fixed_timestep()
{
  while(chr::steady_clock::now() < next_render_clock);
  //"busy" loop - chrono just observes a pre-set time window
  next_render_clock += Framerate{1}; //this is a singleton window value
}

void ground_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3& light_source_vec3_ref, Shader& object_shader_ref)
{
	model_m4 = glm::translate(glm::mat4(1.0f), (center_pos_vec3 + glm::vec3(0.0f, -16.0f, -1.0f)));
	// model_m4 = glm::scale(model_m4, glm::vec3(0.25));

	object_shader_ref.use();
	object_shader_ref.setVec3("lightColor", light_color_vec3);
	object_shader_ref.setVec3("lightPos", light_source_vec3_ref);//used to calculate viewspace for reflection
	object_shader_ref.setVec3("viewerPos", glm::vec3(0.0f));// camera.Position);
	object_shader_ref.setMat4("model", model_m4);
	object_shader_ref.setMat4("projection", projection_m4);
	object_shader_ref.setMat4("view", view_m4);
}

void tire_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3& light_source_vec3_ref, Shader& object_shader_ref)
{
    model_m4 = glm::scale(model_m4, glm::vec3(1.0f, 1.0f, 0.6f));
    model_m4 = glm::rotate(model_m4,  -(3.14159f / 2), glm::vec3(1.0f, 0.0f, 0.0f));
    model_m4 = glm::translate(glm::mat4(1.0f), (center_pos_vec3 + glm::vec3(0.0f, 0.0f, 0.5f)));

    object_shader_ref.use();
    object_shader_ref.setVec3("lightColor", light_color_vec3);
    object_shader_ref.setVec3("lightPos", light_source_vec3_ref);//used to calculate viewspace for reflection
    object_shader_ref.setVec3("viewerPos", glm::vec3(0.0f));// camera.Position);
    object_shader_ref.setMat4("model", model_m4);//model position
    object_shader_ref.setMat4("projection", projection_m4);
    object_shader_ref.setMat4("view", view_m4);
}

void lamp_shader_update(glm::mat4 projection_m4, glm::mat4 view_m4, glm::vec3 lamp_start_position_vec3, glm::vec3 center_pos_vec3, float& radial_position_fl_ref, glm::vec3& light_source_vec3_ref, Shader& lamp_shader_ref)
{
	//this function will render lights for a single frame
  lamp_shader_ref.use();
  light_source_vec3_ref = updated_orbit_path_vec3(center_pos_vec3, lamp_start_position_vec3, radial_position_fl);
  light_m4 = glm::translate(glm::mat4(1.0f), center_pos_vec3);
  light_m4 = glm::translate(light_m4, light_source_vec3_ref);//this is a translation to light_position then rotation
  light_m4 = glm::scale(light_m4, glm::vec3(0.00025f));// it's a bit too big for our scene, so scale it down

  lamp_shader_ref.setMat4("projection", projection_m4);
  lamp_shader_ref.setMat4("view", view_m4);
  lamp_shader_ref.setMat4("model", light_m4);
  //lamp_shader_ref.setVec3("aPos", light_source_vec3_ref);
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

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
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
		if (mouse_pitch_fl > 89.0f){
        mouse_pitch_fl = 89.0f;
	  } else if (mouse_pitch_fl < -89.0f){
	      mouse_pitch_fl = -89.0f;
		}
	  glm::vec3 front_vec3(
			  cos(glm::radians(mouse_yaw_fl)) * cos(glm::radians(mouse_pitch_fl)),
			  sin(glm::radians(mouse_pitch_fl)),
			  sin(glm::radians(mouse_yaw_fl)) * cos(glm::radians(mouse_pitch_fl))
		);
	  camera_front_vec3 = glm::normalize(front_vec3);
}
