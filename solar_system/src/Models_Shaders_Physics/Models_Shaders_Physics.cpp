#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include <cmath>
namespace chr = std::chrono::_V2;
using Framerate = std::chrono::duration<chr::steady_clock::rep, std::ratio<1, 60>>;

void advance_time_in_fixed_timestep();
void camera_track_object_position(btTransform planet_transformation, btTransform sun_transformation);
void game_engine_loop();
void lamp_object_update(glm::mat4 projection_m4, glm::mat4 view_m4,
  glm::vec3& light_source_vec3_ref, Shader& object_shader_ref, btTransform transformation
);
void moon_object_update(glm::mat4 projection_m4, glm::mat4 view_m4,
  glm::vec3& moon_position_vec3_ref, glm::vec3& light_source_vec3_ref,
  Shader& object_shader_ref, btTransform transformation
);
btRigidBody *object_physics_setup(btCollisionShape * collision_shape_ptr, glm::vec3& object_position_vec3_ref);
void open_gl_setup();
void open_gl_destroy();
void process_input(GLFWwindow *window);
void process_mouse_position_callback(GLFWwindow* window, double mouse_x_position, double mouse_y_position);
void physics_setup();
void physics_destroy();
btVector3 relative_gravity(btRigidBody *rigid_body_ptr);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
// Variables
// timing
auto next_render_clock = chr::steady_clock::now() + Framerate{1};
// settings
const unsigned int SCR_WIDTH = 1920 * 2 / 3;//1100;
const unsigned int SCR_HEIGHT = 1080 * 2 / 3;//825;
GLFWwindow* window;
// camera and mouse
glm::vec3 camera_position_vec3(0.0f, 0.0f, 3.0f);
glm::vec3 camera_front_vec3(0.0f, 0.0f, -1.0f);
glm::vec3 camera_up_vec3(0.0f, 1.0f, 0.0f);
glm::mat4 projection_m4, view_m4;
Camera camera(camera_position_vec3);
float mouse_last_x_fl = SCR_WIDTH / 2.0f;
float mouse_last_y_fl = SCR_HEIGHT / 2.0f;
float mouse_yaw_fl = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float mouse_pitch_fl = 0.0f;
bool is_first_mouse = true;
//scene and models maths
glm::vec3 center_pos_vec3(0.67f, 0.5f, -6.0f);
glm::vec3 light_source_vec3(center_pos_vec3 + glm::vec3(1.0f, 10.5f, -1.0f));
glm::vec3 moon_position_vec3(0.f);
glm::vec3 light_color_vec3(1.0f, 1.0f, 1.0f);
glm::vec3& light_source_vec3_ref = light_source_vec3;
glm::vec3& moon_position_vec3_ref = moon_position_vec3;
glm::mat4 light_m4 = glm::mat4(1.0f);
glm::mat4 model_m4;
//physics
btDiscreteDynamicsWorld *dynamicsWorld;
std::vector<const btRigidBody*> rigid_body_ptr_vector;
btAlignedObjectArray<btCollisionShape*> collision_shapes_btAlignedObjectArray;
btDefaultCollisionConfiguration *collisionConfiguration;
///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
btCollisionDispatcher *dispatcher;
///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
btBroadphaseInterface *overlappingPairCache;
///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
btSequentialImpulseConstraintSolver *solver;
const float GRAVITATIONAL_CONSTANT = 6.6743f * pow(10, -11);

void advance_time_in_fixed_timestep()
{
  //"busy" loop - chrono just observes a pre-set time window
    while(chr::steady_clock::now() < next_render_clock);
    next_render_clock += Framerate{1}; //this is a singleton window value
}

void camera_track_object_position(btTransform planet_transformation, btTransform sun_transformation)
{
    glm::vec3 sun_position_vec3(
        sun_transformation.getOrigin().getX(),
        sun_transformation.getOrigin().getY(),
        sun_transformation.getOrigin().getZ()
    );
    glm::vec3 planet_position_vec3(
        planet_transformation.getOrigin().getX(),
        planet_transformation.getOrigin().getY(),
        planet_transformation.getOrigin().getZ()
    );
    glm::vec3 new_camera_positon_vec3(glm::mix(sun_position_vec3, camera_position_vec3, 0.1f));
    camera_front_vec3 = planet_position_vec3 - new_camera_positon_vec3;
    camera_position_vec3 = new_camera_positon_vec3 - (0.05f * camera_front_vec3);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void game_engine_loop()
{
    Model bulb_model(fs::u8path("../assets/lamp_Asset/bulb.obj"));
    Model moon_model(fs::u8path("../assets/moon_Asset/world.obj"));
    Shader lamp_shader("../assets/lamp_Asset/lamp.vs", "../assets/lamp_Asset/lamp.fs");
    Shader moon_shader("../assets/moon_Asset/moon.vs", "../assets/moon_Asset/moon.fs");
    Shader& lamp_shader_ref = lamp_shader;
    Shader& moon_shader_ref = moon_shader;
    btCollisionShape *lamp_shape_ptr = new btSphereShape(btScalar(0.01f));
    btCollisionShape *moon_shape_ptr = new btSphereShape(btScalar(0.01f));
    btRigidBody *lamp_rigid_body_ptr = object_physics_setup(lamp_shape_ptr, light_source_vec3_ref);
    btRigidBody *moon_rigid_body_ptr = object_physics_setup(moon_shape_ptr, moon_position_vec3_ref);
    lamp_rigid_body_ptr->setGravity(relative_gravity(lamp_rigid_body_ptr));
    moon_rigid_body_ptr->setGravity(relative_gravity(moon_rigid_body_ptr));
    moon_rigid_body_ptr->setLinearVelocity(btVector3(5.0f, 2.5f, 0.f));
    while(!glfwWindowShouldClose(window))
    {
        process_input(window);

        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        projection_m4 = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        view_m4 = glm::lookAt(
            camera_position_vec3,
            camera_position_vec3 + camera_front_vec3,
            camera_up_vec3
        );
        dynamicsWorld->stepSimulation(1.f / 60.f);
        advance_time_in_fixed_timestep();
        btTransform planet_transform, sun_transform;
        lamp_rigid_body_ptr->setGravity(relative_gravity(lamp_rigid_body_ptr));
        lamp_rigid_body_ptr->getMotionState()->getWorldTransform(sun_transform);
        lamp_object_update(
            projection_m4,
            view_m4,
            light_source_vec3_ref,
            lamp_shader_ref,
            sun_transform
        );
        bulb_model.Draw(lamp_shader);
        moon_rigid_body_ptr->setGravity(relative_gravity(moon_rigid_body_ptr));
        moon_rigid_body_ptr->getMotionState()->getWorldTransform(planet_transform);
        moon_object_update(
            projection_m4,
            view_m4,
            moon_position_vec3_ref,
            light_source_vec3_ref,
            moon_shader_ref,
            planet_transform
        );
        moon_model.Draw(moon_shader);
        camera_track_object_position(planet_transform, sun_transform);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void lamp_object_update(glm::mat4 projection_m4, glm::mat4 view_m4,
  glm::vec3& light_source_vec3_ref, Shader& object_shader_ref, btTransform transformation
) {
    btVector3 origin = transformation.getOrigin();
    light_source_vec3_ref.x = origin.getX();//trans.getOrigin().getX();
    light_source_vec3_ref.y = origin.getY();//trans.getOrigin().getY();
    light_source_vec3_ref.z = origin.getZ();//trans.getOrigin().getZ();

    light_m4 = glm::translate(glm::mat4(1.0f), light_source_vec3_ref);//this is a translation to light_position then rotation
    light_m4 = glm::scale(light_m4, glm::vec3(0.00025f));// it's a bit too big for our scene, so scale it down

    object_shader_ref.use();
    object_shader_ref.setMat4("projection", projection_m4);
    object_shader_ref.setMat4("view", view_m4);
    object_shader_ref.setMat4("model", light_m4);
}

btRigidBody *object_physics_setup(btCollisionShape * collision_shape_ptr, glm::vec3& object_position_vec3_ref)
{
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(
        btVector3(object_position_vec3_ref.x, object_position_vec3_ref.y, object_position_vec3_ref.z)
    );
    btVector3 local_inertia_btVec3(0, 0, 0);
    btScalar mass_btScalar(2 * pow(10, 6));
    btScalar restitution_btScalar(0.69f);
    collision_shape_ptr->calculateLocalInertia(mass_btScalar, local_inertia_btVec3);
    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody *physics_properties_body_ptr = new btRigidBody(
        btRigidBody::btRigidBodyConstructionInfo(
                mass_btScalar, myMotionState, collision_shape_ptr, local_inertia_btVec3
        )
    );
    physics_properties_body_ptr->setRestitution(restitution_btScalar);
    dynamicsWorld->addRigidBody(physics_properties_body_ptr);
    rigid_body_ptr_vector.push_back(physics_properties_body_ptr);
    collision_shapes_btAlignedObjectArray.push_back(collision_shape_ptr);
    return physics_properties_body_ptr;
}

void moon_object_update(glm::mat4 projection_m4, glm::mat4 view_m4,
  glm::vec3& moon_position_vec3_ref, glm::vec3& light_source_vec3_ref,
  Shader& object_shader_ref, btTransform transformation
) {
    btVector3 origin = transformation.getOrigin();
    moon_position_vec3_ref.x = origin.getX();//trans.getOrigin().getX();
    moon_position_vec3_ref.y = origin.getY();//trans.getOrigin().getY();
    moon_position_vec3_ref.z = origin.getZ();//trans.getOrigin().getZ();

    light_m4 = glm::translate(glm::mat4(1.0f), moon_position_vec3_ref);//this is a translation to light_position then rotation
    light_m4 = glm::scale(light_m4, glm::vec3(5.f));// it's a bit too big for our scene, so scale it down

    object_shader_ref.use();
    object_shader_ref.setVec3("lightColor", glm::vec3(1.0f));
    object_shader_ref.setVec3("lightPos", light_source_vec3_ref);//used to calculate viewspace for reflection
    object_shader_ref.setVec3("viewerPos", camera_position_vec3);// camera.Position);
    object_shader_ref.setMat4("projection", projection_m4);
    object_shader_ref.setMat4("view", view_m4);
    object_shader_ref.setMat4("model", light_m4);
}

void open_gl_setup()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Solar System", 0, 0);
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

void physics_setup()
{
    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
  	collisionConfiguration = new btDefaultCollisionConfiguration();
  	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  	dispatcher = new btCollisionDispatcher(collisionConfiguration);
  	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
  	overlappingPairCache = new btDbvtBroadphase();
  	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  	solver = new btSequentialImpulseConstraintSolver;
  	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    // dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void physics_destroy()
{
    //remove the rigidbodies from the dynamics world and delete them
  	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
  	{
    		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
    		btRigidBody* body = btRigidBody::upcast(obj);
    		if (body && body->getMotionState())
    		{
    			delete body->getMotionState();
    		}
    		dynamicsWorld->removeCollisionObject(obj);
    		delete obj;
  	}
  	//delete collision shapes
  	for (int j = 0; j < collision_shapes_btAlignedObjectArray.size(); j++)
  	{
    		btCollisionShape* shape = collision_shapes_btAlignedObjectArray[j];
    		collision_shapes_btAlignedObjectArray[j] = 0;
    		delete shape;
  	}
  	delete dynamicsWorld;
  	delete solver;
  	delete overlappingPairCache;
  	delete dispatcher;
  	delete collisionConfiguration;
  	//next line is optional: it will be cleared by the destructor when the array goes out of scope
  	collision_shapes_btAlignedObjectArray.clear();
    rigid_body_ptr_vector.clear();
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

btVector3 relative_gravity(btRigidBody *rigid_body_ptr)
{
    btTransform transform;
    rigid_body_ptr->getMotionState()->getWorldTransform(transform);
    btVector3 body_position = transform.getOrigin();
    btScalar body_mass = rigid_body_ptr->getMass();
    btVector3 relative_gravity = btVector3(0.f, 0.f, 0.f);
    // printf("size of rigid_body_ptr_vector: %d\n", rigid_body_ptr_vector.size());
    for(std::vector<const btRigidBody*>::iterator neighbor = rigid_body_ptr_vector.begin();
      neighbor < rigid_body_ptr_vector.end();
      neighbor++
    ) {
        const btRigidBody* neighbor_body = *neighbor;
        btTransform neighbor_transform;
        neighbor_body->getMotionState()->getWorldTransform(neighbor_transform);
        btVector3 neighbor_position = neighbor_transform.getOrigin();
        btScalar radius_squared = btDistance2(body_position, neighbor_position);
        if(radius_squared < 1.0f && radius_squared > -1.0f)
        {   continue;  }
        //Note: a radius ^ 2 greater than 66000 is not visible in render from the midpoint
        btScalar neighbor_mass = neighbor_body->getMass();
        btScalar gravity_btScalar = (GRAVITATIONAL_CONSTANT * body_mass * neighbor_mass) / radius_squared;
        btVector3 neighbor_gravity = (neighbor_position - body_position);
        // printf("body position(%f, %f, %f)\nneighbor position(%f, %f, %f)\nneighbor mass(%f)\nradius ^ 2(%f)\n",
        //     body_position.getX(), body_position.getY(), body_position.getZ(),
        //     neighbor_position.getX(), neighbor_position.getY(), neighbor_position.getZ(),
        //     neighbor_mass, radius_squared
        // );
        // printf("found neighbor_gravity of: x(%f), y(%f), z(%f)\ngravity scalar value: (%f)\n", neighbor_gravity.getX(), neighbor_gravity.getY(), neighbor_gravity.getZ(), gravity_btScalar);
        neighbor_gravity.normalize();
        neighbor_gravity = neighbor_gravity * gravity_btScalar;
        // printf("found final neighbor_gravity of: x(%f), y(%f), z(%f)\n", neighbor_gravity.getX(), neighbor_gravity.getY(), neighbor_gravity.getZ());
        relative_gravity += neighbor_gravity;
    }
    return relative_gravity;
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
