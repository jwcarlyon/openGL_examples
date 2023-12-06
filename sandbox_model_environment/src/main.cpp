#include <chrono>
#include <iostream>
#include "../lib/bullet3/src/btBulletDynamicsCommon.h"
#include <glm/glm.hpp>

#include "Model_CLASS.hpp"
#include "OpenGL_config_CLASS.hpp"
#include "Physics_config_CLASS.hpp"
#include "KillAllThreadsSignal.h"
#include "ThreadPool.h"

void advance_time_in_fixed_timestep();
void hello_world();
void physics_process(
  KillAllThreadsSignal& killAllThreadsSignal,
  WireframeCollector& wireframeCollector_ref,
  ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
  ThreadSafe_matrix4& leftRearWheelPosition_mat4,
  TrackLayout& lowerCircle_trackLayout,
  ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
  ThreadSafe_matrix4& rightRearWheelPosition_mat4,
  TrackLayout& startingCircle_trackLayout,
  VehicleInterface& vehicleInterface,
  ThreadSafe_matrix4& vehiclePosition_mat4,
  Model& offCamber90_model,
  Model& onCamber90_model
);

namespace chr = std::chrono::_V2;//::steady_clock
using Framerate = std::chrono::duration<chr::steady_clock::rep, std::ratio<1, 60>>;
auto next_render_clock = chr::steady_clock::now() + Framerate{1};

int main()
{
    WireframeCollector wireframeCollector;
    OpenGL_config openGL_config;
    Model offCamber90_model(fs::u8path("../assets/autoDromo_Asset/90leftOffCamber.fbx")),
      onCamber90_model(fs::u8path("../assets/autoDromo_Asset/90leftOnCamber.fbx")),
      lightBulb_model(fs::u8path("../assets/lamp_Asset/bulb.obj")),
      testCar_model(fs::u8path("../assets/rangeRover_Asset/RangeRover2016.fbx")),
      wheelAssembly_model(fs::u8path("../assets/tire_Asset/tire_MODEL.fbx"));//Refactor!!!
    VehicleInterface vehicleInterface;

    TrackLayout lowerCircle_trackLayout;
    TrackLayout startingCircle_trackLayout;
    ThreadSafe_matrix4 vehiclePosition_mat4(glm::mat4(1.f));
    ThreadSafe_matrix4 leftFrontWheelPosition_mat4(glm::mat4(1.f));
    ThreadSafe_matrix4 rightFrontWheelPosition_mat4(glm::mat4(1.f));
    ThreadSafe_matrix4 leftRearWheelPosition_mat4(glm::mat4(1.f));
    ThreadSafe_matrix4 rightRearWheelPosition_mat4(glm::mat4(1.f));
    KillAllThreadsSignal killAllThreadsSignal;
    ThreadPool* worker_threadPoolPtr = new ThreadPool(2);


    Shader ground_shader("../assets/pavement_Asset/ground.vs", "../assets/pavement_Asset/ground.fs"),
      lightBulb_shader("../assets/lamp_Asset/lamp.vs", "../assets/lamp_Asset/lamp.fs"),
      vehicle_shader(
        "../assets/frank_grimes_deluxe_Asset/vehicle.vs",
        "../assets/frank_grimes_deluxe_Asset/vehicle.fs"
      ),
      wheelAssembly_shader("../assets/tire_Asset/tire.vs", "../assets/tire_Asset/tire.fs");

   SkyboxShader skyboxShader("../assets/skybox/SkyboxShader.vs", "../assets/skybox/SkyboxShader.fs");


        worker_threadPoolPtr->enqueue(
            physics_process,
            ref(killAllThreadsSignal),
            ref(wireframeCollector),
            ref(leftFrontWheelPosition_mat4),
            ref(leftRearWheelPosition_mat4),
            ref(lowerCircle_trackLayout),
            ref(rightFrontWheelPosition_mat4),
            ref(rightRearWheelPosition_mat4),
            ref(startingCircle_trackLayout),
            ref(vehicleInterface),
            ref(vehiclePosition_mat4),
            ref(offCamber90_model),
            ref(onCamber90_model)
          );
          while(!openGL_config.shouldWindowClose() && !killAllThreadsSignal.checkSignal())
          {
              openGL_config.updateWindow(vehicleInterface);
              openGL_config.updateMatrixArrays(
                leftFrontWheelPosition_mat4,
                leftRearWheelPosition_mat4,
                lowerCircle_trackLayout,
                rightFrontWheelPosition_mat4,
                rightRearWheelPosition_mat4,
                startingCircle_trackLayout,
                vehiclePosition_mat4
              );
              openGL_config.lowerCircleShaderUpdate(ground_shader, onCamber90_model);
              openGL_config.startingCircleShaderUpdate(ground_shader, offCamber90_model);
              openGL_config.lightBulbShaderUpdate(lightBulb_shader, lightBulb_model);
              openGL_config.vehicleShaderUpdate(
                vehicle_shader,
                testCar_model
              );
              openGL_config.wheelShaderUpdate(
                wheelAssembly_shader,
                wheelAssembly_model
              );
              openGL_config.skyboxShaderUpdate(skyboxShader);
          }
          killAllThreadsSignal.setSignal();

    return 0;
}

void advance_time_in_fixed_timestep()
{
    while(chr::steady_clock::now() < next_render_clock);
    //"busy" loop - chrono just observes a pre-set time window
    next_render_clock += Framerate{1}; //this is a singleton window value
}

void physics_process(
  KillAllThreadsSignal& killAllThreadsSignal,
  WireframeCollector& wireframeCollector_ref,
  ThreadSafe_matrix4& leftFrontWheelPosition_mat4,
  ThreadSafe_matrix4& leftRearWheelPosition_mat4,
  TrackLayout& lowerCircle_trackLayout,
  ThreadSafe_matrix4& rightFrontWheelPosition_mat4,
  ThreadSafe_matrix4& rightRearWheelPosition_mat4,
  TrackLayout& startingCircle_trackLayout,
  VehicleInterface& vehicleInterface,
  ThreadSafe_matrix4& vehiclePosition_mat4,
  Model& offCamber90_model,
  Model& onCamber90_model
) {
      WireframeCollector* wireframeCollector = &wireframeCollector_ref;
      Physics_config physics(wireframeCollector);
      physics.set90DegreeOffCamber(offCamber90_model.getMeshGeometry_synchronized());
      physics.set90DegreeOnCamber(onCamber90_model.getMeshGeometry_synchronized());//Refactor!!!
      physics.initPhysics();
      while(!killAllThreadsSignal.checkSignal())
      {
          hello_world();
          advance_time_in_fixed_timestep();
          physics.writeLowerCircleMatrices(lowerCircle_trackLayout);
          physics.writeStartingCircleMatrices(startingCircle_trackLayout);
          physics.writeVehicleMatrix(vehiclePosition_mat4);
          physics.writeWheelMatrices(
            leftFrontWheelPosition_mat4,
            rightFrontWheelPosition_mat4,
            leftRearWheelPosition_mat4,
            rightRearWheelPosition_mat4
          );
          physics.stepEngine(vehicleInterface);
      }
      physics.exitPhysics();
}


void hello_world()
{
    std::cout << "Physics update : hello world" << std::endl;
}

