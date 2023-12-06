#ifndef PHYSICS_CONFIG_HEADER
#define PHYSICS_CONFIG_HEADER

#include <glm/glm.hpp>
#include <vector>

#include "../lib/bullet3/src/btBulletDynamicsCommon.h"
#include "../lib/bullet3/src/btBulletDynamicsCommon.h"
#include "../lib/bullet3/examples/CommonInterfaces/CommonRigidBodyBase.h"
#include "../lib/bullet3/src/BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "../lib/bullet3/src/BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "../lib/bullet3/src/BulletCollision/CollisionShapes/btTriangleShape.h"
#include "../lib/bullet3/src/BulletCollision/CollisionDispatch/btCollisionWorld.h"

#include "Mesh_CLASS.hpp"
#include "Shader_CLASS.hpp"
#include "TrackLayout.h"
#include "VehicleInterface.h"
#include "WireframeCollector.h"

#define BT_LINE_BATCH_SIZE 512

// struct WheelContactResultCallback : btCollisionWorld::ContactResultCallback
// {
//     WheelContactResultCallback() {}
//
//     static bool myCustomMaterialCombinerCallback(
//       btManifoldPoint& cp,
//       const btCollisionObjectWrapper* colObj0Wrap,
//       int partId0,
//       int index0,
//       const btCollisionObjectWrapper* colObj1Wrap,
//       int partId1,
//       int index1
//     ) {
//       	if (colObj1Wrap->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
//         {// one-sided triangles ..huh?
//               const btTriangleShape* triShape = static_cast<const btTriangleShape*>(colObj1Wrap->getCollisionShape());
//               const btVector3* triangleVertex = triShape->m_vertices1;
//               btVector3 faceNormalLs = btCross(triangleVertex[1] - triangleVertex[0], triangleVertex[2] - triangleVertex[0]);
//               faceNormalLs.normalize();
//               btVector3 faceNormalWs = colObj1Wrap->getWorldTransform().getBasis() * faceNormalLs;
//               float nDotF = btDot(faceNormalWs, cp.m_normalWorldOnB);
//               if(nDotF <= 0.0f)
//               {
//                   // flip the contact normal to be aligned with the face normal
//                   cp.m_normalWorldOnB += -2.0f * nDotF * faceNormalWs;
//               }
//         }
//         //this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
//         return false;
//     }
// };

struct SimulationContactResultCallback : public btCollisionWorld::ContactResultCallback
{

    bool bCollision;

    SimulationContactResultCallback() : bCollision(false)
    {}

    virtual btScalar addSingleResult(
      btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,
      int partId0,
      int index0,
      const btCollisionObjectWrapper* colObj1Wrap,
      int partId1,
      int index1
    );
};


struct MyDebugVec3
{
  	MyDebugVec3(const btVector3& org) : x(org.x()), y(org.y()), z(org.z())
  	{
  	}

  	float x, y, z;
};
///A class that implements the btIDebugDraw interface will need to provide non-empty
// implementations of the the drawLine and getDebugMode methods at a minimum.

// ATTRIBUTE_ALIGNED16(class) MyDebugDrawer : public btIDebugDraw
class MyDebugDrawer :public::btIDebugDraw
{
public:
    int m_debugMode;
  	WireframeCollector* m_glApp;
  	std::vector<MyDebugVec3> m_linePoints;
  	std::vector<unsigned int> m_lineIndices;

  	btVector3 m_currentLineColor;
  	DefaultColors m_ourColors;

  	BT_DECLARE_ALIGNED_ALLOCATOR();

    MyDebugDrawer(WireframeCollector* wireframeCollector);
  	// MyDebugDrawer() {};

    void drawLine(const btVector3& from1, const btVector3& to1, const btVector3& color1);

  	virtual ~MyDebugDrawer()
  	{
  	}
  	virtual DefaultColors getDefaultColors() const;
  	///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
  	virtual void setDefaultColors(const DefaultColors& colors);

  	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);

  	virtual void reportErrorWarning(const char* warningString);

  	virtual void draw3dText(const btVector3& location, const char* textString);

  	virtual void setDebugMode(int debugMode);

  	virtual int getDebugMode() const;

  	virtual void flushLines();
};

class Physics_config :public::CommonRigidBodyBase
{
public:
    Physics_config(WireframeCollector* wireframeCollector);

    btCollisionShape* autoDromo_btCollisionShape;
    btCollisionShape* ninetyDegreeOffCamber_btCollisionShape;
    btCollisionShape* ninetyDegreeOnCamber_btCollisionShape;
    btRigidBody* carChassis_btRigidBody;
    btRigidBody* groundRigidBody;
    btTransform startTransform;
    btRaycastVehicle::btVehicleTuning tuning;
    // note this solution to derive the btRaycastVehicle class and include the desired ray testing
    //https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=9&t=5494&start=0&hilit=raycast+vehicle+collision+masks
    btRaycastVehicle* vehicle_btRaycastVehicle;
    btVehicleRaycaster* vehicleRayCaster;
    btVector3* vertices;
    btCollisionShape* wheel_btCollisionShape;
    int wheels_ptrArray[4];
    SimulationContactResultCallback wheelContactResultCallback;

    void clientResetScene();
    void createVehicleInterface();
    virtual void initPhysics();
    void initPlatformerLevelPhysics();
    btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* collisionShape);
    void set90DegreeOffCamber(Mesh& mesh);
    void set90DegreeOnCamber(Mesh& mesh);
    void setAutoDromoModel(Mesh& mesh);
    void specialKeyboard(int key, int x, int y);
    void specialKeyboardUp(int key, int x, int y);
    void stepEngine(VehicleInterface& vehicleInterface_ref);
    void stepSimulation(float deltaTime);
    void vehicleInput(VehicleInterface& vehicleInterface_ref);
    void writeAutoDromoMatrices(TrackLayout& positionMatrices);
    void writeLowerCircleMatrices(TrackLayout& positionMatrices);
    void writeStartingCircleMatrices(TrackLayout& positionMatrices);
    void writeGroundMatrix(ThreadSafe_matrix4& position_matrixRef);
    void writeVehicleMatrix(ThreadSafe_matrix4& positionMatrix_ptr);
    void writeWheelMatrices (
      ThreadSafe_matrix4& leftFront_matrixRef,
      ThreadSafe_matrix4& rightFront_matrixRef,
      ThreadSafe_matrix4& leftRear_matrixRef,
      ThreadSafe_matrix4& rightRear_matrixRef
    );
private:
    MyDebugDrawer* myDebugDrawer;

    btTriangleMesh* autoDromoModel_btTriangleMeshPtr;
    btTriangleMesh* ninetyDegreeOffCamber_btTriangleMeshPtr;
    btTriangleMesh* ninetyDegreeOnCamber_btTriangleMeshPtr;
    VehicleInterface vehicleInterface;
    btTransform vehiclePosition_btTransform;
    float wheelRadius_fl = 0.5f;
    float wheelWidth_fl = 0.4f;
    btRigidBody* groundRigidBodyPtrArray[4];
    btRigidBody* startingCircle_RigidBodyPtrArray[4];
    btRigidBody* lowerCircle_RigidBodyPtrArray[4];

    bool useMCLPSolver = true;

    void createLowerCircleTrackLayout();
    void createStartingCircleTrackLayout();
    void createTrackLayout();
    bool isSteeringReversed(int steeringInput);
    void resetVehicle();
};
void createSupplementalCompoundShape(btCompoundShape* compound);

#endif
