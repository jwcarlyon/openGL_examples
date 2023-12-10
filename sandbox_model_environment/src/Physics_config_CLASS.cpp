#include <iostream>

#include "Physics_config_CLASS.hpp"

static float defaultBreakingForce_fl = 10.f;
static float gBreakingForce_fl = 100.f;
static float gEngineForce_fl = 0.f;
static float maxEngineForce_fl = 1000.f;  //this should be engine/velocity dependent
static float gVehicleSteering_fl = 0.f;
static float steeringClamp = 0.3f;
static float steeringIncrement_fl = 0.04f;
float rollInfluence_fl = 1.0f;//0.1f;
float suspensionCompression_fl = 4.4f;
float suspensionDamping_fl = 2.3f;//2.3f
float suspensionStiffness_fl = 20.f;
float wheelFriction_fl = 1000.f;  //BT_LARGE_FLOAT 1000.f
float wheelRadius_fl = 0.75f;
float wheelWidth_fl = 0.4f;
btScalar suspensionRestLength(.6f);//.6f;


#define CUBE_HALF_EXTENTS 1

btScalar SimulationContactResultCallback::addSingleResult(
  btManifoldPoint& cp,
  const btCollisionObjectWrapper* colObj0Wrap,
  int partId0,
  int index0,
  const btCollisionObjectWrapper* colObj1Wrap,
  int partId1,
  int index1
)
{cd
    //If cp distance less than threshold
    //bCollision = true //given from user as examples
    if (colObj1Wrap->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
    {// one-sided triangles ..huh?
          const btTriangleShape* triShape = static_cast<const btTriangleShape*>(colObj1Wrap->getCollisionShape());
          const btVector3* triangleVertex = triShape->m_vertices1;
          btVector3 faceNormalLs = btCross(triangleVertex[1] - triangleVertex[0], triangleVertex[2] - triangleVertex[0]);
          faceNormalLs.normalize();
          btVector3 faceNormalWs = colObj1Wrap->getWorldTransform().getBasis() * faceNormalLs;
          float nDotF = btDot(faceNormalWs, cp.m_normalWorldOnB);
          cout << "SimulationContactResultCallback::addSingleResult" << std::endl;
          if(nDotF <= 0.0f)
          {
              // flip the contact normal to be aligned with the face normal
              cp.m_normalWorldOnB += -2.0f * nDotF * faceNormalWs;
              cout << "SimulationContactResultCallback::addSingleResult revesed normals" << std::endl;
          }
    }
    //this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
    return false;
}

MyDebugDrawer::MyDebugDrawer(WireframeCollector* wireframeCollector)
  : m_glApp(wireframeCollector),
  m_debugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb),
  m_currentLineColor(-1, -1, -1) {
    setDebugMode(0);
  }

MyDebugDrawer::DefaultColors MyDebugDrawer::getDefaultColors() const
{
    return m_ourColors;
}

void MyDebugDrawer::setDefaultColors(const DefaultColors& colors)
{
    m_ourColors = colors;
}

void MyDebugDrawer::drawLine(const btVector3& from1, const btVector3& to1, const btVector3& color1)
{
    if(m_currentLineColor != color1 || m_linePoints.size() >= BT_LINE_BATCH_SIZE)
    {
        flushLines();
        m_currentLineColor = color1;
    }
    MyDebugVec3 from(from1);
    MyDebugVec3 to(to1);
    m_linePoints.push_back(from);
    m_linePoints.push_back(to);
}

void MyDebugDrawer::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
{
    drawLine(PointOnB, PointOnB + normalOnB * distance, color);
    btVector3 ncolor(0, 0, 0);
    drawLine(PointOnB, PointOnB + normalOnB * 0.01, ncolor);
}

void MyDebugDrawer::reportErrorWarning(const char* warningString)
{}

void MyDebugDrawer::draw3dText(const btVector3& location, const char* textString)
{}

void MyDebugDrawer::setDebugMode(int debugMode)
{ m_debugMode = debugMode; }

int MyDebugDrawer::getDebugMode() const
{ return m_debugMode; }

void MyDebugDrawer::flushLines()
{
    int sizeOfLineCoordinatesArray = m_linePoints.size() * 3;
    if(sizeOfLineCoordinatesArray > 2)
    {
          float debugColor[4];
          debugColor[0] = m_currentLineColor.x();
          debugColor[1] = m_currentLineColor.y();
          debugColor[2] = m_currentLineColor.z();
          debugColor[3] = 1.f;
          m_glApp->drawLines(//call save the wireframe points on the main thread, for the openGL thread
            &m_linePoints[0].x,
            sizeOfLineCoordinatesArray,
            &debugColor[0]
          );
          m_linePoints.clear();
          m_lineIndices.clear();
    }
}

Physics_config::Physics_config(WireframeCollector* wireframeCollector)
  : CommonRigidBodyBase(0) {
    myDebugDrawer = new MyDebugDrawer(wireframeCollector);
}

void Physics_config::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

void createSupplementalCompoundShape(btCompoundShape* compound)
{
		btCollisionShape* supplementalShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
		btTransform supplementalLocalTransform;
		supplementalLocalTransform.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		supplementalLocalTransform.setOrigin(btVector3(0, 1.0, 2.5));
		compound->addChildShape(supplementalLocalTransform, supplementalShape);
}

void Physics_config::createLowerCircleTrackLayout()
{
    int twentyFour_int = 24;
    int thirty_int = 30;
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, -13, 30));
    lowerCircle_RigidBodyPtrArray[0] = localCreateRigidBody(
      0, transform, ninetyDegreeOnCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(73, -13, 11));
    // transform.setOrigin(btVector3(thirty_int, -13, 54));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), (M_PI / 2)));
    lowerCircle_RigidBodyPtrArray[1] = localCreateRigidBody(
      0, transform, ninetyDegreeOnCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(54, -13, -60));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), M_PI));
    lowerCircle_RigidBodyPtrArray[2] = localCreateRigidBody(
      0, transform, ninetyDegreeOnCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(-19, -13, -41));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), (M_PI / -2)));
    lowerCircle_RigidBodyPtrArray[3] = localCreateRigidBody(
      0, transform, ninetyDegreeOnCamber_btCollisionShape
    );
}

void Physics_config::createStartingCircleTrackLayout()
{
    int twentyFour_int = 24;
    int thirty_int = 30;
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, -3, 0));
    startingCircle_RigidBodyPtrArray[0] = localCreateRigidBody(
      0, transform, ninetyDegreeOffCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(thirty_int, -3, twentyFour_int));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), (M_PI / 2)));
    startingCircle_RigidBodyPtrArray[1] = localCreateRigidBody(
      0, transform, ninetyDegreeOffCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(54, -3, -5));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), (M_PI)));
    startingCircle_RigidBodyPtrArray[2] = localCreateRigidBody(
      0, transform, ninetyDegreeOffCamber_btCollisionShape
    );
    transform.setIdentity();
    transform.setOrigin(btVector3(twentyFour_int, -3, -29));
    transform.setRotation(btQuaternion(btVector3(0, 1, 0), (M_PI * 1.5f)));
    startingCircle_RigidBodyPtrArray[3] = localCreateRigidBody(
      0, transform, ninetyDegreeOffCamber_btCollisionShape
    );
}

void Physics_config::createVehicleInterface()
{
		vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		vehicle_btRaycastVehicle = new btRaycastVehicle(tuning, carChassis_btRigidBody, vehicleRayCaster);

		//never deactivate the vehicle
		carChassis_btRigidBody->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(vehicle_btRaycastVehicle);

		float connectionHeight = 1.2f;

		bool isFrontWheel = true;
    int rightIndex = 0;
    int upIndex = 1;
    int forwardIndex = 2;
    btVector3 wheelDirectionCS0(0, -1, 0);
    btVector3 wheelAxleCS(-1, 0, 0);
    btScalar wheelBaseLength = 2 * CUBE_HALF_EXTENTS - (wheelRadius_fl) + .11f;
    // btScalar wheelBaseLength = 2 * CUBE_HALF_EXTENTS - wheelRadius_fl;//frank_grimes_deluxe
		//choose coordinate system
		vehicle_btRaycastVehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3f * wheelWidth_fl), connectionHeight, wheelBaseLength);
    vehicle_btRaycastVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius_fl, tuning, isFrontWheel);

		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth_fl), connectionHeight, wheelBaseLength);
    vehicle_btRaycastVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius_fl, tuning, isFrontWheel);

    isFrontWheel = false;
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth_fl), connectionHeight, -1 * wheelBaseLength);
		vehicle_btRaycastVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius_fl, tuning, isFrontWheel);

    connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3f * wheelWidth_fl), connectionHeight, -1 * wheelBaseLength);
		vehicle_btRaycastVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius_fl, tuning, isFrontWheel);

		for(int i = 0; i < vehicle_btRaycastVehicle->getNumWheels(); i++)
		{
  			btWheelInfo& wheel = vehicle_btRaycastVehicle->getWheelInfo(i);
  			wheel.m_suspensionStiffness = suspensionStiffness_fl;
  			wheel.m_wheelsDampingRelaxation = suspensionDamping_fl;
  			wheel.m_wheelsDampingCompression = suspensionCompression_fl;
  			wheel.m_frictionSlip = wheelFriction_fl;
  			wheel.m_rollInfluence = rollInfluence_fl;
		}
}

void Physics_config::initPhysics()
{
    // m_guiHelper->setUpAxis(1);
    wheelContactResultCallback = SimulationContactResultCallback();
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldMin(-1000, -1000, -1000);
  	btVector3 worldMax(1000, 1000, 1000);
  	m_broadphase = new btAxisSweep3(worldMin, worldMax);
    if(useMCLPSolver)
  	{
    		btDantzigSolver* mlcp = new btDantzigSolver();
    		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
    		btMLCPSolver* solver_ptr = new btMLCPSolver(mlcp);
    		m_solver = solver_ptr;
  	} else {
    		m_solver = new btSequentialImpulseConstraintSolver();
  	}
  	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
  	if(useMCLPSolver)
  	{
	      m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;  //for direct solver it is better to have a small A matrix
  	} else {
    		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
  	}
    m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
    // m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001f;

    m_dynamicsWorld->setDebugDrawer(myDebugDrawer);
    ninetyDegreeOnCamber_btCollisionShape = new btBvhTriangleMeshShape(
      ninetyDegreeOnCamber_btTriangleMeshPtr,
      true,
      true
    );
    m_collisionShapes.push_back(ninetyDegreeOnCamber_btCollisionShape);

    ninetyDegreeOffCamber_btCollisionShape = new btBvhTriangleMeshShape(
      ninetyDegreeOffCamber_btTriangleMeshPtr,
      true,
      true
    );
    m_collisionShapes.push_back(ninetyDegreeOffCamber_btCollisionShape);

    btTransform transform;
  	transform.setIdentity();
  	transform.setOrigin(btVector3(0, -3, 0));
    createStartingCircleTrackLayout();
    createLowerCircleTrackLayout();

  	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
  	m_collisionShapes.push_back(chassisShape);
    btCompoundShape* compound = new btCompoundShape();
  	m_collisionShapes.push_back(compound);
  	btTransform localTrans;
  	localTrans.setIdentity();
  	//localTrans effectively shifts the center of mass with respect to the chassis
  	localTrans.setOrigin(btVector3(0, 1, 0));

  	compound->addChildShape(localTrans, chassisShape);
    createSupplementalCompoundShape(compound);

    const btScalar FALLHEIGHT = 25;
  	const btScalar chassisMass = 100.0f;//4.f
    transform.setOrigin(btVector3(0.f, FALLHEIGHT, 0.f));

  	carChassis_btRigidBody = localCreateRigidBody(chassisMass, transform, compound);

    wheel_btCollisionShape = new btCylinderShapeX(btVector3(wheelWidth_fl, wheelRadius_fl, wheelRadius_fl));
    //Here is where the vehicle interface must be implemented
    createVehicleInterface();
    resetVehicle();
}

bool Physics_config::isSteeringReversed(int steeringInput)
{
      bool isWheelTurned, isInputPositive, isSteeringPositive;
      isWheelTurned = (std::abs(gVehicleSteering_fl) > .25f);
      isInputPositive = (steeringInput > 0);
      isSteeringPositive = (gVehicleSteering_fl >= 0.f);
      // std::cout << "Steering Reversed: " <<
      //   ((isInputPositive != isSteeringPositive) && isWheelTurned) << std::endl;
      return ((isInputPositive != isSteeringPositive) && isWheelTurned);
}

btRigidBody* Physics_config::localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* collisionShape)
{
    btAssert((!collisionShape || collisionShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if(isDynamic)
    {
    		collisionShape->calculateLocalInertia(mass, localInertia);
    }

//using motionstate is recommended, it provides interpolation capabilities,
// and only synchronizes 'active' objects
#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    	btDefaultMotionState* myMotionState = new btDefaultMotionState(worldTransform);

    	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, collisionShape, localInertia);

    	btRigidBody* body = new btRigidBody(cInfo);    	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
#else
    	btRigidBody* body = new btRigidBody(mass, 0, collisionShape, localInertia);
    	body->setWorldTransform(worldTransform);
#endif
    	m_dynamicsWorld->addRigidBody(body);
    	return body;
}

void Physics_config::writeAutoDromoMatrices(TrackLayout& positionMatrices)
{
    glm::mat4 matrixHolder(1.f);
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        groundRigidBodyPtrArray[0]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnOne.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        groundRigidBodyPtrArray[1]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnTwo.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        groundRigidBodyPtrArray[2]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnThree.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        groundRigidBodyPtrArray[3]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnFour.set(matrixHolder);
    }
}

void Physics_config::writeLowerCircleMatrices(TrackLayout& positionMatrices)
{
    glm::mat4 matrixHolder(1.f);
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        lowerCircle_RigidBodyPtrArray[0]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnOne.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        lowerCircle_RigidBodyPtrArray[1]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnTwo.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        lowerCircle_RigidBodyPtrArray[2]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnThree.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        lowerCircle_RigidBodyPtrArray[3]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnFour.set(matrixHolder);
    }
}

void Physics_config::writeStartingCircleMatrices(TrackLayout& positionMatrices)
{
    glm::mat4 matrixHolder(1.f);
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        startingCircle_RigidBodyPtrArray[0]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnOne.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        startingCircle_RigidBodyPtrArray[1]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnTwo.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        startingCircle_RigidBodyPtrArray[2]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnThree.set(matrixHolder);
    }
    {
        btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
        startingCircle_RigidBodyPtrArray[3]->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
        positionMatrices.turnFour.set(matrixHolder);
    }
}

void Physics_config::writeGroundMatrix(ThreadSafe_matrix4& position_matrixRef)
{
    glm::mat4 matrixHolder(1.f);
    btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
    groundRigidBody->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
    position_matrixRef.set(matrixHolder);
    // btScalar* matrixAddress_btScalar = (float*) positionMatrix_ptr;
    // groundRigidBody->getWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
}

void Physics_config::writeVehicleMatrix(ThreadSafe_matrix4& position_matrixRef)
{
    glm::mat4 matrixHolder(1.f);
    btScalar* matrixAddress_btScalar = (float*) &matrixHolder;
    vehicle_btRaycastVehicle->getChassisWorldTransform().getOpenGLMatrix(matrixAddress_btScalar);
    position_matrixRef.set(matrixHolder);
}

void Physics_config::writeWheelMatrices (
  ThreadSafe_matrix4& leftFront_matrixRef,
  ThreadSafe_matrix4& rightFront_matrixRef,
  ThreadSafe_matrix4& leftRear_matrixRef,
  ThreadSafe_matrix4& rightRear_matrixRef
) {
    glm::mat4 matrixHolder(1.f);
    btScalar* matrixAddress_btScalar = (float*) &matrixHolder;

    vehicle_btRaycastVehicle->getWheelTransformWS(0).getOpenGLMatrix(matrixAddress_btScalar);
    leftFront_matrixRef.set(matrixHolder);

    vehicle_btRaycastVehicle->getWheelTransformWS(1).getOpenGLMatrix(matrixAddress_btScalar);
    rightFront_matrixRef.set(matrixHolder);

    vehicle_btRaycastVehicle->getWheelTransformWS(2).getOpenGLMatrix(matrixAddress_btScalar);
    leftRear_matrixRef.set(matrixHolder);

    vehicle_btRaycastVehicle->getWheelTransformWS(3).getOpenGLMatrix(matrixAddress_btScalar);
    rightRear_matrixRef.set(matrixHolder);
}

void Physics_config::resetVehicle()
{
    gVehicleSteering_fl = 0.f;
  	gBreakingForce_fl = defaultBreakingForce_fl;
  	gEngineForce_fl = 0.f;

  	carChassis_btRigidBody->setCenterOfMassTransform(btTransform::getIdentity());
  	carChassis_btRigidBody->setLinearVelocity(btVector3(0, 0, 0));
  	carChassis_btRigidBody->setAngularVelocity(btVector3(0, 0, 0));
  	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
        carChassis_btRigidBody->getBroadphaseHandle(),
        getDynamicsWorld()->getDispatcher()
    );

    if(vehicle_btRaycastVehicle)
  	{
    		vehicle_btRaycastVehicle->resetSuspension();
    		for (int i = 0; i < vehicle_btRaycastVehicle->getNumWheels(); i++)
    		{
    			   //synchronize the wheels with the (interpolated) chassis worldtransform
    			   vehicle_btRaycastVehicle->updateWheelTransform(i, true);
    		}
  	}

}

void Physics_config::set90DegreeOffCamber(Mesh& mesh)
{
    std::vector <btVector3> vertices;
    std::vector <unsigned int> indices;
    for(int i = 0; i < mesh.vertices.size(); i++)
    {
        glm::vec3 vertex = mesh.vertices.at(i).Position;
        btVector3 newVertex(vertex.x, vertex.y, vertex.z);
        vertices.push_back(newVertex);
        indices.push_back(i);
    }
    ninetyDegreeOffCamber_btTriangleMeshPtr = new btTriangleMesh();
    for(int i = 0; (i * 3 + 2) < indices.size(); i++) {
        int index0 = indices.at(i * 3);
        int index1 = indices.at(i * 3 + 1);
        int index2 = indices.at(i * 3 + 2);

        btVector3 vertex0(vertices.at(index0).getX(), vertices.at(index0).getY(), vertices.at(index0).getZ());
        btVector3 vertex1(vertices.at(index1).getX(), vertices.at(index1).getY(), vertices.at(index1).getZ());
        btVector3 vertex2(vertices.at(index2).getX(), vertices.at(index2).getY(), vertices.at(index2).getZ());

        ninetyDegreeOffCamber_btTriangleMeshPtr->addTriangle(vertex0, vertex1, vertex2);
    }
    ninetyDegreeOffCamber_btTriangleMeshPtr->setScaling(btVector3(10.f, 10.f, 10.f));
}


void Physics_config::set90DegreeOnCamber(Mesh& mesh)
{
    std::vector <btVector3> vertices;
    std::vector <unsigned int> indices;
    for(int i = 0; i < mesh.vertices.size(); i++)
    {
        glm::vec3 vertex = mesh.vertices.at(i).Position;
        btVector3 newVertex(vertex.x, vertex.y, vertex.z);
        vertices.push_back(newVertex);
        indices.push_back(i);
    }
    ninetyDegreeOnCamber_btTriangleMeshPtr = new btTriangleMesh();
    for(int i = 0; (i * 3 + 2) < indices.size(); i++) {
        int index0 = indices.at(i * 3);
        int index1 = indices.at(i * 3 + 1);
        int index2 = indices.at(i * 3 + 2);

        btVector3 vertex0(vertices.at(index0).getX(), vertices.at(index0).getY(), vertices.at(index0).getZ());
        btVector3 vertex1(vertices.at(index1).getX(), vertices.at(index1).getY(), vertices.at(index1).getZ());
        btVector3 vertex2(vertices.at(index2).getX(), vertices.at(index2).getY(), vertices.at(index2).getZ());

        ninetyDegreeOnCamber_btTriangleMeshPtr->addTriangle(vertex0, vertex1, vertex2);
    }
    ninetyDegreeOnCamber_btTriangleMeshPtr->setScaling(btVector3(10.f, 10.f, 10.f));
}

void Physics_config::setAutoDromoModel(Mesh& mesh)
{
    // This is a routine that creates
    std::vector <btVector3> vertices;
    std::vector <unsigned int> indices;
    for(int i = 0; i < mesh.vertices.size(); i++)
    {
        glm::vec3 vertex = mesh.vertices.at(i).Position;
        btVector3 newVertex(vertex.x, vertex.y, vertex.z);
        vertices.push_back(newVertex);
        indices.push_back(i);
    }
    autoDromoModel_btTriangleMeshPtr = new btTriangleMesh();
    for(int i = 0; (i * 3 + 2) < indices.size(); i++) {
        int index0 = indices.at(i * 3);
        int index1 = indices.at(i * 3 + 1);
        int index2 = indices.at(i * 3 + 2);

        btVector3 vertex0(vertices.at(index0).getX(), vertices.at(index0).getY(), vertices.at(index0).getZ());
        btVector3 vertex1(vertices.at(index1).getX(), vertices.at(index1).getY(), vertices.at(index1).getZ());
        btVector3 vertex2(vertices.at(index2).getX(), vertices.at(index2).getY(), vertices.at(index2).getZ());

        autoDromoModel_btTriangleMeshPtr->addTriangle(vertex0, vertex1, vertex2);
    }
    // btVector3 autoDromoScaling(10.f, 10.f, 10.f);
    autoDromoModel_btTriangleMeshPtr->setScaling(btVector3(10.f, 10.f, 10.f));
    // btConvexShape *tmpshape = new btConvexTriangleMeshShape(autoDromoModel_btTriangleMeshPtr);
    // btShapeHull *hull = new btShapeHull(tmpshape);
    // btScalar margin = tmpshape->getMargin();
    // hull->buildHull(margin);
    // tmpshape->setUserPointer(hull);
}

void Physics_config::specialKeyboard(int key, int x, int y) { }
void Physics_config::specialKeyboardUp(int key, int x, int y) { }

void Physics_config::stepEngine(VehicleInterface& vehicleInterface_ref)
{
    vehicleInput(vehicleInterface_ref);
    stepSimulation(1 / 60.f);
    m_dynamicsWorld->debugDrawWorld();
}

void Physics_config::stepSimulation(float deltaTime)
{
  	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  	float dt = deltaTime;

  	if(m_dynamicsWorld)
  	{
    		//during idle mode, just run 1 simulation step maximum
    		int maxSimSubSteps = 1;
    		// int maxSimSubSteps = 2;

    		int numSimSteps;
            numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

    		if(m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
    		{
      			btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
      			int numFallbacks = sol->getNumFallbacks();
      			if(numFallbacks)
      			{
        				static int totalFailures = 0;
        				totalFailures += numFallbacks;
        				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
      			}
      			sol->setNumFallbacks(0);
    		}

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
  		if(!numSimSteps){
  			 printf("Interpolated transforms\n");
  		} else {
    			if(numSimSteps > maxSimSubSteps)
    			{//detect dropping frames
    				    printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
    			} else {
    				    printf("Simulated (%i) steps\n", numSimSteps);
    			}
  		}
#endif  //VERBOSE_FEEDBACK
	}
}

void Physics_config::vehicleInput(VehicleInterface& vehicleInterface_ref)
{
    // std::cout << "Physics config CLASS - vehicle speed in kph: "
    //   << vehicle_btRaycastVehicle->getCurrentSpeedKmHour() << std::endl
    //   << "Gas, Brake, Wheel(" << gasPedal << ", "
    //   << brakePedal << ", " << steeringWheel
    //   << ")" << std::endl << "SteeringValue: " << gVehicleSteering_fl << std::endl;
    bool brakePedal = vehicleInterface_ref.getBrakePedal();
    bool gasPedal = vehicleInterface_ref.getGasPedal();
    int steeringWheel = vehicleInterface_ref.getSteeringWheel();
    if(steeringWheel == 0 || isSteeringReversed(steeringWheel))
    {
        gVehicleSteering_fl = (std::abs(gVehicleSteering_fl) > steeringIncrement_fl) ?
          (gVehicleSteering_fl / 1.166f) : 0.f;
          // std::cout << "Steering revert." << std::endl;
    } else {
        if((steeringWheel == 1) && (gVehicleSteering_fl <= 1.f))
        {
            gVehicleSteering_fl += steeringIncrement_fl;
        }
        if((steeringWheel == -1) && (gVehicleSteering_fl >= -1.f))
        {
            gVehicleSteering_fl -= steeringIncrement_fl;
        }
    }
    if(brakePedal)
  	{
        if(gBreakingForce_fl < 1000)
        {
            gBreakingForce_fl += 5.f;
        }
    } else {
        gBreakingForce_fl = 0.f;
    }
    if(gasPedal){
        gEngineForce_fl += (gEngineForce_fl < maxEngineForce_fl) ? 50.f : 0.f;
    } else {
        gEngineForce_fl = 0.f;
    }
    int wheelIndex = 2;
    vehicle_btRaycastVehicle->applyEngineForce(gEngineForce_fl, wheelIndex);
    vehicle_btRaycastVehicle->setBrake(gBreakingForce_fl, wheelIndex);
    wheelIndex = 3;
    vehicle_btRaycastVehicle->applyEngineForce(gEngineForce_fl, wheelIndex);
    vehicle_btRaycastVehicle->setBrake(gBreakingForce_fl, wheelIndex);
    wheelIndex = 0;
    vehicle_btRaycastVehicle->setSteeringValue(gVehicleSteering_fl, wheelIndex);
    wheelIndex = 1;
    vehicle_btRaycastVehicle->setSteeringValue(gVehicleSteering_fl, wheelIndex);
}
