#include <iostream>

#include "Chain.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

const int TOTAL_BOXES = 10;

void ChainExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	// set some simulation inputs sizes and stuff
	// box is open in the y direction
	int Lz = 100; // box size in the z direction
	int Lx = 100; // box size in the x direction



	// create a few basic rigid bodies
	//**********************************************************************************************

	// create the bottom ground plane normal to the y axis
	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,0)); 
	
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}

	// // create the z direction side wall planes
	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
		
	// 	groundTransform.setOrigin(btVector3(0,0,Lz/2));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor

	// 	groundTransform.setOrigin(btVector3(0,0,-Lz/2));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }


	// // create the x direction side wall planes
	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
		
	// 	groundTransform.setOrigin(btVector3(Lx/2,0,0));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor

	// 	groundTransform.setOrigin(btVector3(Lx/2,0,0));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }
	


	// create a few dynamic rigidbodies
	//*********************************************************************************************
	{
		// Re-using the same collision is better for memory usage and performance
        // btBoxShape* colShape = createBoxShape(btVector3(1,1,1));

        btCollisionShape* colShape = new btCylinderShape(btVector3(1,0.5,1));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);
		 
		btAlignedObjectArray<btRigidBody*> boxes;
		int lastBoxIndex = TOTAL_BOXES-1;
		for(int i=0;i<TOTAL_BOXES;++i) {
			startTransform.setOrigin(btVector3(
									 btScalar(0),
									 btScalar(5+i*2),
									 btScalar(0)
									 )
									 );
			// boxes.push_back(createRigidBody((i==lastBoxIndex)?0:mass,startTransform,colShape)); // make the top object static
			boxes.push_back(createRigidBody(mass,startTransform,colShape));	// no static object
		} 
		 
		//add N-1 spring constraints
		for(int i=0;i<TOTAL_BOXES-1;++i) {
			btRigidBody* b1 = boxes[i];
			btRigidBody* b2 = boxes[i+1];
			 
			// btPoint2PointConstraint* leftSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5,1,0), btVector3(-0.5,-1,0));
			// leftSpring->m_setting.m_damping = 1.5; //the damping value for the constraint controls how stiff the constraint is. The default value is 1.0
			// leftSpring->m_setting.m_impulseClamp = 0; //The m_impulseClamp value controls how quickly the dynamic rigid body comes to rest. The defual value is 0.0
			// m_dynamicsWorld->addConstraint(leftSpring);

			// btPoint2PointConstraint* rightSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0.5,1,0), btVector3(0.5,-1,0));
			// rightSpring->m_setting.m_damping = 1.5; //the damping value for the constraint controls how stiff the constraint is. The default value is 1.0
			// rightSpring->m_setting.m_impulseClamp = 0; //The m_impulseClamp value controls how quickly the dynamic rigid body comes to rest. The defual value is 0.0
			// m_dynamicsWorld->addConstraint(rightSpring);

			btPoint2PointConstraint* centerSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0,0.5,0), btVector3(0,-0.5,0));
			centerSpring->m_setting.m_damping = 1.5; //the damping value for the constraint controls how stiff the constraint is. The default value is 1.0
			centerSpring->m_setting.m_impulseClamp = 0; //The m_impulseClamp value controls how quickly the dynamic rigid body comes to rest. The defual value is 0.0
			m_dynamicsWorld->addConstraint(centerSpring);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void ChainExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}



