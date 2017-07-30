#include <iostream>
#include <vector>
#include <array>

#include "cnt_mesh.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

void cnt_mesh::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);
}

void cnt_mesh::create_container(int _half_Lx, int _half_Lz)
{

	// set some simulation inputs sizes and stuff
	// box is open in the y direction
	half_Lz = _half_Lz; // half the box size in the z direction
	half_Lx = _half_Lx; // half the box size in the x direction

	// the the volume of all the cnts and the average height of the cnts mesh to zero.
	volume = 0.;
	Ly = 0.;

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

	// create the z direction side wall planes
	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,-half_Lz));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}

	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, -1), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,half_Lz/2));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}


	// create the x direction side wall planes
	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(-half_Lx/2,0,0));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}

	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(-1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(half_Lx/2,0,0));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void cnt_mesh::add_tube(int _number_of_sections, float _section_length, float _diameter, std::array<float, 3> _drop_coordinate)
{
	tubes.push_back(tube(_number_of_sections, _section_length, _diameter));
	tube& _tube = tubes.back();

	// create a few dynamic rigidbodies
	//*********************************************************************************************
	
	// Re-using the same collision is better for memory usage and performance
	btCollisionShape* colShape;

	bool colShape_exists = false;
	for (int i=0; i<tube_section_collision_shapes.size(); i++)
	{
		if (tube_section_collision_shapes[i].equals(_section_length, _diameter))
		{
			colShape = tube_section_collision_shapes[i].colShape;
			colShape_exists = true;
			break;
		}
	}

	if (not colShape_exists)
	{
		colShape = new btCylinderShape(btVector3(_diameter,_section_length/2.0,1));
		m_collisionShapes.push_back(colShape);
		tube_section_collision_shapes.push_back(tube_section_collision_shape(_section_length,_diameter));
		tube_section_collision_shapes.back().colShape = colShape;
		std::cout << "new collision shape created!" << std::endl;
	}

	// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass,localInertia);
	 
	for(int i=0;i<_number_of_sections;++i)
	{
		btScalar x_loc = _drop_coordinate[0];
		btScalar y_loc = _drop_coordinate[1]+i*_section_length;
		btScalar z_loc = _drop_coordinate[2];
		btVector3 origin(x_loc, y_loc, z_loc);
		startTransform.setOrigin(origin);
		_tube.sections.push_back(createRigidBody(mass,startTransform,colShape));	// no static object
	} 
	 
	//add N-1 spring constraints
	for(int i=0;i<_tube.sections.size()-1;++i) {
		btRigidBody* b1 = _tube.sections[i];
		btRigidBody* b2 = _tube.sections[i+1];
		 
		btPoint2PointConstraint* centerSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0,1.1*_section_length/2.0,0), btVector3(0,-1.1*_section_length/2.0,0));
		centerSpring->m_setting.m_damping = 1.5; //the damping value for the constraint controls how stiff the constraint is. The default value is 1.0
		centerSpring->m_setting.m_impulseClamp = 0; //The m_impulseClamp value controls how quickly the dynamic rigid body comes to rest. The defual value is 0.0
		m_dynamicsWorld->addConstraint(centerSpring);
	}


	// generate the graphical representation of the object
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
	// update the average height of the mesh stack
	// volume = volume + 2*
}

void cnt_mesh::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}



