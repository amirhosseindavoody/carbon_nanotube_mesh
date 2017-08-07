#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <experimental/filesystem>

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
		groundTransform.setOrigin(btVector3(0,0,half_Lz));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}


	// create the x direction side wall planes
	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(-half_Lx,0,0));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}

	{
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(-1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
		m_collisionShapes.push_back(groundShape);
		
		btScalar mass(0.);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(half_Lx,0,0));	
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void cnt_mesh::add_tube(int _number_of_sections, float _section_length, float _diameter)
{
	tubes.push_back(tube(_number_of_sections, _section_length, _diameter));
	tube& _tube = tubes.back();

	std::array<float, 3> _drop_coordinate = drop_coordinate();

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
		btScalar y_loc = _drop_coordinate[1]+(i+0.5)*_section_length;
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
		_tube.springs.push_back(centerSpring);
	}


	// generate the graphical representation of the object
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
	// update the average height of the mesh stack
	volume = volume + 2*acos(-1)*_diameter*_section_length*float(_number_of_sections);
}

// INCOMPLETE: this method creates a new static body so that we can remove unecessary static CNTs.
void cnt_mesh::create_large_mesh_body()
{
	// btTransform trans;
	// trans.setIdentity();

	// for(int i=0;i<8;i++) {

	// 	btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
	// 	btIndexedMesh part;

	// 	part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
	// 	part.m_vertexStride = sizeof(btScalar) * 3;
	// 	part.m_numVertices = LandscapeVtxCount[i];
	// 	part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
	// 	part.m_triangleIndexStride = sizeof( short) * 3;
	// 	part.m_numTriangles = LandscapeIdxCount[i]/3;
	// 	part.m_indexType = PHY_SHORT;

	// 	meshInterface->addIndexedMesh(part,PHY_SHORT);

	// 	bool	useQuantizedAabbCompression = true;
	// 	btBvhTriangleMeshShape* trimeshShape = new btBvhTriangleMeshShape(meshInterface,useQuantizedAabbCompression);
	// 	btVector3 localInertia(0,0,0);
	// 	trans.setOrigin(btVector3(0,-25,0));

	// 	btRigidBody* body = createRigidBody(0,trans,trimeshShape);
	// 	body->setFriction (btScalar(0.9));
		
	// }
	
}

// make tubes static in the simulation and only leave _number_of_active_tubes as dynamic in the simulation.
void cnt_mesh::freeze_tube(int _number_of_active_tubes)
{
	int n = num_tubes() - _number_of_active_tubes;
	for (int i=0; i<n; i++)
	{
		tube& _tube = tubes[i];
		if (_tube.isDynamic)
		{
			//make the sections static by putting their mass equal to zero
			for (int j=0; j<_tube.sections.size(); j++)
			{
				btRigidBody& rigid_body = *(_tube.sections[j]);
				rigid_body.setMassProps(0., btVector3(0.,0.,0.));
			}
			//delete the springs between the tube sections
			for (int j=0; j<_tube.springs.size(); j++)
			{
				m_dynamicsWorld->removeConstraint(_tube.springs[j]);
				delete _tube.springs[j];
				_tube.springs[j] = NULL;
			}
			// _tube.springs.clear();
			_tube.isDynamic = false;

			save_tube(_tube);
		}
	}
}

// remove the tubes from the simulation and only leave _max_number_of_tubes in the simulation
void cnt_mesh::remove_tube(int _max_number_of_tubes)
{
	int n = num_tubes() - _max_number_of_tubes;
	// std::cout << "n = " << n << "  num_tubes()=" << num_tubes() << "  _max_number_of_tubes=" << _max_number_of_tubes << std::endl;
	for (int i=0; i<n; ++i)
	{
		// std::cout << "i = " << i << " n = " << n << std::endl;
		// std::cout << "n = " << n << "  num_tubes()=" << num_tubes() << "  _max_number_of_tubes=" << _max_number_of_tubes << std::endl;
		tube& _tube = tubes[i];
		if (not _tube.isDynamic)
		{				
			for (int j=0; j<_tube.sections.size(); ++j)
			{
				deleteRigidBody(_tube.sections[j]);
				// delete _tube.sections[j];
				_tube.sections[j] = NULL;
			}
			_tube.sections.clear();
			// std::cout << "tube(" << i << ") section size:" << _tube.sections.size();
		}
	}
	if (n>0)
	{
		tubes.erase(tubes.begin(),tubes.begin()+n);
	}
}

// this method gives the appropriate coordinate for releasing the next tube
std::array<float, 3> cnt_mesh::drop_coordinate()
{
	std::array<float, 3> _drop_coordinate = {0., 0., 0.};
	
	_drop_coordinate[0] = half_Lx*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
	_drop_coordinate[1] = 2.0 + volume/(2.*half_Lx * 2.*half_Lz);
	_drop_coordinate[2] = half_Lz*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
	
	return _drop_coordinate;
}

void cnt_mesh::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void cnt_mesh::resetCamera()
{
	float dist = 41;
	float pitch = -35;
	float yaw = 52;
	float targetPos[3]={0,0.46,0};
	m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
}


void cnt_mesh::save_tube(tube &_tube)
{




	btTransform trans;
	btRigidBody* rigid_body;
	//make the sections static by putting their mass equal to zero
	for (int j=0; j<_tube.sections.size(); j++)
	{
		rigid_body = _tube.sections[j];
		rigid_body->getMotionState()->getWorldTransform(trans);
		std::cout << "Y coordinate = " << trans.getOrigin().getY() << std::endl;
		std::cin.ignore();
	}
}

void cnt_mesh::processCommandLineArgs(int argc, char* argv[])
{
	namespace fs = std::experimental::filesystem;


	std::cout << "current path is " << fs::current_path() << std::endl;
	std::cin.ignore();
}


