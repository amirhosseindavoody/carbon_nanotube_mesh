#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <experimental/filesystem>
#include <fstream>

#include "./helper/prepare_directory.hpp"
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

// create a rectangular container using half planes
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

// add a tube with certain number of sections, section length, and diameter
void cnt_mesh::add_tube(int _number_of_sections, float _section_length, float _diameter)
{
	tubes.emplace_back(_number_of_sections, _section_length, _diameter);
	tube& _tube = tubes.back();

	std::array<float, 3> _drop_coordinate = drop_coordinate();

	// create a few dynamic rigidbodies
	//*********************************************************************************************
	
	// Re-using the same collision is better for memory usage and performance
	btCollisionShape* colShape;

	bool colShape_exists = false;
	for (std::list<tube_section_collision_shape>::iterator it=tube_section_collision_shapes.begin(); it != tube_section_collision_shapes.end(); it++)
	{
		if (it->equals(_section_length, _diameter))
		{
			colShape = it->colShape;
			colShape_exists = true;
			break;
		}
	}

	if (not colShape_exists)
	{
		colShape = new btCylinderShape(btVector3(_diameter/2.0,_section_length/2.0,_diameter/2.0));
		m_collisionShapes.push_back(colShape);
		tube_section_collision_shapes.emplace_back(_section_length,_diameter);
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
	float fill_factor = 1.1;
	volume = volume + fill_factor*acos(-1)*std::pow(_diameter/2.0,2.0)*_section_length*float(_number_of_sections);
}

// make tubes static in the simulation and only leave _number_of_active_tubes as dynamic in the simulation.
void cnt_mesh::freeze_tube(int _number_of_active_tubes)
{
	int n = num_tubes() - _number_of_active_tubes;
	std::list<tube>::iterator it = tubes.begin();
	for (int i=0; i<n; i++)
	{
		tube& _tube = *it;
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
		it++;
	}
}

// remove the tubes from the simulation and only leave _max_number_of_tubes in the simulation
void cnt_mesh::remove_tube(int _max_number_of_tubes)
{
	int n = num_tubes() - _max_number_of_tubes;

	while (num_tubes() > _max_number_of_tubes)
	{
		tube& _tube = tubes.front();
		if (not _tube.isDynamic)
		{				
			for (int j=0; j<_tube.sections.size(); ++j)
			{
				deleteRigidBody(_tube.sections[j]);
				// delete _tube.sections[j];
				_tube.sections[j] = NULL;
			}
			_tube.sections.clear();
		}
		tubes.pop_front();
	}
}

// this method gives the appropriate coordinate for releasing the next tube
std::array<float, 3> cnt_mesh::drop_coordinate()
{
	std::array<float, 3> _drop_coordinate = {0., 0., 0.};
	
	_drop_coordinate[0] = half_Lx*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
	// _drop_coordinate[1] = 2.0 + volume/(2.*half_Lx * 2.*half_Lz);
	_drop_coordinate[1] = 5.0 + Ly;
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

// save coordinates of the tube
void cnt_mesh::save_tube(tube &_tube)
{
	if (number_of_saved_tubes % 10000 == 0)
	{
		file.close();
		number_of_cnt_output_files ++;
		std::string filename = "tube"+std::to_string(number_of_cnt_output_files)+".dat";
		output_file_path = output_directory / filename;
		file.open(output_file_path, std::ios::out);
		file << std::showpos << std::scientific;
	}

	number_of_saved_tubes ++;
	file << "tube number: " << number_of_saved_tubes << " ; ";

	btTransform trans;
	btRigidBody* rigid_body;
	for (int j=0; j<_tube.sections.size(); j++)
	{
		rigid_body = _tube.sections[j];
		rigid_body->getMotionState()->getWorldTransform(trans);
		file << trans.getOrigin().getX() << " , " << trans.getOrigin().getY() << " , " << trans.getOrigin().getZ() << " ; ";
	}
	file << "\n";
}

void cnt_mesh::processCommandLineArgs(int argc, char* argv[])
{
	namespace fs = std::experimental::filesystem;

	std::cout << "current path is " << fs::current_path() << std::endl;
	
	output_directory.assign("/home/amirhossein/research/cnt_mesh_longer_bids");
	
	if (not fs::exists(output_directory.path()))
	{
		std::cout << "warning: output directory does NOT exist!!!" << std::endl;
		std::cout << "output directory: " << output_directory.path() << std::endl;
		fs::create_directories(output_directory.path());
		// std::exit(EXIT_FAILURE);
	}

	if (not fs::is_directory(output_directory.path()))
	{
		std::cout << "error: output path is NOT a directory!!!" << std::endl;
		std::cout << "output path: " << output_directory.path() << std::endl;
		std::exit(EXIT_FAILURE);
	}

	if (not fs::is_empty(output_directory.path()))
	{
		std::cout << "warning: output directory is NOT empty!!!" << std::endl;
		std::cout << "output directory: " << output_directory.path() << std::endl;
		std::cout << "deleting the existing directory!!!" << std::endl;
		fs::remove_all(output_directory.path());
		fs::create_directories(output_directory.path());
		// std::exit(EXIT_FAILURE);
	}

	output_file_path	 = output_directory.path() / "tube.dat";
	std::cout << "output file name:" << output_file_path << std::endl;
	// std::cin.ignore();

}

// function to set the output directory
void cnt_mesh::set_output_dir(std::string output_path)
{
	output_directory = prepare_directory(output_dir, keep_old_files=true);
}


void cnt_mesh::get_Ly()
{
	btTransform trans;
	for (std::list<tube>::iterator it = tubes.begin(); it != tubes.end(); it++)
	{
		if (it->isDynamic)
		{
			for (int i=0; i < it->sections.size(); i++)
			{
				it->sections[i]->getMotionState()->getWorldTransform(trans);
				Ly = std::max(Ly, trans.getOrigin().getY());
			}
		}
	}
}


