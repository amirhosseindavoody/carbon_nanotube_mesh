#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <experimental/filesystem>
#include <fstream>
#include <cstddef>

#include "../lib/json.hpp"
#include "./helper/prepare_directory.hpp"
#include "cnt_mesh.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

void cnt_mesh::initPhysics() {
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);
}

// create a rectangular container using half planes
void cnt_mesh::create_container(){

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

	// // create the z direction side wall planes
	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
	// 	groundTransform.setOrigin(btVector3(0,0,-_half_Lz));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }

	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, -1), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
	// 	groundTransform.setOrigin(btVector3(0,0,_half_Lz));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }


	// // create the x direction side wall planes
	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
	// 	groundTransform.setOrigin(btVector3(-_half_Lx,0,0));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }

	// {
	// 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(-1, 0, 0), 0); // plane collision shape with an offset of 0 unit from the origin
	// 	m_collisionShapes.push_back(groundShape);
		
	// 	btScalar mass(0.);

	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
	// 	groundTransform.setOrigin(btVector3(_half_Lx,0,0));	
	// 	createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1)); // I think the last input is not used for anything. On paper it is supposed to be the collor
	// }
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

// make tubes static in the simulation and only leave _number_of_active_tubes as dynamic in the simulation.
void cnt_mesh::freeze_tube(int _number_of_active_tubes) {
	int n = num_tubes() - _number_of_active_tubes;
	int i=0;
	for (auto t: tubes) {
		if (t.isDynamic) {
			//make the sections static by putting their mass equal to zero
			for(auto& b: t.bodies){
				b->setMassProps(0,btVector3(0,0,0));
			}

			//delete constraints between the tube sections
			for (auto& c: t.constraints) {
				m_dynamicsWorld->removeConstraint(c);
				delete c;
				c = nullptr;
			}

			// tube.constraints.clear();
			t.isDynamic = false;

			save_tube(t);

			i++;
		}

		if (i>=n)
			break;
	}
}

// remove the tubes from the simulation and only leave _max_number_of_tubes in the simulation
void cnt_mesh::remove_tube(int _max_number_of_tubes) {
	int n = num_tubes() - _max_number_of_tubes;

	while (tubes.size() > _max_number_of_tubes) {
		tube& my_tube = tubes.front();
		if (! my_tube.isDynamic) {				
			for (auto& b: my_tube.bodies) {
				deleteRigidBody(b);
				b = nullptr;
			}
			my_tube.bodies.clear();
		}
		tubes.pop_front();
	}
}

// this method gives the appropriate coordinate for releasing the next tube
std::array<float, 3> cnt_mesh::drop_coordinate() {
	std::array<float, 3> _drop_coordinate = {0., 0., 0.};
	
	_drop_coordinate[0] = _half_Lx*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
	_drop_coordinate[1] = 5.0 + Ly;
	_drop_coordinate[2] = _half_Lz*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
	
	return _drop_coordinate;
}

void cnt_mesh::renderScene() {
	CommonRigidBodyBase::renderScene();
}

void cnt_mesh::resetCamera() {
	float dist = 41;
	float pitch = -35;
	float yaw = 52;
	float targetPos[3]={0,0.46,0};
	m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
}

// save coordinates of the tube
void cnt_mesh::save_tube(tube &t) {
	if (number_of_saved_tubes % 10000 == 0) {
		file.close();
		number_of_cnt_output_files ++;
		std::string filename = "tube"+std::to_string(number_of_cnt_output_files)+".dat";
		output_file_path = _output_directory / filename;
		file.open(output_file_path, std::ios::out);
		file << std::showpos << std::scientific;
	}

	number_of_saved_tubes ++;
	file << "tube number: " << number_of_saved_tubes << " ; ";

	btTransform trans;
	for (const auto& b: t.bodies) {
		b->getMotionState()->getWorldTransform(trans);
		file << trans.getOrigin().getX() << " , " << trans.getOrigin().getY() << " , " << trans.getOrigin().getZ() << " ; ";
	}

	file << "\n";
}


void cnt_mesh::get_Ly() {
	btTransform trans;
	float avgY=0;
	int count=0;
	for (const auto& t:tubes) {
		if (t.isDynamic) {
			for (const auto& b: t.bodies) {
				b->getMotionState()->getWorldTransform(trans);
				avgY += trans.getOrigin().getY();
				count++;
			}
		}
	}

	if (count==0) {
		Ly = 0;
	} else {
		Ly = avgY/float(count);
	}
	
}

// set and save the json properties that is read and parsed from the input_json file.
void cnt_mesh::save_json_properties(nlohmann::json j) {
	std::ofstream json_file;
	json_file.open(_output_directory.path() / "input.json", std::ios::out);
	json_file << std::setw(4) << j << std::endl;
	json_file.close();
};

// add a tube with a diameter randomly picked from prebuilt colShapes for tube sections
void cnt_mesh::add_tube() {

	tubes.push_back(tube());
	tube& my_tube = tubes.back();

	int d = std::rand()%_tube_section_collision_shapes.size(); // index related to the diameter of the tube
	my_tube.diameter = _tube_diameter[d];
	
	int l = std::rand()%_tube_length.size(); // index related to the length of the tube
	float length = _tube_length[l];

	std::array<float, 3> _drop_coordinate = drop_coordinate();

	// create a few dynamic rigidbodies
	//*********************************************************************************************


	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);

	// Re-using the same collision is better for memory usage and performance
	btCollisionShape* colShape=nullptr;

	float c_length=0;

	while(c_length<length) {
		int sl = std::rand()%_section_length.size();
		colShape = _tube_section_collision_shapes[d][sl];


		// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar x_loc = _drop_coordinate[0];
		btScalar y_loc = _drop_coordinate[1] + c_length + _section_length[sl]/2. + my_tube.diameter;
		btScalar z_loc = _drop_coordinate[2];
		btVector3 origin(x_loc, y_loc, z_loc);
		startTransform.setOrigin(origin);
		my_tube.bodies.push_back(createRigidBody(mass,startTransform,colShape));	// no static object
		my_tube.bodies.back()->setMassProps(1.0,btVector3(1,0,1)); // turn off rotation along the y-axis of the cylinder shapes
		my_tube.body_length.push_back(_section_length[sl]);

		c_length += _section_length[sl]+my_tube.diameter;
	}

	my_tube.length = c_length;

	//add N-1 spring constraints
	for(int i=0;i<my_tube.bodies.size()-1;++i) {
		btRigidBody* b1 = my_tube.bodies[i];
		btRigidBody* b2 = my_tube.bodies[i+1];
		
		// spring constraint
		// btPoint2PointConstraint* centerSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0,(my_tube.body_length[i]+my_tube.diameter)/2,0), btVector3(0,-(my_tube.body_length[i+1]+my_tube.diameter)/2,0));
		// centerSpring->m_setting.m_damping = 1.5; //the damping value for the constraint controls how stiff the constraint is. The default value is 1.0
		// centerSpring->m_setting.m_impulseClamp = 0; //The m_impulseClamp value controls how quickly the dynamic rigid body comes to rest. The defual value is 0.0



		const float pi = 3.14159265358979323846;

		btTransform frameInA, frameInB;
		frameInA = btTransform::getIdentity();
		frameInA.getBasis().setEulerZYX(1, 0, 1);
		frameInA.setOrigin(btVector3(0,1.1*(my_tube.body_length[i])/2,0));
		frameInB = btTransform::getIdentity();
		frameInB.getBasis().setEulerZYX(1,0, 1);
		frameInB.setOrigin(btVector3(0,-1.1*(my_tube.body_length[i+1])/2,0));





		btConeTwistConstraint* centerSpring = new btConeTwistConstraint(*b1, *b2, frameInA, frameInB);
		
		// centerSpring->setLimit(pi/6, pi/6, pi);
		centerSpring->setLimit(pi/20,pi/20,pi,2,0.3,1.0F);


		m_dynamicsWorld->addConstraint(centerSpring);
		my_tube.constraints.push_back(centerSpring);
	}


	// generate the graphical representation of the object
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}