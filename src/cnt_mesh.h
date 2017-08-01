#ifndef cnt_mesh_h
#define cnt_mesh_h

#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 

#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

struct cnt_mesh : public CommonRigidBodyBase
{

	private:
	// container properties
	float half_Lx,half_Lz; // container size. y-axis is the direction in which the top of the container is open (vertical direction and the direction in which gravity is applied), Lx and Lz are the direction in which the container is enclosed
	float Ly; // this is an average height for the stack of cnt mesh
	float volume; // volume of the total carbon nanotubes in the container

	// vector to store all the tubes that we will in the simulation
	class tube;
	std::vector<tube> tubes;

	// vector to store all the collision shapes of the cnt sections that we will create in the simulation
	class tube_section_collision_shape;
	std::vector<tube_section_collision_shape> tube_section_collision_shapes;

	// class to store collision shapes of carbon nanotube sections. The diameter and the length of the section is used to see
	// if we have created such collision before or we need to create it for the first time.
	class tube_section_collision_shape
	{
	public:
		float length;
		float diameter;
		btCollisionShape* colShape;
		tube_section_collision_shape(float _length, float _diameter)
		{
			length = _length;
			diameter = _diameter;
		}
		bool equals(float _length, float _diameter)
		{
			if (int((length-_length)*1.e6)==0 and int((diameter-_diameter)*1.e6)==0)
			{
				return true;
			}
			return false;
		}
	};

	// class to store information and the rigid bodies of each separate cnt.
	class tube
	{
	public:
		int number_of_sections;
		float section_length;
		float diameter;
		float length;
		bool isDynamic;
		std::vector<btRigidBody*> sections;
		std::vector<btPoint2PointConstraint*> springs;
		tube(int _number_of_sections = 10, float _section_length = 1, float _diameter = 0.5)
		{
			number_of_sections = _number_of_sections;
			section_length = _section_length;
			diameter = _diameter;
			length = float(number_of_sections)*section_length;
			isDynamic = true;
		}
	};


	// this method gives the appropriate coordinate for releasing the next tube
	std::array<float, 3> drop_coordinate()
	{
		std::array<float, 3> _drop_coordinate = {0., 0., 0.};
		
		_drop_coordinate[0] = half_Lx*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
		_drop_coordinate[1] = 2.0 + volume/(2.*half_Lx * 2.*half_Lz);
		_drop_coordinate[2] = half_Lz*((2.0*float(std::rand())/float(RAND_MAX))-1.0);
		
		return _drop_coordinate;
	}

public:
	cnt_mesh(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
		std::srand(std::time(0)); // use current time as seed for random generator
		std::cout << "seeded the random number generator!!!" << std::endl;
	}
	virtual ~cnt_mesh(){}
	virtual void initPhysics();
	virtual void renderScene();
	
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}



	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime,10,deltaTime);
		}
	}

	void add_tube(int _number_of_sections = 10, float _section_length = 1, float _diameter = 0.5); // this method adds a tube to the system.
	void create_container(int _half_Lx=20, int _half_Lz=20); // this method creates an open top container for the cnts

	inline int num_tubes()
	{
		return tubes.size();
	}

	void freeze_tube(int _number_of_active_tubes)
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
				_tube.springs.clear();
				_tube.isDynamic = false;
			}
		}
	}

	void remove_tube(int _max_number_of_tubes)
	{
		int n = num_tubes() - _max_number_of_tubes;

		for (int i=0; i<n; i++)
		{
			tube& _tube = tubes[i];
			if (not _tube.isDynamic)
			{				
				for (int j=0; j<_tube.sections.size(); j++)
				{
					deleteRigidBody(_tube.sections[j]);
					delete _tube.sections[j];
					_tube.sections[j] = NULL;
				}
				_tube.sections.clear();
				std::cout << "tube(" << i << ") section size:" << _tube.sections.size();
			}
		}

		tubes.erase(tubes.begin(),tubes.begin()+2);
	}

};


#endif //cnt_mesh_h
