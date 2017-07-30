#ifndef cnt_mesh_h
#define cnt_mesh_h

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

	float volume;

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
		std::vector<btRigidBody*> sections;
		tube(int _number_of_sections = 10, float _section_length = 1, float _diameter = 0.5)
		{
			number_of_sections = _number_of_sections;
			section_length = _section_length;
			diameter = _diameter;
			length = float(number_of_sections)*section_length;
		}
	};

	// vector to store all the tubes that we will in the simulation
	std::vector<tube> tubes;

	// vector to store all the collision shapes of the cnt sections that we will create in the simulation
	std::vector<tube_section_collision_shape> tube_section_collision_shapes;

public:
	cnt_mesh(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
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

	virtual void add_tube(int _number_of_sections = 10, float _section_length = 1, float _diameter = 0.5, std::array<float, 3> _drop_coordinate={0,5,0}); // this method adds a tube to the system.
	virtual void create_container(int _half_Lx=20, int _half_Lz=20); // this method creates an open top container for the cnts

	int num_tubes()
	{
		return tubes.size();
	}


};


#endif //cnt_mesh_h
