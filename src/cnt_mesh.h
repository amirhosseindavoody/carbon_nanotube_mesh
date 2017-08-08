#ifndef cnt_mesh_h
#define cnt_mesh_h

#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>
#include <list>
#include <experimental/filesystem>
#include <fstream>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 

#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

struct cnt_mesh : public CommonRigidBodyBase
{

	private:

	// infomation storing the simulation information
	std::experimental::filesystem::directory_entry output_directory; // this is the address of the output_directory
	std::experimental::filesystem::path output_file_path; // this is the full address of the output file.
	std::fstream file; // this is the output file that the coordinate of the cnts are written into.
	int number_of_saved_tubes; // this is the total number of cnts whos coordinates are saved into output file.
	int number_of_cnt_output_files; // this is the number of output files that the cnt coordinates has been written into.

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
			sections.reserve(_number_of_sections);
			springs.reserve(_number_of_sections-1);
		}
	};

	std::array<float, 3> drop_coordinate(); // this method gives the appropriate coordinate for releasing the next tube

	void create_large_mesh_body(); // INCOMPLETE: this method creates a new static body so that we can remove unecessary static CNTs.

public:
	cnt_mesh(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
		std::srand(std::time(0)); // use current time as seed for random generator
		std::cout << "seeded the random number generator!!!" << std::endl;
		tubes.reserve(10000);

		// initialize the output parameters
		number_of_saved_tubes = 0;
		number_of_cnt_output_files = 0;
	}
	virtual ~cnt_mesh()
	{
		file.close();
		// std::cout << "ran" << "\n";
	}
	virtual void initPhysics();
	virtual void renderScene();
	
	void resetCamera();

	inline void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime,10,deltaTime);
		}
	}

	void add_tube(int _number_of_sections = 10, float _section_length = 1, float _diameter = 0.5); // this method adds a tube to the system.
	void create_container(int _half_Lx=20, int _half_Lz=20); // this method creates an open top container for the cnts

	// gets the number of tubes in the simulation
	inline int num_tubes()	{return tubes.size();}

	void freeze_tube(int _number_of_active_tubes); // make tubes static in the simulation and only leave _number_of_active_tubes as dynamic in the simulation.
	void remove_tube(int _max_number_of_tubes); // remove the tubes from the simulation and only leave _max_number_of_tubes in the simulation
	void save_tube(tube &_tube);

	void processCommandLineArgs(int argc, char* argv[]);

};


#endif //cnt_mesh_h
