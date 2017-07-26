#ifndef BASIC_EXAMPLE_H
#define BASIC_EXAMPLE_H

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample(){}
	virtual void initPhysics();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};


#endif //BASIC_DEMO_PHYSICS_SETUP_H
