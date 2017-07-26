#ifndef ET_CHAIN_EXAMPLE_H
#define ET_CHAIN_EXAMPLE_H


#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 

#include "../misc_files/CommonInterfaces/CommonRigidBodyBase.h"

struct ChainExample : public CommonRigidBodyBase
{
	ChainExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~ChainExample(){}
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
};


#endif //ET_CHAIN_EXAMPLE_H
