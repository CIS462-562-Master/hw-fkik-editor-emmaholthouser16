#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	//need to get the root node in global position
	vec3 newY = m_Guide.getGlobalRotation() * m_pSkeleton->getRootNode()->getGlobalTranslation() + m_Guide.getGlobalTranslation();
	// 2.	Set the y component of the guide position to 0
	newY[1] = 0;
	m_Guide.setGlobalTranslation(newY);
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	//direction that we want it to be going
	vec3 newVec = guideTargetPos - m_Guide.getGlobalTranslation();
	newVec[1] = 0;
	//normalize to use in rotation matrix
	vec3 newNorm = newVec.Normalize();
	//regular y axis
	vec3 y = vec3(0.f, 1.f, 0.f);
	//x is cross product of y and z
	vec3 xAxis = y.Cross(newNorm);
	mat3 newRotation;
	newRotation[0] = xAxis.Normalize();
	newRotation[1] = y;
	newRotation[2] = newNorm;
	//need to transpose because of matrix constructor
    m_Guide.setGlobalRotation(newRotation.Transpose());
	m_pSkeleton->update();


}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	//setting it to the maximum height
	if (leftHeight > rightHeight)
	{
		vec3 left = m_pSkeleton->getRootNode()->getLocalTranslation();
		vec3 newLeft = vec3(left[0], (left[1] + leftHeight), left[2]);
		m_pSkeleton->getRootNode()->setLocalTranslation(newLeft);
	}
	else
	{
		vec3 right = m_pSkeleton->getRootNode()->getLocalTranslation();
		vec3 newRight = vec3(right[0], (right[1] + rightHeight), right[2]);
		m_pSkeleton->getRootNode()->setLocalTranslation(newRight);
	}

	m_pSkeleton->update();

	ATarget leftTarget;
	vec3 leftLoc = leftFoot->getGlobalTranslation();
	//adding height to the global translation
	vec3 leftTarg = vec3(leftLoc[0], leftLoc[1] + leftHeight, leftLoc[2]);
	leftTarget.setGlobalTranslation(leftTarg);
	

	ATarget rightTarget;
	vec3 rightLoc = rightFoot->getGlobalTranslation();
	vec3 rightTarg = vec3(rightLoc[0], rightLoc[1] + rightHeight, rightLoc[2]);
	rightTarget.setGlobalTranslation(rightTarg);
	
	//m_pSkeleton->update();
	//compute the limb ik for the left and the right foot
	m_IKController->IKSolver_Limb(m_IKController->mRfootID, rightTarget);
	m_IKController->IKSolver_Limb(m_IKController->mLfootID, leftTarget);
	
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		//should be the y 
		//x should be the same since we rotate around it
		vec3 x = leftFoot->getLocalRotation() * vec3(1, 0, 0);
		//z is cross between y and x
		vec3 z = leftNormal.Cross(x);
		mat3 newRot;
		newRot[0] = x;
		newRot[1] = leftNormal;
		newRot[2] = z;
		leftFoot->setLocalRotation(newRot.Transpose());

		//;
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		vec3 x = rightFoot->getLocalRotation() * vec3(1, 0, 0);
		vec3 z = rightNormal.Cross(x);
		mat3 newRot;
		newRot[0] = x;
		newRot[1] = rightNormal;
		newRot[2] = z;
		rightFoot->setLocalRotation(newRot.Transpose());
		
	}
	m_pSkeleton->update();
}
