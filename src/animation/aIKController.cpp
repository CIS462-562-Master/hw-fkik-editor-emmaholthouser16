#include "aIKController.h"
#include "aActor.h"

#pragma warning (disable : 4018)

int IKController::gIKmaxIterations = 5;
double IKController::gIKEpsilon = 0.1;

// AIKchain class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
AIKchain::AIKchain()
{
	mWeight0 = 0.1;
}

AIKchain::~AIKchain()
{

}

AJoint* AIKchain::getJoint(int index) 
{ 
	return mChain[index]; 
}

void AIKchain::setJoint(int index, AJoint* pJoint) 
{ 
	mChain[index] = pJoint; 
}

double AIKchain::getWeight(int index) 
{ 
	return mWeights[index]; 
}

void AIKchain::setWeight(int index, double weight) 
{ 
	mWeights[index] = weight; 
}

int AIKchain::getSize() 
{ 
	return mChain.size(); 
}

std::vector<AJoint*>& AIKchain::getChain() 
{ 
	return mChain; 
}

std::vector<double>& AIKchain::getWeights() 
{ 
	return mWeights; 
}

void AIKchain::setChain(std::vector<AJoint*> chain) 
{
	mChain = chain; 
}

void AIKchain::setWeights(std::vector<double> weights) 
{ 
	mWeights = weights; 
}

// AIKController class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

IKController::IKController()
{
	m_pActor = NULL;
	m_pSkeleton = NULL;
	mvalidLimbIKchains = false;
	mvalidCCDIKchains = false;

	// Limb IK
	m_pEndJoint = NULL;
	m_pMiddleJoint = NULL;
	m_pBaseJoint = NULL;
	m_rotationAxis = vec3(0.0, 1.0, 0.0);

	ATransform desiredTarget = ATransform();
	mTarget0.setLocal2Parent(desiredTarget);  // target associated with end joint
	mTarget1.setLocal2Parent(desiredTarget);  // optional target associated with middle joint - used to specify rotation of middle joint about end/base axis
	mTarget0.setLocal2Global(desiredTarget);
	mTarget1.setLocal2Global(desiredTarget);

	//CCD IK
	mWeight0 = 0.1;  // default joint rotation weight value

}

IKController::~IKController()
{
}

ASkeleton* IKController::getSkeleton()
{
	return m_pSkeleton;
}

const ASkeleton* IKController::getSkeleton() const
{
	return m_pSkeleton;
}

ASkeleton* IKController::getIKSkeleton()
{
	return &mIKSkeleton;
}

const ASkeleton* IKController::getIKSkeleton() const
{
	return &mIKSkeleton;
}

AActor* IKController::getActor()
{
	return m_pActor;
}

void IKController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}


AIKchain IKController::createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton)
{
	// TODO: given the end joint ID and the desired length of the IK chain, 
	// add the corresponding skeleton joint pointers to the AIKChain "chain" data member starting with the end joint
	// desiredChainSize = -1 should create an IK chain of maximum length (where the last chain joint is the joint before the root joint)
	// also add weight values to the associated AIKChain "weights" data member which can be used in a CCD IK implemention
	AIKchain newChain;
	std::vector<AJoint*> jointChain;
	std::vector<double> weights;
	AJoint* end = pSkeleton->getJointByID(endJointID);
	jointChain.push_back(pSkeleton->getJointByID(endJointID));
	weights.push_back(0.1);
	if (desiredChainSize == -1) {
		while (end->getParent()->getParent() != nullptr) {
			jointChain.push_back(end->getParent());
			weights.push_back(0.1);
			end = end->getParent();
		}
	}
	else {
		for (int i = 0; i < desiredChainSize - 1; i++) {
			jointChain.push_back(end->getParent());
			weights.push_back(0.1);
			end = end->getParent();

		}
	}
	newChain.setChain(jointChain);
	newChain.setWeights(weights);
	return newChain;
	//return AIKchain();
}



bool IKController::IKSolver_Limb(int endJointID, const ATarget& target)
{
	// Implements the analytic/geometric IK method assuming a three joint limb  

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		if (!mvalidCCDIKchains) {
			return false;
		}
	}

	vec3 desiredRootPosition;

	// The joint IDs are not const, so we cannot use switch here
	if (endJointID == mLhandID)
	{
		mLhandTarget = target;
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
	}
	else if (endJointID == mRhandID)
	{
		mRhandTarget = target;
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
	}
	else if (endJointID == mLfootID)
	{
		mLfootTarget = target;
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
	}
	else if (endJointID == mRfootID)
	{
		mRfootTarget = target;
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
	}
	else if (endJointID == mRootID)
	{
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
	}
	else
	{
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}



int IKController::createLimbIKchains()
{
	bool validChains = false;
	int desiredChainSize = 3;

	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);
	
	if (mLhandIKchain.getSize() == 3 && mRhandIKchain.getSize() == 3 && mLfootIKchain.getSize() == 3 && mRfootIKchain.getSize() == 3)
	{
		validChains = true;
		
		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}




int IKController::computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 midJointAxis, ASkeleton* pIKSkeleton)
{
	// TODO: Implement the analytic/geometric IK method assuming a three joint limb 
	//end joint is [0] in ik chain
	//get the length of the top arm part
	double l1 = IKchain.getJoint(1)->getLocalTranslation().Length();
	double l2 = IKchain.getJoint(0)->getLocalTranslation().Length();


	//double rd = target.getLocalTranslation().Length() - IKchain.getJoint(2)->getLocalTranslation().Length();

	vec3 tV = target.getGlobalTranslation() - IKchain.getJoint(2)->getGlobalTranslation();
	double rd = tV.Length();
	//r is the joint to joint
	
	//clamp arc cos
	
	double phi = acos(std::fmin(std::fmax(-1, (((l1) * (l1)+(l2) * (l2)-(rd) * (rd)) / (2 * l1 * l2))), 1));

	 //local of elbow 
	double theta = M_PI - phi;
	quat qElbow;
	qElbow[3] = (cos(theta / 2));
	qElbow[0] = midJointAxis[0] * sin(theta / 2);
	qElbow[1] = midJointAxis[1] * sin(theta / 2);
	qElbow[2] = midJointAxis[2] * sin(theta / 2);

	mat3 localElbow = qElbow.ToRotation();
	IKchain.getJoint(1)->setLocalRotation(localElbow);

	//updating the skelton wiht the new elbow rotation
	pIKSkeleton->update();

	//vect
	//distance between wrist and shoulder
	vec3 rdV = IKchain.getJoint(0)->getGlobalTranslation() - IKchain.getJoint(2)->getGlobalTranslation();

	//length of the two vectors
	double magT = tV.Length();

	double magR = rdV.Length();

	//check if too long
	if (magT > l1 + l2) {
	//	return false;
	}

	//angle for shoulder
	double ang = acos(Dot(tV, rdV) / (magT * magR));

	//axis for shoulder
	vec3 axis = rdV.Cross(tV) / (rdV.Cross(tV)).Length();

	//updated axis to get into the right coordinate space in relation to the shoulder joint
	vec3 updatedAxis = IKchain.getJoint(2)->getGlobalRotation().Inverse() * axis;

	quat q0(cos(ang / 2), updatedAxis[0] * sin(ang/2), updatedAxis[1] * sin(ang/2), updatedAxis[2] * sin(ang/2));
	
	//getting it to a rotation matrix
	mat3 shouldRot = q0.ToRotation();
	//setting rotation of shoulder
	IKchain.getJoint(2)->setLocalRotation(IKchain.getJoint(2)->getLocalRotation() * shouldRot);

	//orientation of wrist
	mat3 newWrist = (IKchain.getJoint(2)->getGlobalRotation() * IKchain.getJoint(1)->getLocalRotation()).Inverse() * IKchain.getJoint(0)->getGlobalRotation();
	IKchain.getJoint(0)->setLocalRotation(newWrist);
	// The actual position of the end joint should match the target position within some episilon error 
	// the variable "midJointAxis" contains the rotation axis for the middle joint
	pIKSkeleton->update();
	return true;
}

bool IKController::IKSolver_CCD(int endJointID, const ATarget& target)
{
	// Implements the CCD IK method assuming a three joint limb 

	bool validChains = false;

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;


	if (endJointID == mLhandID)
	{
		mLhandTarget = target;
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
	}
	else if (endJointID == mRhandID)
	{
		mRhandTarget = target;
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
	}
	else if (endJointID == mLfootID)
	{
		mLfootTarget = target;
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
	}
	else if (endJointID == mRfootID)
	{
		mRfootTarget = target;
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
	}
	else if (endJointID == mRootID)
	{
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
	}
	else
	{
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computeCCDIK(target, mIKchain, &mIKSkeleton);
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createCCDIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}


int IKController::computeCCDIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement CCD IK  
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	int length = IKchain.getSize();
	
	//Hint:

	int count = 0;
	vec3 e = target.getGlobalTranslation() - IKchain.getJoint(0)->getGlobalTranslation();
	//target.getGlobalTranslation() - IKchain.getJoint(0)->getGlobalTranslation();
	//e.Length() > 0.1 ||
	while ( count < 3) {
		//vec3 dist(0, 0, 0);
		//int	count = 0;
		// 1. compute axis and angle for a joint in the IK chain (distal to proximal) in global coordinates
		for (int i = 1; i < length; i++) {
			
			vec3 dist = IKchain.getJoint(0)->getGlobalTranslation() - IKchain.getJoint(i)->getGlobalTranslation();
			float angTop = (dist.Cross(e)).Length();
			float angBottom = Dot(dist, dist) + Dot(dist, e);
			float ang = IKchain.getWeight(i) * (angTop / angBottom);

			vec3 axis = (dist.Cross(e) / ((dist.Cross(e)).Length()));

			//converting to local axis
			vec3 LocalAxis = IKchain.getJoint(i)->getGlobalRotation().Inverse() * axis;
			//float updatedAng = std::fmin(std::fmax(0, ang), 1);
			quat q;
			q[0] = LocalAxis[0] * sin(ang / 2);
			q[1] = LocalAxis[1] * sin(ang / 2);
			q[2] = LocalAxis[2] * sin(ang / 2);
			q[3] = cos(ang / 2);

			mat3 localRot = q.ToRotation();


			IKchain.getJoint(i)->setLocalRotation(IKchain.getJoint(i)->getLocalRotation() * localRot);

			IKchain.getJoint(i)->updateTransform();
			e = target.getGlobalTranslation() - IKchain.getJoint(0)->getGlobalTranslation();

			

		}
		count++;
		//if (count > 1) { break; }
		
	 } 

	//AIKchain

	// 2. once you have the desired axis and angle, convert axis to local joint coords 
	// 3. compute desired change to local rotation matrix
	// 4. set local rotation matrix to new value
	// 5. update transforms for joint and all children
	return true;
}


bool IKController::IKSolver_PseudoInv(int endJointID, const ATarget& target)
{
	// TODO: Implement Pseudo Inverse-based IK  
	// The actual position of the end joint should match the target position after the skeleton is updated with the new joint angles
	return true;
}

bool IKController::IKSolver_Other(int endJointID, const ATarget& target)
{
	// TODO: Put Optional IK implementation or enhancements here
	 
	return true;
}