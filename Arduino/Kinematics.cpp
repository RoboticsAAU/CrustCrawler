#include "Kinematics.h"

/////////////////////////////////////////////////////////////////////////////
// THE FOLLOWING FUNCTIONS ARE NOT IMPLEMENTED, I.E. THEY ARE NEVER CALLED //
/////////////////////////////////////////////////////////////////////////////

// Function that determines the position of the wrist at given joint angles
eePosition Kinematics::ForwardKinematics(JointAngles& JointAngles)
{
	eePosition returnEEPos;

	// Forward kinematics model of the CrustCrawler
	returnEEPos.x = Joints[2]->Length * cos(JointAngles.thetas[2]) - cos(JointAngles.thetas[1]) * Joints[3]->Length * sin(JointAngles.thetas[2] + JointAngles.thetas[3]);
	returnEEPos.y = Joints[2]->Length * cos(JointAngles.thetas[2]) + sin(JointAngles.thetas[1]) * Joints[3]->Length * sin(JointAngles.thetas[2] + JointAngles.thetas[3]);
	returnEEPos.z = Joints[2]->Length * cos(JointAngles.thetas[2]) + Joints[1]->Length + Joints[3]->Length * cos(JointAngles.thetas[2] + JointAngles.thetas[3]);
	
	return returnEEPos;
}
