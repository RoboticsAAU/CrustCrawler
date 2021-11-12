#include "Controller.h"

Controller::Controller(){

	ComputerConnector* computerConnector = new ComputerConnector();
	Dynamics* dynamics = new Dynamics();
	Kinematics* kinematics = new Kinematics();
	DynamixelConnector* dynamixelConnector = new DynamixelConnector();

}


Controller::~Controller(){
} 


void Controller::main(){

	Looptime = _UpdateLoopTime();
	computerConnector->updateComputerData();
	dynamixelConnector->getJointAngles(Degree,AngleData);
	kinematics->UpdateForwardkinematics();
	dynamics->UpdateDynamics(Looptime);

}

unsigned long Controller::_UpdateLoopTime(){

	_NewTime = millis();
	unsigned long TimeStamp = (_NewTime - _PrevTime)/1000;
	_PrevTime = _NewTime;

	return TimeStamp;


}




// ----------------- Everything below this have to be moved ------------------- \\ 
//void Controller::_SpaceConverter(SpaceType desiredSpace){
//	//_AngleConverter(Radians);
//	
//	if(desiredSpace == inputMotion.currentSpaceType){
//		return;
//	}
//	
//	//Forward Jacobian:
//	BLA::Matrix<3, 3> jacobian;
//	jacobian(0,0) = sin(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
//	jacobian(0,1) = -cos(inputAngles.m_Theta1) * (Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
//	jacobian(0,2) = -cos(inputAngles.m_Theta1) * Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
//	jacobian(1,0) = -cos(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
//	jacobian(1,1) = -sin(inputAngles.m_Theta1) * (Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
//	jacobian(1,2) = -sin(inputAngles.m_Theta1) * Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
//	jacobian(2,0) = 0;
//	jacobian(2,1) = -Joint2.m_length * sin(inputAngles.m_Theta2) - Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);
//	jacobian(2,2) = -Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);
//
//	//Inverse Jacobian:
//	BLA::Matrix<3, 3> jacobianInverse = jacobian;
//	bool is_nonsingular = Invert(jacobianInverse);
//
//	//for (int i = 0; i < 3; i++) {
//	//	for (int j = 0; j < 3; j++) {
//	//		Serial.print(jacobianInverse(i, j));
//	//		Serial.print(" ");
//	//	}
//	//	Serial.println(" ");
//	//}
//	//Serial.println(" ");
//
//
//	//If the jacobian is singular. Wait implementation until further understanding.
//	/*if(!is_nonsingular)
//	{
//		
//	}*/
//
//	BLA::Matrix<3,1> velocityVector;
//	velocityVector(0, 0) = Joint1.m_vel;
//	velocityVector(1, 0) = Joint2.m_vel;
//	velocityVector(2, 0) = Joint3.m_vel;
//
//	switch (desiredSpace){
//	case JointSpace: {
//		velocityVector = jacobianInverse * velocityVector;
//	}
//	case CartesianSpace: {
//		velocityVector = jacobian * velocityVector;
//	}
//	}
//
//	Joint1.m_vel = velocityVector(0,0);
//	Joint2.m_vel = velocityVector(1,0);
//	Joint3.m_vel = velocityVector(2,0);
//
//	MotionData.currentSpaceType = desiredSpace;
//}


double Controller::_PID(double desiredValue, double currentValue){
	double Kp{ 0.7 }, Ki{ 0.01 }, Kd{ 0.1 };
	double error = desiredValue - currentValue;
	
	m_proportional = Kp * error;
	m_integral = Ki * IntegrationOperator(error, m_integral, Looptime);
	m_derivative = Kd * DifferentiationOperator(error, m_lastError, Looptime);
	
	//m_integral += Ki * (error * samplingTime);
	//m_derivative = Kd * ((error - m_lastError) / samplingTime);
	
	m_lastError = error;

	return (m_proportional + m_integral + m_derivative);
}




void Controller::_GetJointPWMConstants(Joint& inputJoint) {
	SpaceConverter(JointSpace);


	int torque_sign = inputJoint.m_torque;
	int vel_sign = inputJoint.m_vel;

	int constantPicker = torque_sign * vel_sign;

	// if(constantPicker > 0) a
	// else if (constantPicker < 0) c
	// else b

	switch(inputJoint.m_servoType) {
	case MX28R: {
		
		if (constantPicker < 0) { inputJoint.m_constantC1 = 211.7; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 427.4; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 642.0; }

		inputJoint.m_constantC2 = 115.3;

	}
	case MX64R: {

		if (constantPicker < 0) { inputJoint.m_constantC1 = 80.9; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 152.7; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 224.5; }
		
		inputJoint.m_constantC2 = 105.3;

	}
	case MX106R: {
		
		if (constantPicker < 0) { inputJoint.m_constantC1 = 40.4; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 83.9; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 127.5; }
		
		inputJoint.m_constantC2 = 160.6;

	}
	}
}

bool Controller::_IsWithinAngleBoundaries(Joint inputJoint, double inputAngle) {
	return inputAngle >= inputJoint.m_minTheta && inputAngle <= inputJoint.m_maxTheta;
}

void Controller::_TorqueToPWM(Joint& inputJoint) {
	SpaceConverter(JointSpace);

	if (!_IsWithinAngleBoundaries(inputJoint, AngleData.m_currentThetas[inputJoint.m_id])) {		
		double _boundaryMidPoint = (inputJoint.m_maxTheta + inputJoint.m_minTheta) / 2;
		double _outputTheta = AngleData.m_currentThetas[inputJoint.m_id] > _boundaryMidPoint ? inputJoint.m_maxTheta : inputJoint.m_minTheta;

		inputJoint.m_PWM = _PID(_outputTheta, AngleData.m_currentThetas[inputJoint.m_id]);
		return;
	}

	if (inputJoint.m_servoType == MX28R) { //If the joint is a gripper servo
		inputJoint.m_PWM = _constantGripperPWM * inputJoint.m_vel; //In reality the velocity is just a sign (positive or negative 1, or 0 if not in gripper mode)
	}
	else {
		// Getting the joint constants for the joint
		_GetJointPWMConstants(inputJoint);

		inputJoint.m_PWM = inputJoint.m_torque * inputJoint.m_constantC1 + inputJoint.m_vel * inputJoint.m_constantC2;
	}
}


/*
void Controller::debugPrint(){
    //_UpdateArmThetas();
    
    //_ForwardKinematics();

    DEBUG_SERIAL.print("Pos X: ");
    DEBUG_SERIAL.println(m_eePosition.x);
    DEBUG_SERIAL.print("Pos Y: ");
    DEBUG_SERIAL.println(m_eePosition.y);
    DEBUG_SERIAL.print("Pos Z: ");
    DEBUG_SERIAL.println(m_eePosition.z);
    DEBUG_SERIAL.print("\n");
}

void Controller::Print() {
	//Test for _ComputerOutputToVelocity
	Serial.print("Vel 1: ");
	Serial.println(Joint1.m_vel);
	Serial.print("Vel 2: ");
	Serial.println(Joint2.m_vel);
	Serial.print("Vel 3: ");
	Serial.println(Joint3.m_vel);
	Serial.println(" ");


}
*/
