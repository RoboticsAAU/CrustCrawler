#include "Controller.h"
#include "ComputerConnector.h"
#include "Dynamics.h"
#include "Kinematics.h"
#include "DynamixelConnector.h"

#include "CrustCrawlerData.h"


Controller::Controller(){

	ComputerConnector* computerConnector = new ComputerConnector();
	Dynamics* dynamics = new Dynamics();
	Kinematics* kinematics = new Kinematics();
	DynamixelConnector* dynamixelConnector = new DynamixelConnector();
}


Controller::~Controller(){
} 


void Controller::main(){
	computerConnector->updateComputerData();

}

void Controller::_ComputerOutputToVelocity(unsigned int controlMode, bool sign, unsigned int speed) {
	Joint1.m_vel = 0;
	Joint2.m_vel = 0;
	Joint3.m_vel = 0;
	Joint4.m_vel = 0;
	Joint5.m_vel = 0;

	int direction = sign ? 1 : -1;


	//Control mode: 1 = base, 2 = in/out, 3 = up/down
	switch (controlMode){
	case 0:{
		int fingerSpeed = 20; //Remember to change fittingly
		Joint4.m_vel = -direction*fingerSpeed;
		Joint5.m_vel = direction *fingerSpeed;

		inputMotion.currentSpaceType = JointSpace;
		break;
	}
	case 1:{
		Joint1.m_vel = direction*speed;

		inputMotion.currentSpaceType = JointSpace;
		break;
	}
	case 2:{
		Joint1.m_vel = direction*speed;

		inputMotion.currentSpaceType = CartesianSpace;
		break;
	}
	case 3: {
		Joint3.m_vel = direction*speed;

		inputMotion.currentSpaceType = CartesianSpace;
		break;
	}
	case 4: {
		break;
	}
	}
}

//void Controller::_ComputerOutputToVelocity(bool emergencyStop, unsigned int controlMode, bool sign, unsigned int speed) {
//	if (emergencyStop) {
//		Joint1.m_vel = 0;
//		Joint2.m_vel = 0;
//		Joint3.m_vel = 0;
//		return;
//	}
//
//	int direction = sign ? 1 : -1;
//
//
//	//Control mode: 1 = base, 2 = in/out, 3 = up/down
//	switch (controlMode){
//	case 1:{
//		Joint1.m_vel = direction*speed;
//		Joint2.m_vel = 0;
//		Joint3.m_vel = 0;
//
//		inputMotion.currentSpaceType = JointSpace;
//		break;
//	}
//	case 2:{
//		Joint1.m_vel = direction*speed;
//		Joint2.m_vel = 0;
//		Joint3.m_vel = 0;
//
//		inputMotion.currentSpaceType = CartesianSpace;
//		break;
//	}
//	case 3: {
//		Joint1.m_vel = 0;
//		Joint2.m_vel = 0;
//		Joint3.m_vel = direction*speed;
//
//		inputMotion.currentSpaceType = CartesianSpace;
//		break;
//	}
//	}
//}


/*void Controller::_UpdateChain(){
	//_ComputerOutputToVelocity(); //Uncomment when computer output is available
    _ForwardKinematics();
    _InverseDynamics();
} */


double Controller::_DifferentiationOperator(double currentValue, double previousValue){
	//double differentiatedValue = 0.0;
	//differentiatedValue = (currentValue - previousValue) / samplingTime;
	//return differentiatedValue;

	return (currentValue - previousValue) / samplingTime;
}

double Controller::_IntegrationOperator(double currentValue, double inputIntegrationVal) {
	//double integratedValue = inputIntegrationVal;
	//integratedValue += currentValue * samplingTime;
	//inputIntegrationVal = integratedValue;
	//return integratedValue;
	inputIntegrationVal += currentValue * samplingTime;

	return inputIntegrationVal;
}

void Controller::_SpaceConverter(SpaceType desiredSpace){
	//_AngleConverter(Radians);
	
	if(desiredSpace == inputMotion.currentSpaceType){
		return;
	}
	
	//Forward Jacobian:
	BLA::Matrix<3, 3> jacobian;
	jacobian(0,0) = sin(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,1) = -cos(inputAngles.m_Theta1) * (Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,2) = -cos(inputAngles.m_Theta1) * Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(1,0) = -cos(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,1) = -sin(inputAngles.m_Theta1) * (Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,2) = -sin(inputAngles.m_Theta1) * Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,0) = 0;
	jacobian(2,1) = -Joint2.m_length * sin(inputAngles.m_Theta2) - Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,2) = -Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);

	//Inverse Jacobian:
	BLA::Matrix<3, 3> jacobianInverse = jacobian;
	bool is_nonsingular = Invert(jacobianInverse);

	//for (int i = 0; i < 3; i++) {
	//	for (int j = 0; j < 3; j++) {
	//		Serial.print(jacobianInverse(i, j));
	//		Serial.print(" ");
	//	}
	//	Serial.println(" ");
	//}
	//Serial.println(" ");


	//If the jacobian is singular. Wait implementation until further understanding.
	/*if(!is_nonsingular)
	{
		
	}*/

	BLA::Matrix<3,1> velocityVector;
	velocityVector(0, 0) = Joint1.m_vel;
	velocityVector(1, 0) = Joint2.m_vel;
	velocityVector(2, 0) = Joint3.m_vel;

	switch (desiredSpace){
	case JointSpace: {
		velocityVector = jacobianInverse * velocityVector;
	}
	case CartesianSpace: {
		velocityVector = jacobian * velocityVector;
	}
	}

	Joint1.m_vel = velocityVector(0,0);
	Joint2.m_vel = velocityVector(1,0);
	Joint3.m_vel = velocityVector(2,0);

	inputMotion.currentSpaceType = desiredSpace;
}

void Controller::_ForwardKinematics(){
	//_AngleConverter(Radians);
	m_eePosition.x = -cos(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.y = -sin(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.z = Joint1.m_length + Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
}

double Controller::_PID(double desiredValue, double currentValue){
	double Kp{ 0.7 }, Ki{ 0.01 }, Kd{ 0.1 };
	double error = desiredValue - currentValue;
	
	m_proportional = Kp * error;
	m_integral = Ki * _IntegrationOperator(error, m_integral);
	m_derivative = Kd * _DifferentiationOperator(error, m_lastError);
	
	//m_integral += Ki * (error * samplingTime);
	//m_derivative = Kd * ((error - m_lastError) / samplingTime);
	
	m_lastError = error;

	return (m_proportional + m_integral + m_derivative);
}


void Controller::_InverseDynamics(){
	Joint1.m_pos = _IntegrationOperator(Joint1.m_vel, Joint1.m_pos);
	Joint2.m_pos = _IntegrationOperator(Joint2.m_vel, Joint2.m_pos);
	Joint3.m_pos = _IntegrationOperator(Joint3.m_vel, Joint3.m_pos);

	Joint1.m_acc = _DifferentiationOperator(Joint1.m_vel, prevVel1);
	Joint2.m_acc = _DifferentiationOperator(Joint2.m_vel, prevVel2);
	Joint3.m_acc = _DifferentiationOperator(Joint3.m_vel, prevVel3);

	prevVel1 = Joint1.m_vel;
	prevVel2 = Joint2.m_vel;
	prevVel3 = Joint3.m_vel;


	Joint1.m_torque = (((0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) - 0.1121e-1) * pow(cos(Joint3.m_pos), 0.2e1) - 0.1762e-1 * cos(Joint3.m_pos) - 0.1581e-1 + 0.5e-3 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((0.5e-3 * sin(Joint2.m_pos) + 0.1121e-1 * sin(Joint2.m_pos) * sin(Joint3.m_pos)) * cos(Joint3.m_pos) + 0.1762e-1 * sin(Joint2.m_pos) * sin(Joint3.m_pos)) * cos(Joint2.m_pos) + 0.1762e-1 * cos(Joint3.m_pos) + 0.5606e-2 * pow(cos(Joint3.m_pos), 0.2e1) + 0.2e-3 * sin(Joint3.m_pos) + 0.2162e-1) * Joint1.m_acc + (0.4e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * cos(Joint3.m_pos) * sin(Joint1.m_pos) - 0.2e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint1.m_pos)) * cos(Joint2.m_pos) * Joint2.m_acc + ((0.2e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) + 0.4e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) * cos(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + (-0.4e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint1.m_pos) * sin(Joint3.m_pos) + 0.2e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * cos(Joint3.m_pos) * sin(Joint1.m_pos) * sin(Joint3.m_pos)) * cos(Joint2.m_pos) - 0.4e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) * cos(Joint3.m_pos) - 0.2e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos)) * pow(Joint1.m_vel, 0.2e1) + ((((0.2242e-1 * sin(Joint3.m_pos) + 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) + 0.10e-2) * cos(Joint3.m_pos) + 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) + 0.3522e-1 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((0.2242e-1 * sin(Joint2.m_pos) + 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint3.m_pos), 0.2e1) + 0.3524e-1 * sin(Joint2.m_pos) * cos(Joint3.m_pos) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) - 0.10e-2 * sin(Joint2.m_pos) * sin(Joint3.m_pos) + 0.3165e-1 * sin(Joint2.m_pos)) * cos(Joint2.m_pos) + (-0.5e-3 - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) - 0.1121e-1 * sin(Joint3.m_pos)) * cos(Joint3.m_pos) - 0.1761e-1 * sin(Joint3.m_pos) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos)) * Joint2.m_vel + (((0.2242e-1 * sin(Joint3.m_pos) + 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) + 0.5e-3) * cos(Joint3.m_pos) + 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) + 0.1761e-1 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((0.2242e-1 * sin(Joint2.m_pos) + 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint3.m_pos), 0.2e1) + (0.1761e-1 * sin(Joint2.m_pos) + 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos)) * cos(Joint3.m_pos) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) - 0.5e-3 * sin(Joint2.m_pos) * sin(Joint3.m_pos) - 0.1121e-1 * sin(Joint2.m_pos)) * cos(Joint2.m_pos) + (0.2e-3 - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) - 0.1121e-1 * sin(Joint3.m_pos)) * cos(Joint3.m_pos) - 0.1761e-1 * sin(Joint3.m_pos) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos)) * Joint3.m_vel) * Joint1.m_vel + (0.8e-5 * cos(Joint1.m_pos) * pow(cos(Joint2.m_pos), 0.2e1) * sin(Joint1.m_pos) * cos(Joint3.m_pos) + (-0.4e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint1.m_pos) * sin(Joint3.m_pos) + 0.2e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * cos(Joint3.m_pos) * sin(Joint1.m_pos) * sin(Joint3.m_pos)) * cos(Joint2.m_pos) - 0.4e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) * cos(Joint3.m_pos) + 0.2e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos)) * pow(Joint2.m_vel, 0.2e1) + ((-0.8e-5 * cos(Joint1.m_pos) * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos) + 0.6e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) * cos(Joint3.m_pos) + 0.4e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) - 0.4e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint1.m_pos) * cos(Joint2.m_pos) * sin(Joint3.m_pos) - 0.2e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) - 0.2e-5 * cos(Joint1.m_pos) * sin(Joint1.m_pos) * cos(Joint3.m_pos) + 0.4e-5 * cos(Joint1.m_pos) * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * Joint3.m_vel * Joint2.m_vel + 0.2e-5 * sin(Joint1.m_pos) * cos(Joint2.m_pos) * pow(Joint3.m_vel, 0.2e1) * cos(Joint3.m_pos) * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint3.m_pos);
	Joint2.m_torque = (((-0.2e-4 * cos(Joint3.m_pos) - 0.1e-4 * pow(cos(Joint3.m_pos), 0.2e1) + 0.3e-4) * pow(cos(Joint1.m_pos), 0.4e1) + (0.2e-4 * cos(Joint3.m_pos) - 0.4e-5 * pow(cos(Joint3.m_pos), 0.2e1) - 0.2e-4) * pow(cos(Joint1.m_pos), 0.2e1) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1)) * pow(cos(Joint2.m_pos), 0.2e1) + (0.2e-4 * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint2.m_pos) * sin(Joint3.m_pos) - 0.2e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) * sin(Joint3.m_pos)) * cos(Joint2.m_pos) + (-0.2e-4 + 0.2e-4 * cos(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) + (0.20e-4 - 0.2e-4 * cos(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.2e1) + 0.2773e-1 + 0.1762e-1 * cos(Joint3.m_pos)) * Joint2.m_acc + ((-0.4e-5 * pow(cos(Joint3.m_pos), 0.2e1) * pow(cos(Joint1.m_pos), 0.2e1) - 0.1e-4 * pow(cos(Joint3.m_pos), 0.2e1) * pow(cos(Joint1.m_pos), 0.4e1) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1)) * pow(cos(Joint2.m_pos), 0.2e1) - 0.4e-5 * cos(Joint2.m_pos) * sin(Joint3.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint2.m_pos) + 0.8808e-2 * cos(Joint3.m_pos) + 0.6206e-2) * Joint3.m_vel + (0.4e-5 * sin(Joint1.m_pos) * sin(Joint2.m_pos) * cos(Joint3.m_pos) - 0.2e-5 * sin(Joint1.m_pos) * sin(Joint2.m_pos)) * cos(Joint1.m_pos) * cos(Joint2.m_pos) * Joint1.m_acc + ((0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) * cos(Joint3.m_pos) * sin(Joint3.m_pos) + (-0.1121e-1 * sin(Joint3.m_pos) - 0.5000e-3) * cos(Joint3.m_pos) - 0.1762e-1 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((0.1e-4 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint1.m_pos), 0.2e1) - 0.1121e-1 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos) - 0.1762e-1 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + (0.5000e-3 * sin(Joint3.m_pos) - 0.1581e-1) * sin(Joint2.m_pos)) * cos(Joint2.m_pos) + (0.2500e-3 + 0.5606e-2 * sin(Joint3.m_pos)) * cos(Joint3.m_pos) + 0.8808e-2 * sin(Joint3.m_pos)) * pow(Joint1.m_vel, 0.2e1) + (((0.10e-4 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos) + 0.4e-5 * sin(Joint1.m_pos) * cos(Joint3.m_pos) - 0.30e-5 * sin(Joint1.m_pos)) * cos(Joint1.m_pos) * pow(cos(Joint2.m_pos), 0.2e1) + (0.70e-5 * sin(Joint1.m_pos) - 0.4e-5 * sin(Joint1.m_pos) * cos(Joint3.m_pos) - 0.5e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * cos(Joint1.m_pos)) * Joint2.m_vel + ((-0.3e-5 * sin(Joint1.m_pos) * cos(Joint3.m_pos) + 0.10e-4 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos) - 0.5e-5 * sin(Joint1.m_pos)) * cos(Joint1.m_pos) * pow(cos(Joint2.m_pos), 0.2e1) - 0.3e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint1.m_pos) * cos(Joint2.m_pos) * sin(Joint3.m_pos) + (0.5e-5 * sin(Joint1.m_pos) - 0.5e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * cos(Joint1.m_pos)) * Joint3.m_vel) * Joint1.m_vel + (((-0.1e-4 * sin(Joint3.m_pos) - 0.12e-4 * sin(Joint3.m_pos) * cos(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) + (0.24e-4 * sin(Joint3.m_pos) + 0.2e-5 * sin(Joint3.m_pos) * cos(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.2e1) + 0.2e-5 * sin(Joint3.m_pos) * cos(Joint3.m_pos) - 0.2e-5 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((-0.18e-4 * sin(Joint2.m_pos) + 0.2e-4 * sin(Joint2.m_pos) * cos(Joint3.m_pos) - 0.12e-4 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) + (0.2e-4 * sin(Joint2.m_pos) - 0.2e-4 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint1.m_pos), 0.2e1) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * cos(Joint2.m_pos) + (0.2e-5 * sin(Joint3.m_pos) * cos(Joint3.m_pos) + 0.1e-4 * sin(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos)) * pow(Joint2.m_vel, 0.2e1) - 0.1762e-1 * Joint2.m_vel * Joint3.m_vel * sin(Joint3.m_pos) + ((0.2e-5 * sin(Joint3.m_pos) * cos(Joint3.m_pos) - 0.12e-4 * cos(Joint3.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos) + 0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) * cos(Joint3.m_pos) * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + ((0.2e-5 * sin(Joint2.m_pos) - 0.12e-4 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos) - 0.4e-5 * sin(Joint2.m_pos) * cos(Joint3.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) + 0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) * pow(cos(Joint3.m_pos), 0.2e1) + 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * cos(Joint2.m_pos) - 0.8808e-2 * sin(Joint3.m_pos) + 0.2e-5 * cos(Joint3.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos)) * pow(Joint3.m_vel, 0.2e1) + 0.9962e0 * sin(Joint2.m_pos) + 0.3930e0 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.3930e0 * cos(Joint2.m_pos) * sin(Joint3.m_pos);
	Joint3.m_torque = ((0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) - 0.1e-4 * pow(cos(Joint3.m_pos), 0.2e1) * pow(cos(Joint1.m_pos), 0.4e1)) * pow(cos(Joint2.m_pos), 0.2e1) + 0.8808e-2 * cos(Joint3.m_pos) + 0.6206e-2) * Joint2.m_acc + (0.6206e-2 + (0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) - 0.1e-4 * pow(cos(Joint3.m_pos), 0.2e1) * pow(cos(Joint1.m_pos), 0.4e1)) * pow(cos(Joint2.m_pos), 0.2e1)) * Joint3.m_acc + ((0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) * cos(Joint3.m_pos) * sin(Joint3.m_pos) + (-0.1121e-1 * sin(Joint3.m_pos) - 0.2500e-3) * cos(Joint3.m_pos) - 0.8808e-2 * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + (-0.8808e-2 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.2500e-3 * sin(Joint2.m_pos) * sin(Joint3.m_pos) - 0.1121e-1 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos) + 0.2e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) * pow(cos(Joint3.m_pos), 0.2e1) + 0.5606e-2 * sin(Joint2.m_pos)) * cos(Joint2.m_pos) + (-0.1000e-3 + 0.5606e-2 * sin(Joint3.m_pos)) * cos(Joint3.m_pos) + 0.8808e-2 * sin(Joint3.m_pos)) * pow(Joint1.m_vel, 0.2e1) + (((-0.2e-5 * sin(Joint1.m_pos) - 0.2e-5 * sin(Joint1.m_pos) * cos(Joint3.m_pos) + 0.4e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * cos(Joint1.m_pos) * pow(cos(Joint2.m_pos), 0.2e1) + (0.4e-5 * sin(Joint2.m_pos) * cos(Joint3.m_pos) * sin(Joint1.m_pos) * sin(Joint3.m_pos) - 0.3e-5 * sin(Joint1.m_pos) * sin(Joint2.m_pos) * sin(Joint3.m_pos)) * cos(Joint1.m_pos) * cos(Joint2.m_pos) + (0.2e-5 * sin(Joint1.m_pos) - 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * cos(Joint1.m_pos)) * Joint2.m_vel + ((0.4e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos) - 0.2e-5 * sin(Joint1.m_pos)) * cos(Joint1.m_pos) * pow(cos(Joint2.m_pos), 0.2e1) + 0.4e-5 * cos(Joint1.m_pos) * sin(Joint2.m_pos) * cos(Joint3.m_pos) * sin(Joint1.m_pos) * cos(Joint2.m_pos) * sin(Joint3.m_pos) + (0.2e-5 * sin(Joint1.m_pos) - 0.2e-5 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint1.m_pos)) * cos(Joint1.m_pos)) * Joint3.m_vel) * Joint1.m_vel + (0.1e-4 * pow(cos(Joint2.m_pos), 0.2e1) * cos(Joint3.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos) + ((0.1e-4 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.1e-4 * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos)) * pow(cos(Joint1.m_pos), 0.4e1) - 0.1e-4 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) * cos(Joint3.m_pos)) * cos(Joint2.m_pos) + 0.8808e-2 * sin(Joint3.m_pos) - 0.4e-5 * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos) + 0.4e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos)) * pow(Joint2.m_vel, 0.2e1) + ((-0.4e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos) + 0.4e-5 * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos)) * pow(cos(Joint2.m_pos), 0.2e1) + (0.4e-5 * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint2.m_pos) * cos(Joint3.m_pos) - 0.4e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint2.m_pos) * cos(Joint3.m_pos)) * cos(Joint2.m_pos) - 0.4e-5 * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos) + 0.4e-5 * pow(cos(Joint1.m_pos), 0.2e1) * sin(Joint3.m_pos)) * Joint3.m_vel * Joint2.m_vel + (0.1e-4 * cos(Joint2.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * pow(cos(Joint3.m_pos), 0.2e1) * sin(Joint2.m_pos) + 0.1e-4 * pow(cos(Joint2.m_pos), 0.2e1) * cos(Joint3.m_pos) * pow(cos(Joint1.m_pos), 0.4e1) * sin(Joint3.m_pos)) * pow(Joint3.m_vel, 0.2e1) + 0.3930e0 * sin(Joint2.m_pos) * cos(Joint3.m_pos) + 0.3930e0 * cos(Joint2.m_pos) * sin(Joint3.m_pos);
}


void Controller::_GetJointPWMConstants(Joint& inputJoint) {
	_SpaceConverter(JointSpace);


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


void Controller::_TorqueToPWM(Joint& inputJoint) {
	_SpaceConverter(JointSpace);

	// Getting the joint constants for each joint
	_GetJointPWMConstants(inputJoint);


	inputJoint.m_PWM = inputJoint.m_torque * inputJoint.m_constantC1 + inputJoint.m_vel * inputJoint.m_constantC2;

}

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
