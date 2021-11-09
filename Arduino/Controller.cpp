#include "Controller.h"


#include "DynamixelConnector.h"

Controller::Controller(){
	m_Joint1 = Joint(1, 10, 6.6, -180, 180, 10);
    m_Joint2 = Joint(2, 10, 22.0, 10, 10, 10);
    m_Joint3 = Joint(3, 10, 14.7, 10, 10, 10);
    m_Joint4 = Joint(4, 10, 10, 10, 10, 10);
    m_Joint5 = Joint(5, 10, 10, 10, 10, 10);

	ComputerConnector CC;
//	Kinemati


    //// Initial motor setup
    //for (unsigned int i = 1; i < 6; i++)
    //{
    //    p_dynamixel->torqueOff(i);
    //    p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, i, OperatingMode::OP_PWM);
    //    p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, i, 600);
    //    //p_dynamixel->torqueOn(i);
    //}
	
    //m_currentTime = millis();
}

Controller::~Controller(){
} 

void Controller::_ComputerOutputToVelocity(bool emergencyStop, unsigned int controlMode, bool sign, unsigned int speed) {
	if (emergencyStop) {
		inputVelocities.m_Vel1 = 0;
		inputVelocities.m_Vel2 = 0;
		inputVelocities.m_Vel3 = 0;
	}

	int direction = sign ? 1 : -1;


	//Control mode: 1 = base, 2 = in/out, 3 = up/down
	switch (controlMode){
	case 1:{
		inputVelocities.m_Vel1 = direction*speed;
		inputVelocities.m_Vel2 = 0;
		inputVelocities.m_Vel3 = 0;

		inputVelocities.currentSpaceType = JointSpace;
		break;
	}
	case 2:{
		inputVelocities.m_Vel1 = direction*speed;
		inputVelocities.m_Vel2 = 0;
		inputVelocities.m_Vel3 = 0;

		inputVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	case 3: {
		inputVelocities.m_Vel1 = 0;
		inputVelocities.m_Vel2 = 0;
		inputVelocities.m_Vel3 = direction*speed;

		inputVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	}
}

void Controller::_UpdateChain(){
	//_ComputerOutputToVelocity(); //Uncomment when computer output is available
    _ForwardKinematics();
    _InverseDynamics();
} 

double Controller::_CalculusOperator(OperationType desiredOperation, double currentValue, double& previousValue){
	switch (desiredOperation){
	case Differentiation:{
		return (currentValue - previousValue) / samplingTime;
	}
	case Integration:{
		
	}	
	}
	
	previousValue = currentValue;
}

void Controller::_SpaceConverter(SpaceType desiredSpace){
	if(desiredSpace == inputVelocities.currentSpaceType){
		return;
	}
	
	//Forward Jacobian:
	BLA::Matrix<3, 3> jacobian; 
	jacobian(0,0) = sin(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,1) = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,2) = -cos(inputAngles.m_Theta1) * m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(1,0) = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,1) = -sin(inputAngles.m_Theta1) * (m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,2) = -sin(inputAngles.m_Theta1) * m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,0) = 0;
	jacobian(2,1) = -m_Joint2.m_length * sin(inputAngles.m_Theta2) - m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,2) = -m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);

	//Inverse Jacobian:
	BLA::Matrix<3, 3> jacobianInverse = jacobian;
	bool is_nonsingular = Invert(jacobianInverse);

	//If the jacobian is singular - leave function 
	if(!is_nonsingular)
	{
		return;
	}

	BLA::Matrix<3,1> velocityVector;
	velocityVector(0, 0) = inputVelocities.m_Vel1;
	velocityVector(1, 0) = inputVelocities.m_Vel2;
	velocityVector(2, 0) = inputVelocities.m_Vel3;

	switch (desiredSpace){
	case JointSpace: {
		velocityVector = jacobianInverse * velocityVector;
	}
	case CartesianSpace: {
		velocityVector = jacobian * velocityVector;
	}
	}

	inputVelocities.m_Vel1 = velocityVector(0,0);
	inputVelocities.m_Vel2 = velocityVector(1,0);
	inputVelocities.m_Vel3 = velocityVector(2,0);

	inputVelocities.currentSpaceType = desiredSpace;
}

void Controller::_ForwardKinematics(){
	//_AngleConverter(Radians);
	m_eePosition.x = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.y = -sin(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.z = m_Joint1.m_length + m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
}

double Controller::_PID(double desiredValue, double currentValue){
	double Kp{ 1 }, Ki{ 1 }, Kd{1};
	double error = desiredValue - currentValue;
	
	m_proportional = Kp * error;
	m_integral += Ki * (error * samplingTime);
	m_derivative = Kd * ((error - m_lastError) / samplingTime);
	
	m_lastError = error; 

	return m_proportional + m_integral + m_derivative;
}


void Controller::_InverseDynamics(){


}

void Controller::debugPrint(){
    //_UpdateArmThetas();
    
    _ForwardKinematics();
    //DEBUG_SERIAL.print("eeX: ");
    //DEBUG_SERIAL.println(m_eePosition.x);
    //DEBUG_SERIAL.print("eeY: ");
    //DEBUG_SERIAL.println(m_eePosition.y);
    //DEBUG_SERIAL.print("eeZ: ");
    //DEBUG_SERIAL.println(m_eePosition.z);
    //DEBUG_SERIAL.print("\n");    
    
}