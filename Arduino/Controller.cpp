#include "Controller.h"
#include "ComputerConnector.h"
#include "Dynamics.h"
#include "Kinematics.h"
#include "DynamixelConnector.h"

Controller::Controller(){
	m_Joint1 = Joint(1, 10, 6.6, -180, 180, 10);
    m_Joint2 = Joint(2, 10, 22.0, 10, 10, 10);
    m_Joint3 = Joint(3, 10, 14.7, 10, 10, 10);
    m_Joint4 = Joint(4, 10, 10, 10, 10, 10);
    m_Joint5 = Joint(5, 10, 10, 10, 10, 10);

	ComputerConnector* CC = new ComputerConnector();
	Dynamics* Dyn = new Dynamics();
	Kinematics* Kin = new Kinematics();
	DynamixelConnector* DC = new DynamixelConnector();



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
	m_Joint1 = Joint(1, 10, 0.066, -180, 180, 10, MX64R);
    m_Joint2 = Joint(2, 10, 0.22, 10, 10, 10, MX106R);
    m_Joint3 = Joint(3, 10, 0.147, 10, 10, 10, MX64R);
    m_Joint4 = Joint(4, 10, 10, 10, 10, 10, MX28R);
    m_Joint5 = Joint(5, 10, 10, 10, 10, 10, MX28R);

	p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL, DIRECTION_PIN);
    p_dynamixel->begin(57600);
	 
    // Initial motor setup
    for (unsigned int i = 1; i < 6; i++)
    {
        p_dynamixel->torqueOff(i);
        p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, i, OperatingMode::OP_PWM);
        p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, i, 600);
        //p_dynamixel->torqueOn(i);
		
    }

    m_currentTime = millis();
}

Controller::~Controller(){
} 


void Controller::main(){
	
	CC->updateComputerData();


	

}

void Controller::_ComputerOutputToVelocity(bool emergencyStop, unsigned int controlMode, bool sign, unsigned int speed) {
	if (emergencyStop) {
		inputMotion.m_Vel1 = 0;
		inputMotion.m_Vel2 = 0;
		inputMotion.m_Vel3 = 0;
		return;
	}

	int direction = sign ? 1 : -1;


	//Control mode: 1 = base, 2 = in/out, 3 = up/down
	switch (controlMode){
	case 1:{
		inputMotion.m_Vel1 = direction*speed;
		inputMotion.m_Vel2 = 0;
		inputMotion.m_Vel3 = 0;

		inputMotion.currentSpaceType = JointSpace;
		break;
	}
	case 2:{
		inputMotion.m_Vel1 = direction*speed;
		inputMotion.m_Vel2 = 0;
		inputMotion.m_Vel3 = 0;

		inputMotion.currentSpaceType = CartesianSpace;
		break;
	}
	case 3: {
		inputMotion.m_Vel1 = 0;
		inputMotion.m_Vel2 = 0;
		inputMotion.m_Vel3 = direction*speed;

		inputMotion.currentSpaceType = CartesianSpace;
		break;
	}
	}
}

void Controller::_UpdateChain(){
	//_ComputerOutputToVelocity(); //Uncomment when computer output is available
    _ForwardKinematics();
    _InverseDynamics();
} 

void Controller::_UpdateAngles(){
	inputAngles.m_Theta1 = p_dynamixel->getPresentPosition(1,UNIT_RAW);
	inputAngles.m_Theta2 = p_dynamixel->getPresentPosition(2,UNIT_RAW);
	inputAngles.m_Theta3 = p_dynamixel->getPresentPosition(3,UNIT_RAW);
	inputAngles.m_Theta4 = p_dynamixel->getPresentPosition(4,UNIT_RAW);
	inputAngles.m_Theta5 = p_dynamixel->getPresentPosition(5,UNIT_RAW);
	
	inputAngles.currentUnitType = Raw; 
	_AngleConverter(Radians);
}

double Controller::_DifferentiationOperator(double currentValue, double previousValue){
	double differentiatedValue = 0.0;
	
	differentiatedValue = (currentValue - previousValue) / samplingTime;
	
	return differentiatedValue;
}

double Controller::_IntegrationOperator(double currentValue, double& inputIntegrationVal) {
	double integratedValue = inputIntegrationVal;
	
	integratedValue += currentValue * samplingTime;

	inputIntegrationVal = integratedValue;

	return integratedValue;
}

void Controller::_SpaceConverter(SpaceType desiredSpace){
	_AngleConverter(Radians);
	
	if(desiredSpace == inputMotion.currentSpaceType){
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
	velocityVector(0, 0) = inputMotion.m_Vel1;
	velocityVector(1, 0) = inputMotion.m_Vel2;
	velocityVector(2, 0) = inputMotion.m_Vel3;

	switch (desiredSpace){
	case JointSpace: {
		velocityVector = jacobianInverse * velocityVector;
	}
	case CartesianSpace: {
		velocityVector = jacobian * velocityVector;
	}
	}

	inputMotion.m_Vel1 = velocityVector(0,0);
	inputMotion.m_Vel2 = velocityVector(1,0);
	inputMotion.m_Vel3 = velocityVector(2,0);

	inputMotion.currentSpaceType = desiredSpace;
}

void Controller::_ForwardKinematics(){
	//_AngleConverter(Radians);
	m_eePosition.x = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.y = -sin(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.z = m_Joint1.m_length + m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
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
	inputMotion.m_Pos1 = _IntegrationOperator(inputMotion.m_Vel1, inputMotion.m_Pos1);
	inputMotion.m_Pos2 = _IntegrationOperator(inputMotion.m_Vel2, inputMotion.m_Pos2);
	inputMotion.m_Pos3 = _IntegrationOperator(inputMotion.m_Vel3, inputMotion.m_Pos3);

	inputMotion.m_Acc1 = _DifferentiationOperator(inputMotion.m_Vel1, prev_Vel1);
	inputMotion.m_Acc2 = _DifferentiationOperator(inputMotion.m_Vel2, prev_Vel2);
	inputMotion.m_Acc3 = _DifferentiationOperator(inputMotion.m_Vel3, prev_Vel3);

	prev_Vel1 = inputMotion.m_Vel1;
	prev_Vel2 = inputMotion.m_Vel2;
	prev_Vel3 = inputMotion.m_Vel3;


	m_Joint1.m_torque = (((0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) - 0.1121e-1) * pow(cos(inputMotion.m_Pos3), 0.2e1) - 0.1762e-1 * cos(inputMotion.m_Pos3) - 0.1581e-1 + 0.5e-3 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((0.5e-3 * sin(inputMotion.m_Pos2) + 0.1121e-1 * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos3) + 0.1762e-1 * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) + 0.1762e-1 * cos(inputMotion.m_Pos3) + 0.5606e-2 * pow(cos(inputMotion.m_Pos3), 0.2e1) + 0.2e-3 * sin(inputMotion.m_Pos3) + 0.2162e-1) * inputMotion.m_Acc1 + (0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos1) - 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos2) * inputMotion.m_Acc2 + ((0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) + 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (-0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos3) + 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) - 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) - 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1)) * pow(inputMotion.m_Vel1, 0.2e1) + ((((0.2242e-1 * sin(inputMotion.m_Pos3) + 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) + 0.10e-2) * cos(inputMotion.m_Pos3) + 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) + 0.3522e-1 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((0.2242e-1 * sin(inputMotion.m_Pos2) + 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos3), 0.2e1) + 0.3524e-1 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) - 0.10e-2 * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) + 0.3165e-1 * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) + (-0.5e-3 - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) - 0.1121e-1 * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos3) - 0.1761e-1 * sin(inputMotion.m_Pos3) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3)) * inputMotion.m_Vel2 + (((0.2242e-1 * sin(inputMotion.m_Pos3) + 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) + 0.5e-3) * cos(inputMotion.m_Pos3) + 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) + 0.1761e-1 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((0.2242e-1 * sin(inputMotion.m_Pos2) + 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos3), 0.2e1) + (0.1761e-1 * sin(inputMotion.m_Pos2) + 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos3) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) - 0.5e-3 * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) - 0.1121e-1 * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) + (0.2e-3 - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) - 0.1121e-1 * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos3) - 0.1761e-1 * sin(inputMotion.m_Pos3) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3)) * inputMotion.m_Vel3) * inputMotion.m_Vel1 + (0.8e-5 * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos2), 0.2e1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + (-0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos3) + 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) - 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1)) * pow(inputMotion.m_Vel2, 0.2e1) + ((-0.8e-5 * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1) + 0.6e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1)) * pow(cos(inputMotion.m_Pos2), 0.2e1) - 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) - 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) - 0.2e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + 0.4e-5 * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * inputMotion.m_Vel3 * inputMotion.m_Vel2 + 0.2e-5 * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) * pow(inputMotion.m_Vel3, 0.2e1) * cos(inputMotion.m_Pos3) * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3);
	m_Joint2.m_torque = (((-0.2e-4 * cos(inputMotion.m_Pos3) - 0.1e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) + 0.3e-4) * pow(cos(inputMotion.m_Pos1), 0.4e1) + (0.2e-4 * cos(inputMotion.m_Pos3) - 0.4e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) - 0.2e-4) * pow(cos(inputMotion.m_Pos1), 0.2e1) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) - 0.2e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) + (-0.2e-4 + 0.2e-4 * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.4e1) + (0.20e-4 - 0.2e-4 * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.2e1) + 0.2773e-1 + 0.1762e-1 * cos(inputMotion.m_Pos3)) * inputMotion.m_Acc2 + ((-0.4e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * pow(cos(inputMotion.m_Pos1), 0.2e1) - 0.1e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * pow(cos(inputMotion.m_Pos1), 0.4e1) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1)) * pow(cos(inputMotion.m_Pos2), 0.2e1) - 0.4e-5 * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos2) + 0.8808e-2 * cos(inputMotion.m_Pos3) + 0.6206e-2) * inputMotion.m_Vel3 + (0.4e-5 * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) - 0.2e-5 * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) * inputMotion.m_Acc1 + ((0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos3) + (-0.1121e-1 * sin(inputMotion.m_Pos3) - 0.5000e-3) * cos(inputMotion.m_Pos3) - 0.1762e-1 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((0.1e-4 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos1), 0.2e1) - 0.1121e-1 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2) - 0.1762e-1 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + (0.5000e-3 * sin(inputMotion.m_Pos3) - 0.1581e-1) * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) + (0.2500e-3 + 0.5606e-2 * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos3) + 0.8808e-2 * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel1, 0.2e1) + (((0.10e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1) + 0.4e-5 * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) - 0.30e-5 * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (0.70e-5 * sin(inputMotion.m_Pos1) - 0.4e-5 * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) - 0.5e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1)) * inputMotion.m_Vel2 + ((-0.3e-5 * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + 0.10e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1) - 0.5e-5 * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos2), 0.2e1) - 0.3e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) + (0.5e-5 * sin(inputMotion.m_Pos1) - 0.5e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1)) * inputMotion.m_Vel3) * inputMotion.m_Vel1 + (((-0.1e-4 * sin(inputMotion.m_Pos3) - 0.12e-4 * sin(inputMotion.m_Pos3) * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.4e1) + (0.24e-4 * sin(inputMotion.m_Pos3) + 0.2e-5 * sin(inputMotion.m_Pos3) * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.2e1) + 0.2e-5 * sin(inputMotion.m_Pos3) * cos(inputMotion.m_Pos3) - 0.2e-5 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((-0.18e-4 * sin(inputMotion.m_Pos2) + 0.2e-4 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) - 0.12e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos1), 0.4e1) + (0.2e-4 * sin(inputMotion.m_Pos2) - 0.2e-4 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos1), 0.2e1) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) + (0.2e-5 * sin(inputMotion.m_Pos3) * cos(inputMotion.m_Pos3) + 0.1e-4 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.4e1) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel2, 0.2e1) - 0.1762e-1 * inputMotion.m_Vel2 * inputMotion.m_Vel3 * sin(inputMotion.m_Pos3) + ((0.2e-5 * sin(inputMotion.m_Pos3) * cos(inputMotion.m_Pos3) - 0.12e-4 * cos(inputMotion.m_Pos3) * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3) + 0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + ((0.2e-5 * sin(inputMotion.m_Pos2) - 0.12e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2) - 0.4e-5 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos1), 0.4e1) + 0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) * pow(cos(inputMotion.m_Pos3), 0.2e1) + 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) - 0.8808e-2 * sin(inputMotion.m_Pos3) + 0.2e-5 * cos(inputMotion.m_Pos3) * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel3, 0.2e1) + 0.9962e0 * sin(inputMotion.m_Pos2) + 0.3930e0 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.3930e0 * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3);
	m_Joint3.m_torque = ((0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) - 0.1e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * pow(cos(inputMotion.m_Pos1), 0.4e1)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + 0.8808e-2 * cos(inputMotion.m_Pos3) + 0.6206e-2) * inputMotion.m_Acc2 + (0.6206e-2 + (0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) - 0.1e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * pow(cos(inputMotion.m_Pos1), 0.4e1)) * pow(cos(inputMotion.m_Pos2), 0.2e1)) * inputMotion.m_Acc3 + ((0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos3) + (-0.1121e-1 * sin(inputMotion.m_Pos3) - 0.2500e-3) * cos(inputMotion.m_Pos3) - 0.8808e-2 * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (-0.8808e-2 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.2500e-3 * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) - 0.1121e-1 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2) + 0.2e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) * pow(cos(inputMotion.m_Pos3), 0.2e1) + 0.5606e-2 * sin(inputMotion.m_Pos2)) * cos(inputMotion.m_Pos2) + (-0.1000e-3 + 0.5606e-2 * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos3) + 0.8808e-2 * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel1, 0.2e1) + (((-0.2e-5 * sin(inputMotion.m_Pos1) - 0.2e-5 * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos3) + 0.4e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (0.4e-5 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos3) - 0.3e-5 * sin(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) + (0.2e-5 * sin(inputMotion.m_Pos1) - 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1)) * inputMotion.m_Vel2 + ((0.4e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1) - 0.2e-5 * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1) * pow(cos(inputMotion.m_Pos2), 0.2e1) + 0.4e-5 * cos(inputMotion.m_Pos1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) * sin(inputMotion.m_Pos1) * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3) + (0.2e-5 * sin(inputMotion.m_Pos1) - 0.2e-5 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos1)) * cos(inputMotion.m_Pos1)) * inputMotion.m_Vel3) * inputMotion.m_Vel1 + (0.1e-4 * pow(cos(inputMotion.m_Pos2), 0.2e1) * cos(inputMotion.m_Pos3) * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3) + ((0.1e-4 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.1e-4 * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2)) * pow(cos(inputMotion.m_Pos1), 0.4e1) - 0.1e-4 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) + 0.8808e-2 * sin(inputMotion.m_Pos3) - 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3) + 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel2, 0.2e1) + ((-0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3) + 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3)) * pow(cos(inputMotion.m_Pos2), 0.2e1) + (0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) - 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3)) * cos(inputMotion.m_Pos2) - 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3) + 0.4e-5 * pow(cos(inputMotion.m_Pos1), 0.2e1) * sin(inputMotion.m_Pos3)) * inputMotion.m_Vel3 * inputMotion.m_Vel2 + (0.1e-4 * cos(inputMotion.m_Pos2) * pow(cos(inputMotion.m_Pos1), 0.4e1) * pow(cos(inputMotion.m_Pos3), 0.2e1) * sin(inputMotion.m_Pos2) + 0.1e-4 * pow(cos(inputMotion.m_Pos2), 0.2e1) * cos(inputMotion.m_Pos3) * pow(cos(inputMotion.m_Pos1), 0.4e1) * sin(inputMotion.m_Pos3)) * pow(inputMotion.m_Vel3, 0.2e1) + 0.3930e0 * sin(inputMotion.m_Pos2) * cos(inputMotion.m_Pos3) + 0.3930e0 * cos(inputMotion.m_Pos2) * sin(inputMotion.m_Pos3);
}

void _GetJointPWMConstants(Joint inputJoint) {

	

	switch(inputJoint.m_servoType) {
	case MX28R: {

		inputJoint.m_constantC2 = 115.3;

	}
	case MX64R: {

		inputJoint.m_constantC2 = 105.3;

	}
	case MX106R: {

		inputJoint.m_constantC2 = 160.6;
	}
	}
}

double _TorqueToPWM() {


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
	Serial.println(inputMotion.m_Vel1);
	Serial.print("Vel 2: ");
	Serial.println(inputMotion.m_Vel2);
	Serial.print("Vel 3: ");
	Serial.println(inputMotion.m_Vel3);
	Serial.println(" ");

}
