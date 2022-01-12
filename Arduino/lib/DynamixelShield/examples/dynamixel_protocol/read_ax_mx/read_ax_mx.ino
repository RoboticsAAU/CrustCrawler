/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

//Please see eManual Control Table section of your DYNAMIXEL.
//This example is written for DYNAMIXEL AX & MX series with Protocol 1.0.
//For MX 2.0 with Protocol 2.0, refer to write_x.ino example.
#define ID_ADDR                   3
#define ID_ADDR_LEN               1
#define BAUDRATE_ADDR             4
#define BAUDRATE_ADDR_LEN         1
#define PRESENT_POSITION_ADDR     36
#define PRESENT_POSITION_ADDR_LEN  2
#define TIMEOUT 10    //default communication timeout 10ms

uint8_t returned_id = 0;
uint8_t returned_baudrate = 0;
uint16_t present_position = 0;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Refer to eManual for more details.");
  DEBUG_SERIAL.println("https://emanual.robotis.com/docs/en/dxl/");
  DEBUG_SERIAL.print("Read for PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);
  DEBUG_SERIAL.print(", ID ");
  DEBUG_SERIAL.println(DXL_ID);
  
  // Read DYNAMIXEL ID
  dxl.read(DXL_ID, ID_ADDR, ID_ADDR_LEN, (uint8_t*)&returned_id, sizeof(returned_id), TIMEOUT);
  DEBUG_SERIAL.print("ID : ");
  DEBUG_SERIAL.println(returned_id);
  delay(100);
  // Read DYNAMIXEL Baudrate
  dxl.read(DXL_ID, BAUDRATE_ADDR, BAUDRATE_ADDR_LEN, (uint8_t*)&returned_baudrate, sizeof(returned_baudrate), TIMEOUT);
  DEBUG_SERIAL.print("Baud Rate : ");
  DEBUG_SERIAL.println(returned_baudrate);
  delay(100);
  // Read DYNAMIXEL Present Position
  dxl.read(DXL_ID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN, (uint8_t*)&present_position, sizeof(present_position), TIMEOUT);
  DEBUG_SERIAL.print("Present Position : ");
  DEBUG_SERIAL.println(present_position);
}

void loop() {
  // put your main code here, to run repeatedly:
}
