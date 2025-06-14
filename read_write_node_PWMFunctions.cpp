 // Copyright 2020 ROBOTIS CO., LTD.
 //
 // Licensed under the Apache License, Version 2.0 (the "License");
 // you may not use this file except in compliance with the License.
 // You may obtain a copy of the License at
 //
 //     http://www.apache.org/licenses/LICENSE-2.0
 //
 // Unless required by applicable law or agreed to in writing, software
 // distributed under the License is distributed on an "AS IS" BASIS,
 // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 // See the License for the specific language governing permissions and
 // limitations under the License.
 
 /*******************************************************************************
  * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
  * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
  * To test this example, please follow the commands below.
  *
  * Open terminal #1
  * $ roscore
  *
  * Open terminal #2
  * $ rosrun dynamixel_sdk_examples read_write_node
  *
  * Open terminal #3 (run one of below commands at a time)
  * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
  * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
  * $ rosservice call /get_position "id: 1"
  * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
  * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
  * $ rosservice call /get_position "id: 2"
  * 
  * 
  * $ rostopic pub -1 /set_operating_mode dynamixel_sdk_examples/SetOperatingMode "{id: 1, mode: 16}"   <- Message to set us to PWM MODE
  * 
  * $ rostopic pub /set_pwm_goal dynamixel_sdk_examples/SetPWMGoal "{id: 2, PWMGoal: 500}"   <- Message to set the PWM target (-885 to 885 representing -100% to 100%)
  *
  * Author: Zerom
 *******************************************************************************/
 
 #include <ros/ros.h>
 
 #include "std_msgs/String.h"
 #include "dynamixel_sdk_examples/GetPosition.h"
 #include "dynamixel_sdk_examples/SetPosition.h"
 #include "dynamixel_sdk/dynamixel_sdk.h"
 
 using namespace dynamixel;
 
 // Control table address
 #define ADDR_TORQUE_ENABLE    64
 #define ADDR_GOAL_POSITION    116
 #define ADDR_PRESENT_POSITION 132
 #define ADDR_OPERATING_MODE   11              //This address controls between velocity control, position control, extended position control and PWM Voltage control
 #define ADDR_GOAL_PWM         100              //This address is what our PWM target should be. We should continuously re-write this as we command our motors!
 
 // Protocol version
 #define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.
 
 // Default setting
 #define DXL1_ID               1               // DXL1 ID
 #define DXL2_ID               2               // DXL2 ID
 #define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
 #define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command
 
 PortHandler * portHandler;
 PacketHandler * packetHandler;
 
 bool getPresentPositionCallback(
   dynamixel_sdk_examples::GetPosition::Request & req,
   dynamixel_sdk_examples::GetPosition::Response & res)
 {
   uint8_t dxl_error = 0;
   int dxl_comm_result = COMM_TX_FAIL;
 
   // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
   int32_t position = 0;
 
   // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
   // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
   dxl_comm_result = packetHandler->read4ByteTxRx(
     portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
   if (dxl_comm_result == COMM_SUCCESS) {
     ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
     res.position = position;
     return true;
   } else {
     ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
     return false;
   }
 }
 

 void setOperatingModeCallback(const dynamixel_sdk_examples::SetOperatingMode::ConstPtr & msg)
 {
   uint8_t dxl_error = 0;
   int dxl_comm_result = COMM_TX_FAIL;
 
   // Operating mode Value of X series is 1 byte data.
   uint8_t id = static_cast<uint8_t>(msg->id); 
   uint8_t mode = static_cast<uint8_t>(msg->mode); // ensure 1 byte data
 
   // Write mode (length : 1 bytes)
   // When writing 1 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
   dxl_comm_result = packetHandler->write1ByteTxRx(
     portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);     ///Send the message to set OPERATING MODE address to the value in the message
   if (dxl_comm_result == COMM_SUCCESS) {
     ROS_INFO("setOperatingMode : [ID:%d] [MODE:%d]", id, mode);  ///If we get success message display info that we set the mode of the ID to what we did
   } else {
     ROS_ERROR("Failed to set Mode! Result: %d", dxl_comm_result);  ///If we dont get success message display the error
   }
 }

 void setPWMGoalCallback(const dynamixel_sdk_examples::SetPWMGoal::ConstPtr & msg)
 {
   uint8_t dxl_error = 0;
   int dxl_comm_result = COMM_TX_FAIL;
 
   // PWM limit Value of X series is 2 byte data (signed).
   uint8_t id = static_cast<uint8_t>(msg->id); 
   int16_t PWMGoal = static_cast<int16_t>(msg->PWMGoal); // Convert to signed 2 byte data
 
   // Write Value (from -885 to 885, representing -100% to 100%) (length : 2 bytes)
   dxl_comm_result = packetHandler->write2ByteTxRx(
     portHandler, id, ADDR_GOAL_PWM, PWMGoal, &dxl_error);     ///Send message of PWM goal to the motor (range: -885 to 885)
   if (dxl_comm_result == COMM_SUCCESS) {
     ROS_INFO("setPWM Goal : [ID:%d] [PWM Target (out of 885):%d]", id, PWMGoal);  ///If we get success message display info that we set the target PWM value of the ID to what we did
   } else {
     ROS_ERROR("Failed to set PWM Target! Result: %d", dxl_comm_result);  ///If we dont get success message display the error
   }
 }



 int main(int argc, char ** argv)
 {
   uint8_t dxl_error = 0;
   int dxl_comm_result = COMM_TX_FAIL;
 
   portHandler = PortHandler::getPortHandler(DEVICE_NAME);
   packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
 
   if (!portHandler->openPort()) {
     ROS_ERROR("Failed to open the port!");
     return -1;
   }
 
   if (!portHandler->setBaudRate(BAUDRATE)) {
     ROS_ERROR("Failed to set the baudrate!");
     return -1;
   }
 
   dxl_comm_result = packetHandler->write1ByteTxRx(
     portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
     return -1;
   }
 
   dxl_comm_result = packetHandler->write1ByteTxRx(
     portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
     return -1;
   }
 
   ros::init(argc, argv, "read_write_node");
   ros::NodeHandle nh;
   ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
   /////ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);          ////Commenting this out for now since I got rid of that function and dont know how to properly implement the new ones in main
   ros::spin();
 
   portHandler->closePort();
   return 0;
 }