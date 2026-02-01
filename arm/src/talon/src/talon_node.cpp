#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <messages/msg/key_state.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/talon_status.hpp"
#include "utils/utils.hpp"
#include <cmath>

#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/** @file
 * @brief Node controlling one Talon motor 
 * 
 * This node receives information published by the logic node,
 * then transforms the data received into movement by the motor
 * controlled by the Talon instance.  The topics that the node
 * subscribes to are as follows:
 * \li \b speed_topic
 * \li \b STOP
 * \li \b GO
 * 
 * The \b speed_topic topic is either \b drive_left_speed
 * or \b drive_right_speed as defined in the parameters set in
 * the launch file.  To read more about the logic node or the
 * launch file
 * \see logic_node.cpp
 * \see launch.py
 * 
 * The topics being published are as follows:
 * \li \b info_topic
 * 
 * This string has the general form talon_{motorNumber}_info and
 * is defined by the user in the launch file.  To read more about
 * the launch file,
 * \see launch.py
 * 
 * */


rclcpp::Node::SharedPtr nodeHandle;
bool printData = false;
int motorNumber = 0;
bool usePosition = false;

TalonSRX* talonSRX;
bool TEMP_DISABLE = false;
bool GO = true;

/** @brief Speed Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of drive_left_speed or
 * drive_right_speed.  This function takes the data
 * from the topic and sets the motor to the speed
 * specified.
 * @param speed
 * @return void
 * */
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;
	talonSRX->Set(ControlMode::PercentOutput, speed->data);
	usePosition = false;
}

void positionCallback(const std_msgs::msg::Int32::SharedPtr position){
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"Position---------->>> %d ", position->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;
	talonSRX->Set(ControlMode::Position, position->data);
	usePosition = true;
}

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");
	//int success;

	motorNumber = utils::getParameter<int>(nodeHandle, "motor_number", 1);
	int portNumber = utils::getParameter<int>(nodeHandle, "diagnostics_port", 1);
	//c_SetPhoenixDiagnosticsStartTime(-1); //Disables the Phoenix Diagnostics server, but does not allow the Talons to run
	c_Phoenix_Diagnostics_Create1(portNumber);  //Creates a Phoenix Diagnostics server with the port specified
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	std::string infoTopic = utils::getParameter<std::string>(nodeHandle, "info_topic", "unset");
	std::string speedTopic = utils::getParameter<std::string>(nodeHandle, "speed_topic", "unset");
	std::string positionTopic = utils::getParameter<std::string>(nodeHandle, "position_topic", "unset");

	bool invertMotor = utils::getParameter<bool>(nodeHandle, "invert_motor", false);
	double kP = utils::getParameter<double>(nodeHandle, "kP", 1.0);
	double kI = utils::getParameter<double>(nodeHandle, "kI", 0.0);
	double kD = utils::getParameter<double>(nodeHandle, "kD", 0.0);
	double kF = utils::getParameter<double>(nodeHandle, "kF", 0.0);
	printData = utils::getParameter<bool>(nodeHandle, "print_data", false);
	std::string can_interface = utils::getParameter<std::string>(nodeHandle, "can_interface", "can0");

	if(can_interface != "can0")
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());
	RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	//int kSlotIdx=0;
	talonSRX=new TalonSRX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	talonSRX->SetInverted(invertMotor);
	talonSRX->SelectProfileSlot(0,0);
	//talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, kTimeoutMs);
	talonSRX->SetSensorPhase(true);
	talonSRX->ConfigClosedloopRamp(2);
	talonSRX->ConfigNominalOutputForward(0, kTimeoutMs);
	talonSRX->ConfigNominalOutputReverse(0, kTimeoutMs);
	talonSRX->ConfigPeakOutputForward(1, kTimeoutMs);
	talonSRX->ConfigPeakOutputReverse(-1, kTimeoutMs);
	talonSRX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonSRX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonSRX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonSRX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonSRX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

	talonSRX->Set(ControlMode::PercentOutput, 0.0);
	talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 30);
	talonSRX->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 10, 10);

	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	TalonSRXConfiguration allConfigs;

	messages::msg::TalonStatus talonStatus;
	auto talonStatusPublisher=nodeHandle->create_publisher<messages::msg::TalonStatus>(infoTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);
	auto positionSubscriber=nodeHandle->create_subscription<std_msgs::msg::Int32>(positionTopic.c_str(),1,positionCallback);

	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(100);
	auto start2 = std::chrono::high_resolution_clock::now();
	auto start = std::chrono::high_resolution_clock::now();
	float maxCurrent = 0.0;
	double busVoltage = 0.0;

	int counter = 0;

	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 100){

			int deviceID=talonSRX->GetDeviceID();
			busVoltage=talonSRX->GetBusVoltage();
			double outputCurrent=talonSRX->GetOutputCurrent();
			bool isInverted=talonSRX->GetInverted();
			double motorOutputVoltage=talonSRX->GetMotorOutputVoltage();
			double motorOutputPercent=talonSRX->GetMotorOutputPercent();
			double temperature=talonSRX->GetTemperature();				
			int sensorPosition0=talonSRX->GetSelectedSensorPosition(0);
			int sensorVelocity0=talonSRX->GetSelectedSensorVelocity(0);
			int closedLoopError0=talonSRX->GetClosedLoopError(0);
			double integralAccumulator0=talonSRX->GetIntegralAccumulator(0);
			double errorDerivative0=talonSRX->GetErrorDerivative(0);
			RCLCPP_INFO(nodeHandle->get_logger(),"Sensor Position: %d", sensorPosition0);
		
			talonStatus.device_id=deviceID;	
			talonStatus.bus_voltage=busVoltage;
			talonStatus.output_current=outputCurrent;
			talonStatus.output_voltage=motorOutputVoltage;
			talonStatus.output_percent=motorOutputPercent;
			talonStatus.temperature=temperature;
			talonStatus.sensor_position=sensorPosition0;
			talonStatus.sensor_velocity=sensorVelocity0;
			talonStatus.closed_loop_error=closedLoopError0;
			talonStatus.integral_accumulator=integralAccumulator0;
			talonStatus.error_derivative=errorDerivative0;
			talonStatus.temp_disable = TEMP_DISABLE;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			talonStatus.max_current = maxCurrent;
			talonStatusPublisher->publish(talonStatus);
        	start = std::chrono::high_resolution_clock::now();
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}

