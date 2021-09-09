#include <functional>
#include <thread>
#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "am_extruder_msg/msg/extruder_command.hpp"
#include "am_extruder_msg/msg/extruder_stepping_state.hpp"
#include "am_extruder_msg/msg/extruder_temperature_state.hpp"

#include "RS-232/rs232.h"

#define EXTRUDER_LOGGER_NAME "AMExtruderCom"

class AMExtruderCom : public rclcpp::Node
{
public:
	AMExtruderCom(std::string com_port_name, int baudrate);
	~AMExtruderCom();
private:
	void processByte();
	void readSerial();
	void onCommandMsg(const am_extruder_msg::msg::ExtruderCommand::SharedPtr msg);
	rclcpp::Publisher<am_extruder_msg::msg::ExtruderTemperatureState>::SharedPtr extruder_heater_state_publisher;
	rclcpp::Publisher<am_extruder_msg::msg::ExtruderSteppingState>::SharedPtr extruder_mover_state_publisher;
	std::thread* reader_thread;
	std::atomic<bool> is_running;

	rclcpp::Subscription<am_extruder_msg::msg::ExtruderCommand>::SharedPtr extruder_command_subscriber;
	std::string com_port_name;
	int com_port_number;
	int baudrate;
};

AMExtruderCom::AMExtruderCom(std::string com_port_name, int baudrate)
	: Node("am_hw_com"), is_running(false)
{
	this->declare_parameter<std::string>("serial_port", com_port_name);
	this->declare_parameter<std::string>("baud_rate", std::to_string(baudrate));

	this->extruder_heater_state_publisher = this->create_publisher<am_extruder_msg::msg::ExtruderTemperatureState>(
		"am_extruder_temperature_state", 5);
	this->extruder_mover_state_publisher = this->create_publisher<am_extruder_msg::msg::ExtruderSteppingState>(
		"am_extruder_stepping_state", 5);

	this->reader_thread = new std::thread(&AMExtruderCom::readSerial, this);
	this->reader_thread->detach();

	this->extruder_command_subscriber = this->create_subscription<am_extruder_msg::msg::ExtruderCommand>(
		"am_extruder_command", 10, std::bind(&AMExtruderCom::onCommandMsg, this, std::placeholders::_1)
	);

	std::string baudrate_str;
	this->get_parameter("baud_rate", baudrate_str);
	this->baudrate = std::stoi(baudrate_str);
	switch (this->baudrate)
    {
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
        break;

    default:
        RCLCPP_WARN(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
        	"Non-supported baudrate (%d) set! Changed to default value 9600.", this->baudrate);
        this->baudrate = 9600;
        break;
    }

	this->get_parameter("serial_port", this->com_port_name);
	this->com_port_number = RS232_GetPortnr(this->com_port_name.c_str());
	if (this->com_port_number == -1)
	{
		RCLCPP_ERROR(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
			"Cannot find comport number based on device name '%s'", this->com_port_name);
	}
	
	char mode[] = {'8', 'N', '1'};
	int err = RS232_OpenComport(this->com_port_number, this->baudrate, mode, 0);
	if (err != 0)
	{
		RCLCPP_ERROR(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
        	"Could not open COM port '%i'!", this->com_port_number);
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
			"Using COM port '%s' (COM number: %i) with baudrate %i.",
			this->com_port_name.c_str(), this->com_port_number, this->baudrate);
	}

	this->is_running = true;
}

AMExtruderCom::~AMExtruderCom()
{
	delete this->reader_thread;

	RS232_CloseComport(this->com_port_number);
}

#define MSG_EXTRUSION_SPEED_DATA 'E'
#define MSG_HEATING_DATA         'T'
#define LBUF_SIZE 5

void AMExtruderCom::processByte()
{
    static int numBytesTot  = 1;
    static int numBytesRcvd = 0;
    static unsigned char local_buffer[LBUF_SIZE] = {0};

    if (numBytesRcvd < 1)
    {
        int cnt = RS232_PollComport(this->com_port_number, &(local_buffer[0]), 1);
        if (cnt > 0)
        {
        	switch (local_buffer[0])
        	{
        	case MSG_EXTRUSION_SPEED_DATA:
            	numBytesTot  = 5;
            	numBytesRcvd = 1;
            	break;

        	case MSG_HEATING_DATA:
            	numBytesTot  = 5;
            	numBytesRcvd = 1;
            	break;

        	default:
            	break;
        	}
        }
    }

    /* Read numBytesTot-1 extra bytes */
    else if (numBytesRcvd < numBytesTot)
    {
        int num_read = RS232_PollComport(this->com_port_number, &(local_buffer[numBytesRcvd]), numBytesTot-numBytesRcvd);
        numBytesRcvd += num_read;
    }

    /* Execute command when all bytes have been read */
    if (numBytesRcvd >= numBytesTot)
    {
    	unsigned char msg_type = local_buffer[0];
    	switch (msg_type)
    	{
    		case MSG_HEATING_DATA:
    		{
    			auto temperature_state_msg = am_extruder_msg::msg::ExtruderTemperatureState();

    			float* temp_data_ptr = (float*)&local_buffer[1];
    			temperature_state_msg.temperature = *temp_data_ptr;

    			this->extruder_heater_state_publisher->publish(temperature_state_msg);
   				break;
   			}
   			case MSG_EXTRUSION_SPEED_DATA:
   			{
   				auto stepping_state_msg = am_extruder_msg::msg::ExtruderSteppingState();

   				float* step_data_ptr = (float*)&local_buffer[1];
   				stepping_state_msg.stepping_frequency = *step_data_ptr;

   				this->extruder_mover_state_publisher->publish(stepping_state_msg);
   				break;
   			}
   			default:
   				break;
    	}

        numBytesTot  = 1;
        numBytesRcvd = 0;
    }

    return;
}

void AMExtruderCom::readSerial()
{
	RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "Starting readSerial thread.");
	
	while (true)
	{
		if (this->is_running)
		{
			this->processByte();
		}
		else
		{
			std::this_thread::yield();
		}
	}
	return;
}

#define MSG_CMD_SET_HEATING         'H'
#define MSG_CMD_SET_EXTRUSION_SPEED 'X'
#define RS232_SEND_ERROR -1

void AMExtruderCom::onCommandMsg(const am_extruder_msg::msg::ExtruderCommand::SharedPtr msg)
{
	float temperature_target        = msg->temperature_target;
	float stepping_frequency_target = msg->stepping_frequency_target;

	unsigned char buf_temp[1+sizeof(float)] = {0};
	buf_temp[0] = MSG_CMD_SET_HEATING;
	memcpy(&(buf_temp[1]), (unsigned char*)&temperature_target, sizeof(float));

	if (RS232_SendBuf(this->com_port_number, buf_temp, 1+sizeof(float)) == RS232_SEND_ERROR)
	{
		RCLCPP_ERROR(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
           	"Error transmitting '%c%f' on COM port %i!", MSG_CMD_SET_HEATING, temperature_target, this->com_port_number);
	}

	unsigned char buf_step[1+sizeof(float)] = {0};
	buf_step[0] = MSG_CMD_SET_EXTRUSION_SPEED;
	memcpy(&(buf_step[1]), (unsigned char*)&stepping_frequency_target, sizeof(float));

	if (RS232_SendBuf(this->com_port_number, buf_step, 1+sizeof(float)) == RS232_SEND_ERROR)
	{
		RCLCPP_ERROR(rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
        	"Error transmitting '%c%f' on COM port %i!", MSG_CMD_SET_EXTRUSION_SPEED, stepping_frequency_target, this->com_port_number);
	}

	return;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AMExtruderCom>("ttyS10", 10000));
	rclcpp::shutdown();
	return 0;
}
