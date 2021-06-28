#include <functional>
//#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "RS-232/rs232.h"

//using namespace std::chrono_literals;

#define EXTRUDER_LOGGER_NAME "AMExtruderCom"

class AMExtruderCom : public rclcpp::Node
{
public:
	AMExtruderCom(int com_port_number, int baudrate);
	~AMExtruderCom();
private:
	void processByte();
	void readSerial();
	void onCommand(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
	rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr extruder_data_publisher;
	//rclcpp::TimerBase::SharedPtr publish_timer;
	std::thread* reader_thread;
	std::atomic<bool> is_running;

	rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr extruder_command_subscriber;
	int com_port_number;
	int baudrate;
};

AMExtruderCom::AMExtruderCom(int com_port_number, int baudrate)
	: Node("am_hw_com"), is_running(false), com_port_number(com_port_number), baudrate(baudrate)
{


	this->extruder_data_publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>("am_extruder_data", 10);
	//this->publish_timer = this->create_wall_timer(100ms, std::bind(&AMExtruderCom::readSerial, this));
	this->reader_thread = new std::thread(&AMExtruderCom::readSerial, this);
	this->reader_thread->detach();

	this->extruder_command_subscriber = this->create_subscription<std_msgs::msg::ByteMultiArray>(
		"am_extruder_command", 10, std::bind(&AMExtruderCom::onCommand, this, std::placeholders::_1)
	);

	char mode[] = {'8', 'N', '1'};
	int err = RS232_OpenComport(this->com_port_number, this->baudrate, mode, 0);
	if (err != 0)
	{
		RCLCPP_ERROR(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Could not open COM port '%i'!",
            this->com_port_number);
	}

	this->is_running = true;
}

AMExtruderCom::~AMExtruderCom()
{
	delete this->reader_thread;
	RS232_flushRXTX(this->com_port_number);
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
        RS232_PollComport(this->com_port_number, &(local_buffer[0]), 1);
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

    /* Read numBytesTot-1 extra bytes */
    else if (numBytesRcvd < numBytesTot)
    {
        int num_read = RS232_PollComport(this->com_port_number, &(local_buffer[numBytesRcvd]), numBytesTot-numBytesRcvd);
        numBytesRcvd += num_read;
    }

    /* Execute command when all bytes have been read */
    if (numBytesRcvd >= numBytesTot)
    {
    	auto message = std_msgs::msg::ByteMultiArray();
    	for (int i = 0; i < LBUF_SIZE; i++)
    	{
    		message.data.emplace_back(local_buffer[i]);
    	}
        this->extruder_data_publisher->publish(message);

        numBytesTot  = 1;
        numBytesRcvd = 0;
    }

    return;
}

void AMExtruderCom::readSerial()
{
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

void AMExtruderCom::onCommand(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
	int len = msg->data.size();
	if (len <= 0)
	{
		return;
	}

	unsigned char buf[1+sizeof(float)];
	for (int i = 0; i < len; i++)
	{
		buf[i] = msg->data[i];
	}

	if (RS232_SendBuf(this->com_port_number, buf, len))
	{
		char msg_type = buf[0];
		if ((msg_type == MSG_CMD_SET_HEATING) || (msg_type == MSG_CMD_SET_EXTRUSION_SPEED))
		{
			float* data = (float*)&(msg->data[1]);
			RCLCPP_ERROR(
            	rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            	"Error transmitting '%s%f' on COM port %s!",
            	msg_type, *data, this->com_port_number);
		}
		else
		{
			RCLCPP_ERROR(
            	rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            	"Error transmitting '%s' on COM port %s!",
            	msg_type, this->com_port_number);
		}
	}
	return;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AMExtruderCom>(24, 9600));
	rclcpp::shutdown();
	return 0;
}
