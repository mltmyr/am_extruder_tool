#include "am_extruder_tool/AMExtruderHardware.hpp"
#include "RS-232/rs232.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <thread>
#include <atomic>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

//#define SIMULATE_EXTRUDER

namespace am_extruder_tool
{

#ifdef SIMULATE_EXTRUDER
#include <chrono>
#

class FirstOrderProcessSimulation
{
public:
    FirstOrderProcessSimulation(double T, double K, double x0, double sampling_hz)
        : x(x0), u(0)
    {
        t_duration = std::chrono::microseconds((long int)(1000000.0/sampling_hz));
        dt = t_duration.count()/1000000.0;

        this->T = T;
        this->K = K;
        //this->x = x0;
        this->x_prev = x0;

        a = (2*T - dt)/(2*T + dt);
        b =     (K*dt)/(2*T + dt);

        //u = 0;
        u_prev = 0;
        
        t_end = std::chrono::steady_clock::now();
    }

    void process()
    {
        t_now = std::chrono::steady_clock::now();
        if (t_now >= t_end)
        {
            x_prev = x;
            x = a*x_prev + b*(u + u_prev);
            u_prev = u;

            t_end = t_now + t_duration;
        }
        return;
    }

    double readState()
    {
        return this->x;
    }
    void actuate(double u)
    {
        this->u = u;
        return;
    }

private:
    std::atomic<double> x, u;
    double x_prev, u_prev;

    double dt;
    double T, K;
    double a, b;

    std::chrono::duration<long int, std::ratio<1,1000000>> t_duration;
    std::chrono::time_point<std::chrono::steady_clock> t_now;
    std::chrono::time_point<std::chrono::steady_clock> t_end;

};

FirstOrderProcessSimulation temperature_sim(220.0, 13.0, 25.0, 5.0);
FirstOrderProcessSimulation filament_speed_sim(0.1, 1.0, 0.0, 10.0);
bool read_temp = true;

#endif


#define MSG_EXTRUSION_SPEED_DATA 'E'
#define MSG_HEATING_DATA         'T'

void AMExtruderHardware::parseData(unsigned char* local_buffer, int len)
{
    if (len <= 0)
    {
        return;
    }

    unsigned char cmd_byte = local_buffer[0];

    if ( len == 5 && cmd_byte == MSG_EXTRUSION_SPEED_DATA )
    {
        float* data = (float*)(&(local_buffer[1]));
        this->mover_val_ = *data;
    }
    else if ( len == 5 && cmd_byte == MSG_HEATING_DATA )
    {
        float* data = (float*)(&(local_buffer[1]));
        this->heater_val_ = *data;
    }

    return;
}

#define LBUF_SIZE 5

void AMExtruderHardware::processByte()
{
    static int numBytesTot  = 1;
    static int numBytesRcvd = 0;
    static unsigned char local_buffer[LBUF_SIZE] = {0};

    if (numBytesRcvd < 1)
    {
#ifndef SIMULATE_EXTRUDER
        RS232_PollComport(this->hw_com_port_number_, &(local_buffer[0]), 1);
#else
        if (read_temp)
        {
            local_buffer[0] = MSG_HEATING_DATA;
        }
        else
        {
            local_buffer[0] = MSG_EXTRUSION_SPEED_DATA;
        }
#endif
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
#ifndef SIMULATE_EXTRUDER
        int num_read = RS232_PollComport(this->hw_com_port_number_, &(local_buffer[numBytesRcvd]), numBytesTot-numBytesRcvd);
#else
        float temp;
        if (read_temp)
        {
            temp = float(temperature_sim.readState());
            read_temp = false;
        }
        else
        {
            temp = float(filament_speed_sim.readState());
            read_temp = true;
        }
        memcpy(&local_buffer[numBytesRcvd], &temp, numBytesTot-numBytesRcvd);
        int num_read = numBytesTot-numBytesRcvd;
#endif
        numBytesRcvd += num_read;
    }

    /* Execute command when all bytes have been read */
    if (numBytesRcvd >= numBytesTot)
    {
        this->parseData(local_buffer, numBytesRcvd);
        numBytesTot  = 1;
        numBytesRcvd = 0;
    }

    return;
}

void AMExtruderHardware::readSerial()
{
    while (true)
    {
#ifdef SIMULATE_EXTRUDER
        temperature_sim.process();
        filament_speed_sim.process();
#endif

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

double calc_stepper_motor_steps_per_mm_filament(int mot_steps_per_rev, int micro_stepping, double gear_ratio, double hobb_diam_mm)
{
    return (double)(mot_steps_per_rev*micro_stepping)*gear_ratio/(hobb_diam_mm*M_PI);
}

AMExtruderHardware::AMExtruderHardware()
    : reader_thread_ptr(nullptr), is_running(false)
{

}

AMExtruderHardware::~AMExtruderHardware()
{
    delete reader_thread_ptr;
}

hardware_interface::return_type AMExtruderHardware::configure(const hardware_interface::HardwareInfo & info)
{
    if (this->configure_default(info) != hardware_interface::return_type::OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    this->hw_com_port_name_ = this->info_.hardware_parameters["com_port_name"];
#ifndef SIMULATE_EXTRUDER
    this->hw_com_port_number_ = RS232_GetPortnr(this->hw_com_port_name_.c_str());

    if (this->hw_com_port_number_ == -1)
    {
        RCLCPP_WARN(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Could not find COM port number for COM port with name %s.",
            this->hw_com_port_name_.c_str());
    }
    else
    {
        RCLCPP_INFO(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Using COM port '%s' (COM number: %d).",
            this->hw_com_port_name_.c_str(), this->hw_com_port_number_);
    }
#endif

    this->hw_baud_rate_ = stoi(this->info_.hardware_parameters["com_port_baud_rate"]);
    switch (this->hw_baud_rate_)
    {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
            RCLCPP_INFO(
                rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                "Using baudrate %d on serial connection.",
                this->hw_baud_rate_);
            break;

        default:
            RCLCPP_WARN(
                rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                "Non-supported baudrate (%d) set! Changed to default value 9600.",
                this->hw_baud_rate_);
            this->hw_baud_rate_ = 9600;
            break;
    }

    this->hw_stepper_motor_steps_per_revolution_ = stoi(this->info_.hardware_parameters["stepper_motor_steps_per_revolution"]);
    this->hw_micro_stepping_        = stoi(this->info_.hardware_parameters["micro_stepping"]);
    this->hw_gear_ratio_            = stod(this->info_.hardware_parameters["gear_ratio"]);
    this->hw_hobb_gear_diameter_mm_ = stod(this->info_.hardware_parameters["hobb_gear_diameter_mm"]);
    this->hw_filament_diameter_mm_  = stod(this->info_.hardware_parameters["filament_diameter_mm"]);
    
    this->hw_filament_mover_state_   = std::numeric_limits<double>::quiet_NaN();
    this->hw_filament_mover_command_ = 0.0; //std::numeric_limits<double>::quiet_NaN();

    this->hw_filament_heater_state_   = std::numeric_limits<double>::quiet_NaN();
    this->hw_filament_heater_command_ = 0.0; //std::numeric_limits<double>::quiet_NaN();

    this->hw_steps_per_mm_filament_ = calc_stepper_motor_steps_per_mm_filament(
        this->hw_stepper_motor_steps_per_revolution_, this->hw_micro_stepping_,
        this->hw_gear_ratio_, this->hw_hobb_gear_diameter_mm_);

    bool found_filament_mover_joint  = false;
    bool found_filament_heater_joint = false;
    for (const hardware_interface::ComponentInfo & joint : this->info_.joints)
    {
        if (joint.name == "filament_mover")
        {
            found_filament_mover_joint = true;

            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Joint '%s' has %d command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Have found %s command interface for joint '%s'. '%s' expected.",
                    joint.command_interfaces[0].name.c_str(), joint.name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Joint '%s' has %d state interfaces found. 1 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Have found %s state interface for joint '%s'. '%s' expected.",
                    joint.state_interfaces[0].name.c_str(), joint.name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }
        }
        else if (joint.name == "filament_heater")
        {
            found_filament_heater_joint = true;

            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Joint '%s' has %d command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Have found %s command interface for joint '%s'. '%s' expected.",
                    joint.command_interfaces[0].name.c_str(), joint.name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Joint '%s' has %d state interfaces found. 1 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                    "Have found %s state interface for joint '%s'. '%s' expected.",
                    joint.state_interfaces[0].name.c_str(), joint.name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }
        }
        else
        {
            RCLCPP_INFO(
                rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                "Found unexpected joint named '%s'.",
                joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

    if (!found_filament_mover_joint)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Did not find 'filament_mover' joint.");
        return hardware_interface::return_type::ERROR;
    }

    if (!found_filament_heater_joint)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Did not find 'filament_heater' joint.");
        return hardware_interface::return_type::ERROR;
    }

    this->reader_thread_ptr = new std::thread(&AMExtruderHardware::readSerial, this);
    this->reader_thread_ptr->detach();

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AMExtruderHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < this->info_.joints.size(); i++)
    {
        std::string joint_name = this->info_.joints[i].name;

        if (joint_name == "filament_mover")
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &(this->hw_filament_mover_state_)));
        }
        else if (joint_name == "filament_heater")
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &(this->hw_filament_heater_state_)));
        }
        else
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                "Found unexpected joint with name '%s' when exporting state interface.",
                joint_name.c_str());
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AMExtruderHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < this->info_.joints.size(); i++)
    {
        std::string joint_name = this->info_.joints[i].name;
        
        if (joint_name == "filament_mover")
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &(this->hw_filament_mover_command_)));
        }
        else if (joint_name == "filament_heater")
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &(this->hw_filament_heater_command_)));
        }
        else
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
                "Found unexpected joint with name '%s' when exporting command interface.",
                joint_name.c_str());
        }
    }

    return command_interfaces;
}

hardware_interface::return_type AMExtruderHardware::start()
{
    RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "start");
#ifndef SIMULATE_EXTRUDER
    char mode[]={'8','N','1'};
    if (RS232_OpenComport(this->hw_com_port_number_, this->hw_baud_rate_, mode, 0))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Could not open COM port '%s'!",
            this->hw_com_port_name_.c_str());
        return hardware_interface::return_type::ERROR;
    }
#endif

    this->is_running = true;

    this->status_ = hardware_interface::status::STARTED;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AMExtruderHardware::stop()
{
    RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "stop");
    // 1. Set heating and extrusion to safe default values.
    // (2. Disable Extruder)

#ifndef SIMULATE_EXTRUDER
    RS232_flushRXTX(this->hw_com_port_number_);
    RS232_CloseComport(this->hw_com_port_number_);
#endif

    this->is_running = false;

    this->status_ = hardware_interface::status::STOPPED;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AMExtruderHardware::read()
{
    RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "read");
    this->hw_filament_mover_state_  = calcExtrusionSpeed(this->mover_val_);
    this->hw_filament_heater_state_ = this->heater_val_;

    RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "temp: %f", this->hw_filament_heater_state_);
    return hardware_interface::return_type::OK;
}

#define MSG_CMD_SET_HEATING         'H'
#define MSG_CMD_SET_EXTRUSION_SPEED 'X'


float AMExtruderHardware::calcStepperFrequency(double extrusion_speed_mm_per_sec)
{
    return (float)((this->hw_steps_per_mm_filament_)*extrusion_speed_mm_per_sec);
}

double AMExtruderHardware::calcExtrusionSpeed(float stepper_frequency)
{

    return ((double)stepper_frequency)/(this->hw_steps_per_mm_filament_);
}


hardware_interface::return_type AMExtruderHardware::write()
{
    RCLCPP_INFO(rclcpp::get_logger(EXTRUDER_LOGGER_NAME), "write");
    unsigned char buf[1+sizeof(float)];

    float extruder_set_value = this->calcStepperFrequency(this->hw_filament_mover_command_);

    buf[0] = MSG_CMD_SET_EXTRUSION_SPEED;
    memcpy(&(buf[1]), (unsigned char*)&(extruder_set_value), sizeof(float));
#ifndef SIMULATE_EXTRUDER
    if (RS232_SendBuf(this->hw_com_port_number_, buf, sizeof(buf)))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Error transmitting '%s%f' on COM port %s!",
            MSG_CMD_SET_EXTRUSION_SPEED, this->hw_filament_mover_command_,
            this->hw_com_port_name_);
    }
#else
    filament_speed_sim.actuate(extruder_set_value);
#endif

    buf[0] = MSG_CMD_SET_HEATING;
    memcpy(&(buf[1]), (unsigned char*)&(this->hw_filament_heater_command_), sizeof(float));
#ifndef SIMULATE_EXTRUDER
    if (RS232_SendBuf(this->hw_com_port_number_, buf, sizeof(buf)))
    {
         RCLCPP_ERROR(
            rclcpp::get_logger(EXTRUDER_LOGGER_NAME),
            "Error transmitting '%s%f' on COM port %s!",
            MSG_CMD_SET_HEATING, this->hw_filament_heater_command_,
            this->hw_com_port_name_);
    }
#else
    temperature_sim.actuate(this->hw_filament_heater_command_);
#endif

    return hardware_interface::return_type::OK;
}

} // namespace am_extruder_tool

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    am_extruder_tool::AMExtruderHardware,
    hardware_interface::ActuatorInterface
)
