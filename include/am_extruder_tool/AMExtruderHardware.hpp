#ifndef AM_EXTRUDER_HARDWARE_HPP_
#define AM_EXTRUDER_HARDWARE_HPP_

#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#define EXTRUDER_LOGGER_NAME "AMExtruderHardware"

namespace am_extruder_tool
{

class AMExtruderHardware : public hardware_interface::BaseInterface<hardware_interface::ActuatorInterface>
{
public:
    return_type configure(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type start() override;
    return_type stop() override;

    return_type read() override;
    return_type write() override;

private:
    void parseData(char* local_buffer, int len);
    void processBytes();

    void readSerial();
    std::thread* reader_thread_ptr;
    std::atomic<bool> is_running;


    std::string hw_com_port_name_;
    int         hw_com_port_number_;
    int         hw_baud_rate_;

    int    hw_stepper_motor_steps_per_revolution_;
    int    hw_micro_stepping_;
    double hw_gear_ratio_;
    double hw_hobb_gear_diameter_mm_;
    double hw_filament_diameter_mm_;

    std::atomic<float> mover_val_;
    std::atomic<float> heater_val_;

    float hw_filament_mover_state_;
    float hw_filament_mover_command_;
    
    float hw_filament_heater_state_;
    float hw_filament_heater_command_;
};

} // namespace am_extruder_tool


#endif // AM_EXTRUDER_HARDWARE_HPP_
