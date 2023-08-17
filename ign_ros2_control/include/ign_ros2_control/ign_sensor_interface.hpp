// #ifndef IGN_ROS2_CONTROL__IGN_SENSOR_INTERFACE_HPP_
// #define IGN_ROS2_CONTROL__IGN_SENSOR_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/System.hh>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/sensor_interface.hpp>


#include <rclcpp/rclcpp.hpp>

namespace ign_ros2_control
{
    class IgnitionSensorInterface
    : public hardware_interface::SensorInterface
    {
    public:
        virtual bool InitSensorInterface(
            rclcpp::Node::SharedPtr & model_nh,
            const hardware_interface::HardwareInfo & hardware_info,
            ignition::gazebo::EntityComponentManager & _ecm,
            int & update_rate) = 0;
    protected:
        rclcpp::Node::SharedPtr nh_;
    };
}