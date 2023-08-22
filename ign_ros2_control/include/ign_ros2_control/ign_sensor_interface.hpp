#ifndef IGN_ROS2_CONTROL__IGN_SENSOR_INTERFACE_HPP_
#define IGN_ROS2_CONTROL__IGN_SENSOR_INTERFACE_HPP_

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
    /// \brief Initialize the sensor interface
    /// param[in] model_nh Pointer to the ros2 node
    /// param[in] hardware_info structure with data from URDF.
    /// param[in] _ecm Entity-component manager.
    /// param[in] update_rate controller update rate
        virtual bool InitSensorInterface(
            rclcpp::Node::SharedPtr & model_nh,
            const hardware_interface::HardwareInfo & hardware_info,
            ignition::gazebo::EntityComponentManager & _ecm,
            int & update_rate) = 0;
    protected:
        rclcpp::Node::SharedPtr nh_;
    };
} // namespace ign_ros2_control

#endif // IGN_ROS2_CONTROL__IGN_SENSOR_INTERFACE_HPP_