#ifndef IGN_ROS2_CONTROL__IGN_FTS_INTERFACE_HPP_
#define IGN_ROS2_CONTROL__IGN_FTS_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/msgs/wrench.pb.h>

#include "ign_ros2_control/ign_sensor_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/ForceTorque.hh>

#include <hardware_interface/hardware_info.hpp>



namespace ign_ros2_control
{
    class FtsData
    {
    public:
        std::string name{};
        std::string topicName{};
        ignition::gazebo::Entity sim_fts_sensors_ = ignition::gazebo::kNullEntity;
        std::array<double, 6> fts_sensor_data_;
        void OnFts(const ignition::msgs::Wrench & _msg);
    };
    void FtsData::OnFts(const ignition::msgs::Wrench & _msg)
    {
        this->fts_sensor_data_[0] = _msg.force().x();
        this->fts_sensor_data_[1] = _msg.force().y();
        this->fts_sensor_data_[2] = _msg.force().z();
        this->fts_sensor_data_[3] = _msg.torque().x();
        this->fts_sensor_data_[4] = _msg.torque().y();
        this->fts_sensor_data_[5] = _msg.torque().z();
    }
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class IgnitionFtsPrivate;

    class IgnitionFts : public IgnitionSensorInterface
    {
    public: 
        CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info)
        override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
        override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
        override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
        override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

        hardware_interface::return_type read(
            const rclcpp::Time & time,
            const rclcpp::Duration & period
        ) override;

        bool InitSensorInterface(
            rclcpp::Node::SharedPtr & model_nh,
            const hardware_interface::HardwareInfo & hardware_info,
            ignition::gazebo::EntityComponentManager & _ecm,
            int & update_rate) override;

    private:
        std::unique_ptr<IgnitionFtsPrivate> dataPtr;
    };
}  // namespace ign_ros2_control

#endif // IGN_ROS2_CONTROL__IGN_FTS_INTERFACE_HPP_