#include "ign_ros2_control/ign_fts_sensor.hpp"
#include <ignition/transport/Node.hh>

class ign_ros2_control::IgnitionFtsPrivate
{
public:
    IgnitionFtsPrivate() = default;
    ~IgnitionFtsPrivate() = default;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    rclcpp::Time last_update_sim_time_ros_;

    std::vector<std::shared_ptr<FtsData>> fts_;

    ignition::gazebo::EntityComponentManager * ecm;

    int * update_rate;

    ignition::transport::Node node;
};

namespace ign_ros2_control
{
bool IgnitionFts::InitSensorInterface(
    rclcpp::Node::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    ignition::gazebo::EntityComponentManager & _ecm,
    int & update_rate)
    {
        this->dataPtr = std::make_unique<IgnitionFtsPrivate>();
        this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();
        this->dataPtr->ecm = &_ecm;
        this->dataPtr->update_rate = &update_rate;
        this->nh_ = model_nh;
        size_t n_sensors = hardware_info.sensors.size();
        std::vector<hardware_interface::ComponentInfo> sensor_components_;

        for (unsigned int j = 0; j < n_sensors; j++)
        {
            hardware_interface::ComponentInfo component = hardware_info.sensors[j];
            sensor_components_.push_back(component);
        }

        RCLCPP_WARN(this->nh_->get_logger(), "Found number of sensors: %ld", sensor_components_.size());

        this->dataPtr->ecm->Each<ignition::gazebo::components::ForceTorque,
            ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity & _entity,
            const ignition::gazebo::components::ForceTorque *,
            const ignition::gazebo::components::Name * _name) -> bool
            {
                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());
                auto sensorTopicComp = this->dataPtr->ecm->Component<
                ignition::gazebo::components::SensorTopic>(_entity);
                if(sensorTopicComp)
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
                }

                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

                auto ftsData = std::make_shared<FtsData>();

                ftsData->name = _name->Data();
                ftsData->sim_fts_sensors_ = _entity;

                hardware_interface::ComponentInfo component;

                for( auto & comp : sensor_components_)
                {
                    if (comp.name == _name->Data())
                    {
                        component = comp;
                    }
                }

                static const std::map<std::string, size_t> interface_name_map = {
                    {"force.x", 0},
                    {"force.y", 1},
                    {"force.z", 2},
                    {"torque.x", 3},
                    {"torque.y", 4},
                    {"torque.z", 5},
                };

                for (const auto & state_interface : component.state_interfaces)
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t" << state_interface.name);

                    size_t data_index = interface_name_map.at(state_interface.name);
                    this->dataPtr->state_interfaces_.emplace_back(
                        ftsData->name,
                        state_interface.name,
                        &ftsData->fts_sensor_data_[data_index]
                    );
                }
                this->dataPtr->fts_.push_back(ftsData);
                return true;
            });
        return true;
    }

hardware_interface::return_type IgnitionFts::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
    {
        for (unsigned int i = 0; i < this->dataPtr->fts_.size(); ++i)
        {
            if (this->dataPtr->fts_[i]->topicName.empty())
            {
                auto sensorTopicComp = this->dataPtr->ecm->Component<
                ignition::gazebo::components::SensorTopic>(this->dataPtr->fts_[i]->sim_fts_sensors_);
                if (sensorTopicComp)
                {
                    this->dataPtr->fts_[i]->topicName = sensorTopicComp->Data();
                    RCLCPP_INFO_STREAM(
                        this->nh_->get_logger(), "FTS " << this->dataPtr->fts_[i]->name << 
                        " has a topic name: " << sensorTopicComp->Data());
                    this->dataPtr->node.Subscribe(
                        this->dataPtr->fts_[i]->topicName, &FtsData::OnFts,
                        this->dataPtr->fts_[i].get());
                }
            }
        }
        return hardware_interface::return_type::OK;
    }


CallbackReturn
IgnitionFts::on_init(const hardware_interface::HardwareInfo & system_info)
{
  RCLCPP_WARN(this->nh_->get_logger(), "On init...");
  if (hardware_interface::SensorInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IgnitionFts::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IgnitionFts::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

CallbackReturn IgnitionFts::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SensorInterface::on_activate(previous_state);
}

CallbackReturn IgnitionFts::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SensorInterface::on_deactivate(previous_state);
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ign_ros2_control::IgnitionFts, ign_ros2_control::IgnitionSensorInterface)
