#include "tbai_ros_gazebo/ContactSensor.hpp"

#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/config/Config.hpp>

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(ContactSensor)

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (!this->parentSensor) {
        TBAI_GLOBAL_LOG_FATAL("Could not load contact sensor plugin.");
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactSensor::OnUpdate, this));
    this->parentSensor->SetActive(true);

    // create ROS publisher
    const std::string topicName = this->parentSensor->Name();
    contactPublisher_ = ros::NodeHandle().advertise<std_msgs::Bool>(topicName, 1);

    // Set initial state
    lastState_ = false;

    TBAI_GLOBAL_LOG_INFO("Loading ContactSensor plugin. Publishing on topic /{}", topicName);

    // Setup update rate
    auto updateRate = tbai::fromGlobalConfig<tbai::scalar_t>("contact_sensor/update_rate");
    this->parentSensor->SetUpdateRate(updateRate);

    TBAI_GLOBAL_LOG_INFO("Loaded ContactSensor plugin. Update rate: {} Hz", updateRate);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::OnUpdate() {
    bool state = parentSensor->Contacts().contact_size() != 0;
    if (state == lastState_) {
        return;
    }

    // Contact flag changed, publish message
    contactMsg_.data = state;
    contactPublisher_.publish(contactMsg_);
    lastState_ = state;
}

};  // namespace gazebo
