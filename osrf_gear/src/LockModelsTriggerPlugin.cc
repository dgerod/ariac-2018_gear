/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <limits>
#include <string>
#include <gazebo/transport/Node.hh>

#include "LockModelsTriggerPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the VacuumGripperPlugin class
  struct LockModelsTriggerPluginPrivate
  {
    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Publisher for lock models trigger.
    public: transport::PublisherPtr lockModelsPub;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LockModelsTriggerPlugin)

/////////////////////////////////////////////////
LockModelsTriggerPlugin::LockModelsTriggerPlugin()
: SideContactPlugin(),
  dataPtr(new LockModelsTriggerPluginPrivate)
{
}

/////////////////////////////////////////////////
LockModelsTriggerPlugin::~LockModelsTriggerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void LockModelsTriggerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "LockModelsTriggerPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "LockModelsTriggerPlugin running at the default update rate\n";

  this->dataPtr->node = transport::NodePtr(new transport::Node());

      std::string topic_name = "/ariac/" + std::string("shipping_box_0") + "/lock_models";
  this->dataPtr->lockModelsPub = this->dataPtr->node->Advertise<msgs::GzString>(topic_name);
}

/////////////////////////////////////////////////
void LockModelsTriggerPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  this->CalculateContactingModels();
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void LockModelsTriggerPlugin::ActOnContactingModels()
{
  for (auto model : this->contactingModels) {
    if (model) {
      // Ignore models not matching the expected format
      size_t index = model->GetName().rfind("shipping_box");
      if (index == std::string::npos)
      {
      std::cout << "[" << this->model->GetName() << "] Ignoring: " << model->GetName() << std::endl;
        continue;
      }
      std::cout << "[" << this->model->GetName() << "] Lock models triggered for: " << model->GetName() << std::endl;
      gzdbg << "[" << this->model->GetName() << "] Lock models triggered for: " << model->GetName() << "\n";
      std::string topic_name = "/ariac/" + model->GetName() + "/lock_models";
      gazebo::msgs::GzString lock_msg;
      lock_msg.set_data("lock");
      this->dataPtr->lockModelsPub->Publish(lock_msg);
    }
  }
}
