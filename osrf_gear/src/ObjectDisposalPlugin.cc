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

#include "ObjectDisposalPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)

static const std::string SHIPPING_BOX_MODEL_NAME = "shipping_box";
static const int NUM_SHIPPING_BOXES = 10;

/////////////////////////////////////////////////
ObjectDisposalPlugin::ObjectDisposalPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  if (this->updateRate > 0)
    gzdbg << "ObjectDisposalPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ObjectDisposalPlugin running at the default update rate\n";

  this->centerOfGravityCheck = false;
  if (_sdf->HasElement("center_of_gravity_check"))
  {
    this->centerOfGravityCheck = _sdf->Get<bool>("center_of_gravity_check");
  }

  std::string activation_topic = "/ariac/" +  this->model->GetName() + "/activate";
  if (_sdf->HasElement("activation_topic"))
  {
    activation_topic = _sdf->Get<std::string>("activation_topic");
  }
  this->activationSub = this->node->Subscribe(
    activation_topic,
    &ObjectDisposalPlugin::OnActivation, this);

  if (!_sdf->HasElement("disposal_pose"))
  {
    gzerr << "ObjectDisposalPlugin: Unable to find <disposal_pose> element\n";
    return;
  }
  this->disposalPose = _sdf->Get<math::Pose>("disposal_pose");
  fprintf(stderr, "ObjectDisposalPlugin: finished loading");

  for (int i = 0; i < NUM_SHIPPING_BOXES; ++i) {
    std::string shippingBoxName = SHIPPING_BOX_MODEL_NAME + "_" + std::to_string(i);
    std::string topicName = "/ariac/" + shippingBoxName + "_visual_toggle";
    auto pub = this->node->Advertise<msgs::GzString>(topicName);
    this->closeBoxPubMap.insert(std::pair<std::string, transport::PublisherPtr>(shippingBoxName, pub));
    this->boxClosedMap.insert(std::pair<std::string, bool>(shippingBoxName, false));
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  this->CalculateContactingModels();
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::ActOnContactingModels()
{
  for (auto model : this->contactingModels)
  {
    if (!(model && model->GetName().compare(0, SHIPPING_BOX_MODEL_NAME.length(), SHIPPING_BOX_MODEL_NAME) == 0))
    {
      continue;
    }
    auto it = this->closeBoxPubMap.find(model->GetName());
    if (it != this->closeBoxPubMap.end() && !this->boxClosedMap.find(model->GetName())->second)
    {
      // Toggle the box visual.
      gazebo::msgs::GzString toggleMsg;
      toggleMsg.set_data("");
      it->second->Publish(toggleMsg);
      this->boxClosedMap[model->GetName()] = true;
    }
  }

  if (!this->active)
    return;

  // Only remove models if their center of gravity is "above" the link
  // TODO: make more general than just z axis
  auto linkBox = this->parentLink->GetBoundingBox();
  auto linkBoxMax = linkBox.max;
  auto linkBoxMin = linkBox.min;
  linkBoxMin.z = std::numeric_limits<double>::lowest();
  linkBoxMax.z = std::numeric_limits<double>::max();
  auto disposalBox = math::Box(linkBoxMin, linkBoxMax);

  for (auto model : this->contactingModels) {
    if (model) {
      bool removeModel = true;
      if (this->centerOfGravityCheck)
      {
        // Calculate the center of gravity of the model
        math::Vector3 modelCog = math::Vector3::Zero;
        double modelMass = 0.0;
        for (auto modelLink : model->GetLinks())
        {
          double linkMass = modelLink->GetInertial()->GetMass();
          modelCog += modelLink->GetWorldCoGPose().pos * linkMass;
          modelMass += linkMass;
        }
        if (modelMass > 0.0)
        {
          modelCog /= modelMass;
        }
        removeModel = disposalBox.Contains(modelCog);
      }
      if (removeModel)
      {
        gzdbg << "[" << this->model->GetName() << "] Removing model: " << model->GetName() << "\n";
        model->SetAutoDisable(true);
        model->SetStatic(true);
        model->SetWorldPose(this->disposalPose + math::Pose(0, 0.25 * this->numRemovedModels++, 0, 0, 0, 0));
      }
    }
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  fprintf(stderr, "ObjectDisposalPlugin: received activation request: %s\n", _msg->data().c_str());

  if (_msg->data() == "activate")
    this->active = true;
  else if (_msg->data() == "deactivate")
    this->active = false;
  else
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
}
