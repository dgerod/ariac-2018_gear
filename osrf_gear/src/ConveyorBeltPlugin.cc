/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include "ConveyorBeltPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Read and set the velocity of the belt.
  if (_sdf->HasElement("velocity"))
    this->beltVelocity = _sdf->Get<double>("velocity");
  else
    this->beltVelocity = 0.5;
  gzdbg << "Using belt velocity of: " << this->beltVelocity << " m/s\n";

  // Read and set the joint that controls the belt.
  std::string jointName = "belt_joint";
  if (_sdf->HasElement("joint"))
    jointName = _sdf->Get<std::string>("joint");
  gzdbg << "Using joint name of: [" << jointName << "]\n";
  this->joint = _model->GetJoint(jointName);
  if (!this->joint)
    gzerr << "Joint [" << jointName << "] not found, belt disabled\n";

  // Read and set the belt's link.
  std::string linkName = "belt";
  if (_sdf->HasElement("link"))
    linkName = _sdf->Get<std::string>("link");
  gzdbg << "Using link name of: [" << linkName << "]\n";
  this->link = _model->GetLink(linkName);
  if (!this->link)
    gzerr << "Link not found" << std::endl;

  // Set the point where the link will be moved to its starting pose.
  this->limit = this->joint->GetUpperLimit(0) - 0.6;

  // Listen to the update event that is broadcasted every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ConveyorBeltPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate()
{
  if (!this->joint)
    return;

  this->joint->SetVelocity(0, this->beltVelocity);

  // Reset the belt.
  if (this->joint->GetAngle(0) >= this->limit)
  {
    // Warning: Megahack!!
    // We should use "this->joint->SetPosition(0, 0)" here but I found that
    // this line occasionally freezes the joint. I tracked the problem and
    // found an incorrect value in childLinkPose within
    // Joint::SetPositionMaximal(). This workaround makes sure that the right
    // numbers are always used in our scenario.
    const math::Pose childLinkPose(1.20997, 2.5998, 0.8126, 0, 0, -1.57);
    const math::Pose newChildLinkPose(1.20997, 2.98, 0.8126, 0, 0, -1.57);
    this->link->MoveFrame(childLinkPose, newChildLinkPose);
  }
}

/////////////////////////////////////////////////
double ConveyorBeltPlugin::Velocity() const
{
  if (!this->joint)
    return 0.0;

  return this->beltVelocity;
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::SetVelocity(const double _velocity)
{
  if (!this->joint)
    return;

  this->beltVelocity = _velocity;
}
