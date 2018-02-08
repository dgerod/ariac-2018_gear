/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "ROSConveyorBeltPlugin.hh"
#include "osrf_gear/ConveyorBeltState.h"

#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  this->rosnode_->shutdown();
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    this->robotNamespace_ = _sdf->GetElement(
        "robot_namespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string controlTopic = "conveyor/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "conveyor/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  this->controlService_ = this->rosnode_->advertiseService(controlTopic,
    &ROSConveyorBeltPlugin::OnControlCommand, this);

  // Message used for publishing the state of the conveyor.
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    osrf_gear::ConveyorState>(stateTopic, 1000);
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Publish()
{
  osrf_gear::ConveyorState stateMsg;
  stateMsg.enabled = this->IsEnabled();
  stateMsg.power = this->Power();
  this->dataPtr->statePub.publish(stateMsg);
}

/////////////////////////////////////////////////
bool ROSConveyorBeltPlugin::OnControlCommand(ros::ServiceEvent<
  osrf_gear::ConveyorBeltControl::Request, osrf_gear::ConveyorBeltControl::Response> & event)
{
  const osrf_gear::ConveyorBeltControl::Request& req = event.getRequest();
  osrf_gear::ConveyorBeltControl::Response& res = event.getResponse();
  gzdbg << "Conveyor control service called with: " << req.power << std::endl;

  const std::string& callerName = event.getCallerName();
  gzdbg << "Conveyor control service called by: " << callerName << std::endl;

  if (this->IsEnabled())
  {
    this->SetPower(req.power);
    res.success = true;
  } else {
    std::string errStr = "Belt is not currently enabled so power cannot be set. It may be congested.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
  }
  return true;
}
