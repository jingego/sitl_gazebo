/*
 * Copyright 2015  Aurelien Roy
 * 
 * This file is a modified version of github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin.git
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
 */

 /**
  * @brief Parachute Plugin
  *
  * This plugin simulates parachute deployment
  *
  * @author Aurelien Roy  <aurroy@hotmail.com>
  */

#include "gazebo_parachute_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(ParachutePlugin)

ParachutePlugin::ParachutePlugin() : ModelPlugin()
{
}

ParachutePlugin::~ParachutePlugin()
{
  _updateConnection->~Connection();
}

void ParachutePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_parachute_plugin] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", trigger_sub_topic_, trigger_sub_topic_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ParachutePlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &ParachutePlugin::VelocityCallback, this);
}

void ParachutePlugin::OnUpdate(const common::UpdateInfo&){
  #if GAZEBO_MAJOR_VERSION >= 9
    physics::ModelPtr parachute_model = world_->ModelByName("parachute_small");
  #else
    physics::ModelPtr parachute_model = world_->GetModel("parachute_small");
  #endif
    //Trigger parachute if flight termination
    if(ref_motor_rot_vel_ <= terminate_rot_vel_ ) LoadParachute();

    if(!attached_parachute_ && parachute_model){
      AttachParachute(parachute_model); //Attach parachute to model
      attached_parachute_ = true;
    }
}

void ParachutePlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void ParachutePlugin::LoadParachute(){
  // Don't create duplicate paracutes
  #if GAZEBO_MAJOR_VERSION >= 9
    if(physics::ModelPtr parachute_model = world_->ModelByName("parachute_small")) return;
  #else
    if(physics::ModelPtr parachute_model = world_->GetModel("parachute_small")) return;
  #endif
  // Insert parachute model
  world_->InsertModelFile("model://parachute_small");

  msgs::Int request;
  request.set_data(0);
  
}

void ParachutePlugin::AttachParachute(physics::ModelPtr &parachute_model){

  const ignition::math::Pose3d uavPose = model_->WorldPose();
  parachute_model->SetWorldPose(ignition::math::Pose3d(uavPose.Pos().X(), uavPose.Pos().Y(), uavPose.Pos().Z()+0.3, 0, 0, 0));        // or use uavPose.ros.GetYaw() ?

  gazebo::physics::JointPtr parachute_joint = world_->Physics()->CreateJoint("fixed", model_);
  parachute_joint->SetName("parachute_joint");
  
  // Attach parachute to base_link
  gazebo::physics::LinkPtr base_link = model_->GetLink("base_link");
  gazebo::physics::LinkPtr parachute_link = parachute_model->GetLink("chute");
  parachute_joint->Attach(base_link, parachute_link);

  // load the joint, and set up its anchor point
  parachute_joint->Load(base_link, parachute_link, ignition::math::Pose3d(0, 0, 0.3, 0, 0, 0));
}
} // namespace gazebo