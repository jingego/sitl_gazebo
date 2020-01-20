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

#ifndef _GAZEBO_PARACHUTE_PLUGIN_HH_
#define _GAZEBO_PARACHUTE_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include "CommandMotorSpeed.pb.h"


#include <Odometry.pb.h>

namespace gazebo
{

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;


class GAZEBO_VISIBLE ParachutePlugin : public ModelPlugin
{
public:
  ParachutePlugin();
  virtual ~ParachutePlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  void LoadParachute();
  void TriggerCallback(const boost::shared_ptr<const msgs::Int> &_msg);
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
  void AttachParachute(physics::ModelPtr &parachute_model);


  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr _updateConnection;

  bool attached_parachute_ = false;
  double max_rot_velocity_ = 3500;
  double ref_motor_rot_vel_ = 0;
  double terminate_rot_vel_ = -1700;
  int motor_number_;

  std::string trigger_sub_topic_ = "/gazebo/command/motor_speed";

  transport::NodePtr node_handle_;
  transport::SubscriberPtr trigger_sub_;

};     // class GAZEBO_VISIBLE ParachutePlugin
}      // namespace gazebo
#endif // _GAZEBO_PARACHUTE_PLUGIN_HH_
