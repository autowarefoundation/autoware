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

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/AdvertiseOptions.hh>

#include "PriusHybridPlugin.hh"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include "PriusHybridPlugin.hh"

#include <ros/ros.h>

namespace gazebo
{
  class PriusHybridPluginPrivate
  {
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    public: enum DirectionType {
              /// \brief Reverse
              REVERSE = -1,
              /// \brief Neutral
              NEUTRAL = 0,
              /// \brief Forward
              FORWARD = 1
            };

    public: ros::NodeHandle nh;

    public: ros::Subscriber controlSub;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the parent model
    public: physics::ModelPtr model;

    /// \brief Transport node
    public: transport::NodePtr gznode;

    /// \brief Ignition transport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport position pub
    public: ignition::transport::Node::Publisher posePub;

    /// \brief Ignition transport console pub
    public: ignition::transport::Node::Publisher consolePub;

    /// \brief Physics update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Chassis link
    public: physics::LinkPtr chassisLink;

    /// \brief Front left wheel joint
    public: physics::JointPtr flWheelJoint;

    /// \brief Front right wheel joint
    public: physics::JointPtr frWheelJoint;

    /// \brief Rear left wheel joint
    public: physics::JointPtr blWheelJoint;

    /// \brief Rear right wheel joint
    public: physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    public: physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    public: physics::JointPtr frWheelSteeringJoint;

    /// \brief Steering wheel joint
    public: physics::JointPtr handWheelJoint;

    /// \brief PID control for the front left wheel steering joint
    public: common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    public: common::PID frWheelSteeringPID;

    /// \brief PID control for steering wheel joint
    public: common::PID handWheelPID;

    /// \brief Last pose msg time
    public: common::Time lastMsgTime;

    /// \brief Last sim time received
    public: common::Time lastSimTime;

    /// \brief Last sim time when a pedal command is received
    public: common::Time lastPedalCmdTime;

    /// \brief Last sim time when a steering command is received
    public: common::Time lastSteeringCmdTime;

    /// \brief Last sim time when a EV mode command is received
    public: common::Time lastModeCmdTime;

    /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
    public: DirectionType directionState;

    /// \brief Chassis aerodynamic drag force coefficient,
    /// with units of [N / (m/s)^2]
    public: double chassisAeroForceGain = 0;

    /// \brief Max torque that can be applied to the front wheels
    public: double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    public: double backTorque = 0;

    /// \brief Max speed (m/s) of the car
    public: double maxSpeed = 0;

    /// \brief Max steering angle
    public: double maxSteer = 0;

    /// \brief Max torque that can be applied to the front brakes
    public: double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    public: double backBrakeTorque = 0;

    /// \brief Angle ratio between the steering wheel and the front wheels
    public: double steeringRatio = 0;

    /// \brief Max range of hand steering wheel
    public: double handWheelHigh = 0;

    /// \brief Min range of hand steering wheel
    public: double handWheelLow = 0;

    /// \brief Front left wheel desired steering angle (radians)
    public: double flWheelSteeringCmd = 0;

    /// \brief Front right wheel desired steering angle (radians)
    public: double frWheelSteeringCmd = 0;

    /// \brief Steering wheel desired angle (radians)
    public: double handWheelCmd = 0;

    /// \brief Front left wheel radius
    public: double flWheelRadius = 0;

    /// \brief Front right wheel radius
    public: double frWheelRadius = 0;

    /// \brief Rear left wheel radius
    public: double blWheelRadius = 0;

    /// \brief Rear right wheel radius
    public: double brWheelRadius = 0;

    /// \brief Front left joint friction
    public: double flJointFriction = 0;

    /// \brief Front right joint friction
    public: double frJointFriction = 0;

    /// \brief Rear left joint friction
    public: double blJointFriction = 0;

    /// \brief Rear right joint friction
    public: double brJointFriction = 0;

    /// \brief Distance distance between front and rear axles
    public: double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    public: double frontTrackWidth = 0;

    /// \brief Distance distance between rear left and right wheels
    public: double backTrackWidth = 0;

    /// \brief Gas energy density (J/gallon)
    public: const double kGasEnergyDensity = 1.29e8;

    /// \brief Battery charge capacity in Watt-hours
    public: double batteryChargeWattHours = 280;

    /// \brief Battery discharge capacity in Watt-hours
    public: double batteryDischargeWattHours = 260;

    /// \brief Gas engine efficiency
    public: double gasEfficiency = 0.37;

    /// \brief Minimum gas flow rate (gallons / sec)
    public: double minGasFlow = 1e-4;

    /// \brief Gas consumption (gallon)
    public: double gasConsumption = 0;

    /// \brief Battery state-of-charge (percent, 0.0 - 1.0)
    public: double batteryCharge = 0.75;

    /// \brief Battery charge threshold when it has to be recharged.
    public: const double batteryLowThreshold = 0.125;

    /// \brief Whether EV mode is on or off.
    public: bool evMode = false;

    /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
    public: double gasPedalPercent = 0;

    /// \brief Power for charging a low battery (Watts).
    public: const double kLowBatteryChargePower = 2000;

    /// \brief Threshold delimiting the gas pedal (throttle) low and medium
    /// ranges.
    public: const double kGasPedalLowMedium = 0.25;

    /// \brief Threshold delimiting the gas pedal (throttle) medium and high
    /// ranges.
    public: const double kGasPedalMediumHigh = 0.5;

    /// \brief Threshold delimiting the speed (throttle) low and medium
    /// ranges in miles/h.
    public: const double speedLowMedium = 26.0;

    /// \brief Threshold delimiting the speed (throttle) medium and high
    /// ranges in miles/h.
    public: const double speedMediumHigh = 46.0;

    /// \brief Brake pedal position in percentage. 1.0 =
    public: double brakePedalPercent = 0;

    /// \brief Hand brake position in percentage.
    public: double handbrakePercent = 1.0;

    /// \brief Angle of steering wheel at last update (radians)
    public: double handWheelAngle = 0;

    /// \brief Steering angle of front left wheel at last update (radians)
    public: double flSteeringAngle = 0;

    /// \brief Steering angle of front right wheel at last update (radians)
    public: double frSteeringAngle = 0;

    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    public: ignition::math::Vector3d chassisLinearVelocity;

    /// \brief Angular velocity of front left wheel at last update (rad/s)
    public: double flWheelAngularVelocity = 0;

    /// \brief Angular velocity of front right wheel at last update (rad/s)
    public: double frWheelAngularVelocity = 0;

    /// \brief Angular velocity of back left wheel at last update (rad/s)
    public: double blWheelAngularVelocity = 0;

    /// \brief Angular velocity of back right wheel at last update (rad/s)
    public: double brWheelAngularVelocity = 0;

    /// \brief Subscriber to the keyboard topic
    public: transport::SubscriberPtr keyboardSub;

    /// \brief Mutex to protect updates
    public: std::mutex mutex;

    /// \brief Odometer
    public: double odom = 0.0;

    /// \brief Keyboard control type
    public: int keyControl = 1;

    /// \brief Publisher for the world_control topic.
    public: transport::PublisherPtr worldControlPub;
  };
}

using namespace gazebo;

/////////////////////////////////////////////////
PriusHybridPlugin::PriusHybridPlugin()
    : dataPtr(new PriusHybridPluginPrivate)
{
  int argc = 0;
  char *argv = nullptr;
  ros::init(argc, &argv, "PriusHybridPlugin");
  ros::NodeHandle nh;
  this->dataPtr->controlSub = nh.subscribe("prius", 10, &PriusHybridPlugin::OnPriusCommand, this);

  this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
  this->dataPtr->flWheelRadius = 0.3;
  this->dataPtr->frWheelRadius = 0.3;
  this->dataPtr->blWheelRadius = 0.3;
  this->dataPtr->brWheelRadius = 0.3;
}


void PriusHybridPlugin::OnPriusCommand(const prius_msgs::Control::ConstPtr &msg)
{
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();

  // Steering wheel command
  double handCmd = (msg->steer < 0.)
    ? (msg->steer * -this->dataPtr->handWheelLow)
    : (msg->steer * this->dataPtr->handWheelHigh);

  handCmd = ignition::math::clamp(handCmd, this->dataPtr->handWheelLow,
      this->dataPtr->handWheelHigh);
  this->dataPtr->handWheelCmd = handCmd;

  // Brake command
  double brake = ignition::math::clamp(msg->brake, 0.0, 1.0);
  this->dataPtr->brakePedalPercent = brake;

  // Throttle command
  double throttle = ignition::math::clamp(msg->throttle, 0.0, 1.0);
  this->dataPtr->gasPedalPercent = throttle;

  switch (msg->shift_gears)
  {
    case prius_msgs::Control::NEUTRAL:
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    case prius_msgs::Control::FORWARD:
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    case prius_msgs::Control::REVERSE:
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
PriusHybridPlugin::~PriusHybridPlugin()
{
  this->dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "PriusHybridPlugin loading params" << std::endl;
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();
  auto physicsEngine = this->dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  this->dataPtr->node.Subscribe("/prius/reset",
      &PriusHybridPlugin::OnReset, this);
  this->dataPtr->node.Subscribe("/prius/stop",
      &PriusHybridPlugin::OnStop, this);

  this->dataPtr->node.Subscribe("/cmd_vel", &PriusHybridPlugin::OnCmdVel, this);
  this->dataPtr->node.Subscribe("/cmd_gear",
      &PriusHybridPlugin::OnCmdGear, this);
  this->dataPtr->node.Subscribe("/cmd_mode",
      &PriusHybridPlugin::OnCmdMode, this);

  this->dataPtr->posePub = this->dataPtr->node.Advertise<ignition::msgs::Pose>(
      "/prius/pose");
  this->dataPtr->consolePub =
    this->dataPtr->node.Advertise<ignition::msgs::Double_V>("/prius/console");

  std::string chassisLinkName = dPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("chassis");
  dPtr->chassisLink = dPtr->model->GetLink(chassisLinkName);
  if (!dPtr->chassisLink)
  {
    std::cerr << "could not find chassis link" << std::endl;
    return;
  }

  std::string handWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("steering_wheel");
  this->dataPtr->handWheelJoint =
    this->dataPtr->model->GetJoint(handWheelJointName);
  if (!this->dataPtr->handWheelJoint)
  {
    std::cerr << "could not find steering wheel joint" <<std::endl;
    return;
  }

  std::string flWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel");
  this->dataPtr->flWheelJoint =
    this->dataPtr->model->GetJoint(flWheelJointName);
  if (!this->dataPtr->flWheelJoint)
  {
    std::cerr << "could not find front left wheel joint" <<std::endl;
    return;
  }

  std::string frWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel");
  this->dataPtr->frWheelJoint =
    this->dataPtr->model->GetJoint(frWheelJointName);
  if (!this->dataPtr->frWheelJoint)
  {
    std::cerr << "could not find front right wheel joint" <<std::endl;
    return;
  }

  std::string blWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_left_wheel");
  this->dataPtr->blWheelJoint =
    this->dataPtr->model->GetJoint(blWheelJointName);
  if (!this->dataPtr->blWheelJoint)
  {
    std::cerr << "could not find back left wheel joint" <<std::endl;
    return;
  }

  std::string brWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_right_wheel");
  this->dataPtr->brWheelJoint =
    this->dataPtr->model->GetJoint(brWheelJointName);
  if (!this->dataPtr->brWheelJoint)
  {
    std::cerr << "could not find back right wheel joint" <<std::endl;
    return;
  }

  std::string flWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel_steering");
  this->dataPtr->flWheelSteeringJoint =
    this->dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->dataPtr->flWheelSteeringJoint)
  {
    std::cerr << "could not find front left steering joint" <<std::endl;
    return;
  }

  std::string frWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel_steering");
  this->dataPtr->frWheelSteeringJoint =
    this->dataPtr->model->GetJoint(frWheelSteeringJointName);
  if (!this->dataPtr->frWheelSteeringJoint)
  {
    std::cerr << "could not find front right steering joint" <<std::endl;
    return;
  }

  std::string paramName;
  double paramDefault;

  paramName = "chassis_aero_force_gain";
  paramDefault = 1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->chassisAeroForceGain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->chassisAeroForceGain = paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backBrakeTorque = paramDefault;

  paramName = "battery_charge_watt_hours";
  paramDefault = 280;
  if (_sdf->HasElement(paramName))
    this->dataPtr->batteryChargeWattHours = _sdf->Get<double>(paramName);
  else
    this->dataPtr->batteryChargeWattHours = paramDefault;

  paramName = "battery_discharge_watt_hours";
  paramDefault = 260;
  if (_sdf->HasElement(paramName))
    this->dataPtr->batteryDischargeWattHours = _sdf->Get<double>(paramName);
  else
    this->dataPtr->batteryDischargeWattHours = paramDefault;

  paramName = "gas_efficiency";
  paramDefault = 0.37;
  if (_sdf->HasElement(paramName))
    this->dataPtr->gasEfficiency = _sdf->Get<double>(paramName);
  else
    this->dataPtr->gasEfficiency = paramDefault;

  paramName = "min_gas_flow";
  paramDefault = 1e-4;
  if (_sdf->HasElement(paramName))
    this->dataPtr->minGasFlow = _sdf->Get<double>(paramName);
  else
    this->dataPtr->minGasFlow = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSteer = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSteer = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetPGain(paramDefault);

  paramName = "frwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetPGain(paramDefault);

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetIGain(paramDefault);

  paramName = "frwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetIGain(paramDefault);

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetDGain(paramDefault);

  paramName = "frwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetDGain(paramDefault);

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->dataPtr->flWheelRadius = this->CollisionRadius(
      this->dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->frWheelRadius = this->CollisionRadius(
      this->dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->blWheelRadius = this->CollisionRadius(
      this->dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->brWheelRadius = this->CollisionRadius(
      this->dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  // Get initial joint friction and add it to braking friction
  dPtr->flJointFriction = dPtr->flWheelJoint->GetParam("friction", 0);
  dPtr->frJointFriction = dPtr->frWheelJoint->GetParam("friction", 0);
  dPtr->blJointFriction = dPtr->blWheelJoint->GetParam("friction", 0);
  dPtr->brJointFriction = dPtr->brWheelJoint->GetParam("friction", 0);

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos =
    this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos =
    this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos =
    this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos =
    this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;
  this->dataPtr->backTrackWidth = vec3.Length();
  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  // gzerr << "wheel base length and track width: "
  //   << this->dataPtr->wheelbaseLength << " "
  //   << this->dataPtr->frontTrackWidth
  //   << " " << this->dataPtr->backTrackWidth << std::endl;

  // Max force that can be applied to hand steering wheel
  double handWheelForce = 10;
  this->dataPtr->handWheelPID.Init(100, 0, 10, 0, 0,
      handWheelForce, -handWheelForce);

  // Max force that can be applied to wheel steering joints
  double kMaxSteeringForceMagnitude = 5000;

  this->dataPtr->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusHybridPlugin::Update, this));

  this->dataPtr->keyboardSub =
    this->dataPtr->gznode->Subscribe("~/keyboard/keypress",
        &PriusHybridPlugin::OnKeyPress, this, true);

  this->dataPtr->worldControlPub =
    this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");

  this->dataPtr->node.Subscribe("/keypress", &PriusHybridPlugin::OnKeyPressIgn,
      this);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdVel(const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->gasPedalPercent = std::min(_msg.position().x(), 1.0);
  this->dataPtr->handWheelCmd = _msg.position().y();
  this->dataPtr->brakePedalPercent = _msg.position().z();

  this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
}
/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdGear(const ignition::msgs::Int32 &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // -1 reverse, 0 neutral, 1 forward
  int state = static_cast<int>(this->dataPtr->directionState);
  state += _msg.data();
  state = ignition::math::clamp(state, -1, 1);
  this->dataPtr->directionState =
      static_cast<PriusHybridPluginPrivate::DirectionType>(state);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdMode(const ignition::msgs::Boolean &/*_msg*/)
{
  // toggle ev mode
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->evMode = !this->dataPtr->evMode;
}

/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControlTypeA(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // e - gas pedal
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // w - release pedals
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // q - brake
    case 113:
    {
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->brakePedalPercent += 0.1;
      this->dataPtr->brakePedalPercent =
          std::min(this->dataPtr->brakePedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - center steering
    case 83:
    case 115:
    {
      this->dataPtr->handWheelCmd = 0;
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // z reverse
    case 90:
    case 122:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // c forward
    case 67:
    case 99:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    }

    default:
    {
      this->dataPtr->brakePedalPercent = 0;
      this->dataPtr->gasPedalPercent = 0;
      break;
    }
  }
}


/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControlTypeB(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // w - accelerate forward
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - reverse
    case 83:
    case 115:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      if (this->dataPtr->directionState != PriusHybridPluginPrivate::REVERSE)
        this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // e brake
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 1.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // q - EV mode
    case 81:
    case 113:
    {
      // avoid rapid mode changes due to repeated key press
      common::Time now = this->dataPtr->world->SimTime();
      if ((now - this->dataPtr->lastModeCmdTime).Double() > 0.3)
      {
        this->dataPtr->evMode = !this->dataPtr->evMode;
        this->dataPtr->lastModeCmdTime = now;
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControl(const int _key)
{
  if (this->dataPtr->keyControl == 0)
    this->KeyControlTypeA(_key);
  else if (this->dataPtr->keyControl == 1)
    this->KeyControlTypeB(_key);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  this->KeyControl(_msg->int_value());
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg)
{
  this->KeyControl(_msg.int_value());
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnReset(const ignition::msgs::Any & /*_msg*/)
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnStop(const ignition::msgs::Any & /*_msg*/)
{
  ignition::msgs::StringMsg req;
  ignition::msgs::StringMsg rep;
  bool result = false;
  unsigned int timeout = 5000;
  bool executed = this->dataPtr->node.Request("/priuscup/upload",
      req, timeout, rep, result);
  if (executed)
  {
    std::cerr << "Result: " << result << std::endl;
    std::cerr << rep.data() << std::endl;
  }
  else
  {
    std::cerr << "Service call timed out" << std::endl;
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Reset()
{
  this->dataPtr->odom = 0;
  this->dataPtr->flWheelSteeringPID.Reset();
  this->dataPtr->frWheelSteeringPID.Reset();
  this->dataPtr->handWheelPID.Reset();
  this->dataPtr->lastMsgTime = 0;
  this->dataPtr->lastSimTime = 0;
  this->dataPtr->lastModeCmdTime = 0;
  this->dataPtr->lastPedalCmdTime = 0;
  this->dataPtr->lastSteeringCmdTime = 0;
  this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
  this->dataPtr->flWheelSteeringCmd = 0;
  this->dataPtr->frWheelSteeringCmd = 0;
  this->dataPtr->handWheelCmd = 0;
  this->dataPtr->batteryCharge = 0.75;
  this->dataPtr->gasConsumption = 0;
  this->dataPtr->gasPedalPercent = 0;
  this->dataPtr->brakePedalPercent = 0;
  this->dataPtr->handbrakePercent = 1.0;
  this->dataPtr->handWheelAngle  = 0;
  this->dataPtr->flSteeringAngle = 0;
  this->dataPtr->frSteeringAngle = 0;
  this->dataPtr->flWheelAngularVelocity  = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity  = 0;
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Update()
{
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0)
  {
    this->Reset();
    return;
  }
  else if (ignition::math::equal(dt, 0.0))
  {
    return;
  }

  dPtr->handWheelAngle = dPtr->handWheelJoint->Position();
  dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
  dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();

  dPtr->flWheelAngularVelocity = dPtr->flWheelJoint->GetVelocity(0);
  dPtr->frWheelAngularVelocity = dPtr->frWheelJoint->GetVelocity(0);
  dPtr->blWheelAngularVelocity = dPtr->blWheelJoint->GetVelocity(0);
  dPtr->brWheelAngularVelocity = dPtr->brWheelJoint->GetVelocity(0);

  dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();
  // Convert meter/sec to miles/hour
  double linearVel = dPtr->chassisLinearVelocity.Length() * 2.23694;

  // Distance traveled in miles.
  this->dataPtr->odom += (fabs(linearVel) * dt/3600.0);

  bool neutral = dPtr->directionState == PriusHybridPluginPrivate::NEUTRAL;

  this->dataPtr->lastSimTime = curTime;

  // Aero-dynamic drag on chassis
  // F: force in world frame, applied at center of mass
  // V: velocity in world frame of chassis center of mass
  // C: drag coefficient based on straight-ahead driving [N / (m/s)^2]
  // |V|: speed
  // V_hat: velocity unit vector
  // F = -C |V|^2 V_hat
  auto dragForce = -dPtr->chassisAeroForceGain *
        dPtr->chassisLinearVelocity.SquaredLength() *
        dPtr->chassisLinearVelocity.Normalized();
  dPtr->chassisLink->AddForce(dragForce);

  // PID (position) steering
  this->dataPtr->handWheelCmd =
    ignition::math::clamp(this->dataPtr->handWheelCmd,
        -this->dataPtr->maxSteer / this->dataPtr->steeringRatio,
        this->dataPtr->maxSteer / this->dataPtr->steeringRatio);
  double steerError =
      this->dataPtr->handWheelAngle - this->dataPtr->handWheelCmd;
  double steerCmd = this->dataPtr->handWheelPID.Update(steerError, dt);
  this->dataPtr->handWheelJoint->SetForce(0, steerCmd);
  //this->dataPtr->handWheelJoint->SetPosition(0, this->dataPtr->handWheelCmd);
  //this->dataPtr->handWheelJoint->SetLowStop(0, this->dataPtr->handWheelCmd);
  //this->dataPtr->handWheelJoint->SetHighStop(0, this->dataPtr->handWheelCmd);

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here
  //  \TODO provide documentation for these equations
  double tanSteer =
      tan(this->dataPtr->handWheelCmd * this->dataPtr->steeringRatio);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  // this->flWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;
  // this->frWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;

  double flwsError =
      this->dataPtr->flSteeringAngle - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);
  // this->dataPtr->flWheelSteeringJoint->SetPosition(0,
  // this->dataPtr->flWheelSteeringCmd);
  // this->dataPtr->flWheelSteeringJoint->SetLowStop(0,
  // this->dataPtr->flWheelSteeringCmd);
  // this->dataPtr->flWheelSteeringJoint->SetHighStop(0,
  // this->dataPtr->flWheelSteeringCmd);

  double frwsError =
      this->dataPtr->frSteeringAngle - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);
  // this->dataPtr->frWheelSteeringJoint->SetPosition(0,
  // this->dataPtr->frWheelSteeringCmd);
  // this->dataPtr->frWheelSteeringJoint->SetLowStop(0,
  // this->dataPtr->frWheelSteeringCmd);
  // this->dataPtr->frWheelSteeringJoint->SetHighStop(0,
  // this->dataPtr->frWheelSteeringCmd);

  //static common::Time lastErrorPrintTime = 0.0;
  //if (curTime - lastErrorPrintTime > 0.01 || curTime < lastErrorPrintTime)
  //{
  //  lastErrorPrintTime = curTime;
  //  double maxSteerError =
  //    std::abs(frwsError) > std::abs(flwsError) ? frwsError : flwsError;
  //  double maxSteerErrPer = maxSteerError / this->dataPtr->maxSteer * 100.0;
  //  std::cerr << std::fixed << "Max steering error: " << maxSteerErrPer
  //    << std::endl;
  //}

  // Model low-speed caaaareep and high-speed regen braking
  // with term added to gas/brake
  // Cross-over speed is 7 miles/hour
  // 10% throttle at 0 speed
  // max 2.5% braking at higher speeds
  double creepPercent;
  if (std::abs(linearVel) <= 7)
  {
    creepPercent = 0.1 * (1 - std::abs(linearVel) / 7);
  }
  else
  {
    creepPercent = 0.025 * (7 - std::abs(linearVel));
  }
  creepPercent = ignition::math::clamp(creepPercent, -0.025, 0.1);

  // Gas pedal torque.
  // Map gas torques to individual wheels.
  // Cut off gas torque at a given wheel if max speed is exceeded.
  // Use directionState to determine direction of that can be applied torque.
  // Note that definition of DirectionType allows multiplication to determine
  // torque direction.
  // also, make sure gas pedal is at least as large as the creepPercent.
  double gasPercent = std::max(this->dataPtr->gasPedalPercent, creepPercent);
  double gasMultiplier = this->GasTorqueMultiplier();
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  // Apply equal torque at left and right wheels, which is an implicit model
  // of the differential.
  if (fabs(dPtr->flWheelAngularVelocity * dPtr->flWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->frWheelAngularVelocity * dPtr->frWheelRadius) < dPtr->maxSpeed)
  {
    flGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
    frGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
  }
  if (fabs(dPtr->blWheelAngularVelocity * dPtr->blWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->brWheelAngularVelocity * dPtr->brWheelRadius) < dPtr->maxSpeed)
  {
    blGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
    brGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
  }
  double throttlePower =
      std::abs(flGasTorque * dPtr->flWheelAngularVelocity) +
      std::abs(frGasTorque * dPtr->frWheelAngularVelocity) +
      std::abs(blGasTorque * dPtr->blWheelAngularVelocity) +
      std::abs(brGasTorque * dPtr->brWheelAngularVelocity);

  // auto release handbrake as soon as the gas pedal is depressed
  if (this->dataPtr->gasPedalPercent > 0)
    this->dataPtr->handbrakePercent = 0.0;

  double brakePercent = this->dataPtr->brakePedalPercent
      + this->dataPtr->handbrakePercent;
  // use creep braking if not in Neutral
  if (!neutral)
  {
    brakePercent = std::max(brakePercent,
        -creepPercent - this->dataPtr->gasPedalPercent);
  }

  brakePercent = ignition::math::clamp(brakePercent, 0.0, 1.0);
  dPtr->flWheelJoint->SetParam("friction", 0,
      dPtr->flJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->frWheelJoint->SetParam("friction", 0,
      dPtr->frJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->blWheelJoint->SetParam("friction", 0,
      dPtr->blJointFriction + brakePercent * dPtr->backBrakeTorque);
  dPtr->brWheelJoint->SetParam("friction", 0,
      dPtr->brJointFriction + brakePercent * dPtr->backBrakeTorque);

  this->dataPtr->flWheelJoint->SetForce(0, flGasTorque);
  this->dataPtr->frWheelJoint->SetForce(0, frGasTorque);
  this->dataPtr->blWheelJoint->SetForce(0, blGasTorque);
  this->dataPtr->brWheelJoint->SetForce(0, brGasTorque);

  // gzerr << "gas and brake torque " << flGasTorque << " "
  //       << flBrakeTorque << std::endl;

  // Battery

  // Speed x throttle regions
  //
  //    throttle |
  //             |
  //        high |____
  //             |    |
  //      medium |____|_____
  //             |    |     |
  //         low |____|_____|_________
  //              low  med   high    speed

  bool engineOn;
  bool regen = !neutral;
  double batteryChargePower = 0;
  double batteryDischargePower = 0;

  // Battery is below threshold
  if (this->dataPtr->batteryCharge < this->dataPtr->batteryLowThreshold)
  {
    // Gas engine is on and recharing battery
    engineOn = true;
    this->dataPtr->evMode = false;
    batteryChargePower = dPtr->kLowBatteryChargePower;
    throttlePower += dPtr->kLowBatteryChargePower;
  }
  // Neutral and battery not low
  else if (neutral)
  {
    // Gas engine is off, battery not recharged
    engineOn = false;
  }
  // Speed below medium-high threshold, throttle below low-medium threshold
  else if (linearVel < this->dataPtr->speedMediumHigh &&
      this->dataPtr->gasPedalPercent <= this->dataPtr->kGasPedalLowMedium)
  {
    // Gas engine is off, running on battery
    engineOn = false;
    batteryDischargePower = throttlePower;
  }
  // EV mode, speed below low-medium threshold, throttle below medium-high
  // threshold
  else if (this->dataPtr->evMode && linearVel < this->dataPtr->speedLowMedium
      && this->dataPtr->gasPedalPercent <= this->dataPtr->kGasPedalMediumHigh)
  {
    // Gas engine is off, running on battery
    engineOn = false;
    batteryDischargePower = throttlePower;
  }
  else
  {
    // Gas engine is on
    engineOn = true;
    this->dataPtr->evMode = false;
  }

  if (regen)
  {
    // regen max torque at same level as throttle limit in EV mode
    // but only at the front wheels
    batteryChargePower +=
      std::min(this->dataPtr->kGasPedalMediumHigh, brakePercent)*(
        dPtr->frontBrakeTorque * std::abs(dPtr->flWheelAngularVelocity) +
        dPtr->frontBrakeTorque * std::abs(dPtr->frWheelAngularVelocity) +
        dPtr->backBrakeTorque * std::abs(dPtr->blWheelAngularVelocity) +
        dPtr->backBrakeTorque * std::abs(dPtr->brWheelAngularVelocity));
  }
  dPtr->batteryCharge += dt / 3600 * (
      batteryChargePower / dPtr->batteryChargeWattHours
    - batteryDischargePower / dPtr->batteryDischargeWattHours);
  if (dPtr->batteryCharge > 1)
  {
    dPtr->batteryCharge = 1;
  }

  // engine has minimum gas flow if the throttle is pressed at all
  if (engineOn && throttlePower > 0)
  {
    dPtr->gasConsumption += dt*(dPtr->minGasFlow
        + throttlePower / dPtr->gasEfficiency / dPtr->kGasEnergyDensity);
  }

  // Accumulated mpg since last reset
  // max value: 999.9
  double mpg = std::min(999.9,
      dPtr->odom / std::max(dPtr->gasConsumption, 1e-6));

  if ((curTime - this->dataPtr->lastMsgTime) > .5)
  {
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    ignition::msgs::Double_V consoleMsg;

    // linearVel (meter/sec) = (2*PI*r) * (rad/sec).
    double linearVel = (2.0 * IGN_PI * this->dataPtr->flWheelRadius) *
      ((this->dataPtr->flWheelAngularVelocity +
        this->dataPtr->frWheelAngularVelocity) * 0.5);

    // Convert meter/sec to miles/hour
    linearVel *= 2.23694;

    // Distance traveled in miles.
    this->dataPtr->odom += (fabs(linearVel) * dt/3600);

    // \todo: Actually compute MPG
    double mpg = 1.0 / std::max(linearVel, 0.0);

    // Gear information: 1=drive, 2=reverse, 3=neutral
    if (this->dataPtr->directionState == PriusHybridPluginPrivate::FORWARD)
      consoleMsg.add_data(1.0);
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::REVERSE)
      consoleMsg.add_data(2.0);
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::NEUTRAL)
      consoleMsg.add_data(3.0);

    // MPH. A speedometer does not go negative.
    consoleMsg.add_data(std::max(linearVel, 0.0));

    // MPG
    consoleMsg.add_data(mpg);

    // Miles
    consoleMsg.add_data(this->dataPtr->odom);

    // EV mode
    this->dataPtr->evMode ? consoleMsg.add_data(1.0) : consoleMsg.add_data(0.0);

    // Battery state
    consoleMsg.add_data(this->dataPtr->batteryCharge);

    this->dataPtr->consolePub.Publish(consoleMsg);

    // Output prius car data.
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    this->dataPtr->lastMsgTime = curTime;
  }

  // reset if last command is more than x sec ago
  if ((curTime - this->dataPtr->lastPedalCmdTime).Double() > 0.3)
  {
    this->dataPtr->gasPedalPercent = 0.0;
    this->dataPtr->brakePedalPercent = 0.0;
  }

  if ((curTime - this->dataPtr->lastSteeringCmdTime).Double() > 0.3)
  {
    this->dataPtr->handWheelCmd = 0;
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->dataPtr->handWheelHigh = 7.85;
  this->dataPtr->handWheelLow = -7.85;
  double handWheelRange =
      this->dataPtr->handWheelHigh - this->dataPtr->handWheelLow;
  double high = 0.8727;
  high = std::min(high, this->dataPtr->maxSteer);
  double low = -0.8727;
  low = std::max(low, -this->dataPtr->maxSteer);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->dataPtr->steeringRatio = tireAngleRange / handWheelRange;
}

/////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double PriusHybridPlugin::CollisionRadius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

/////////////////////////////////////////////////
double PriusHybridPlugin::GasTorqueMultiplier()
{
  // if (this->dataPtr->keyState == ON)
  {
    if (this->dataPtr->directionState == PriusHybridPluginPrivate::FORWARD)
      return 1.0;
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::REVERSE)
      return -1.0;
  }
  return 0;
}

GZ_REGISTER_MODEL_PLUGIN(PriusHybridPlugin)
