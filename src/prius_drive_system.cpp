#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/msgs/int32.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace offroad_gazebo_integration
{
namespace
{
constexpr double kDefaultWheelBaseM = 2.86;
constexpr double kDefaultFrontTrackWidthM = 1.534;
constexpr double kDefaultWheelRadiusM = 0.31265;
constexpr double kDefaultMaxSteeringAngleRad = 0.6458;
constexpr double kDefaultMaxSpeedMps = 10.0;
constexpr double kDefaultFrontAxleDriveTorqueNm = 859.4004393;
constexpr double kDefaultRearAxleDriveTorqueNm = 0.0;
constexpr double kDefaultFrontAxleBrakeTorqueNm = 1031.28052716;
constexpr double kDefaultRearAxleBrakeTorqueNm = 687.52035144;
constexpr double kDefaultSteeringKp = 1.0e4;
constexpr double kDefaultSteeringKd = 3.0e2;
constexpr double kDefaultSteeringForceLimitNm = 2.0e3;
constexpr double kDefaultDriveSpeedGain = 125.0;
constexpr double kDefaultBrakeDampingGain = 800.0;
constexpr int32_t kGearParking = 1;
constexpr int32_t kGearReverse = 2;
constexpr int32_t kGearNeutral = 3;
constexpr int32_t kGearDriving = 4;

double Clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double FirstJointValue(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::gazebo::Entity _entity,
    const bool readPosition)
{
  if (_entity == ignition::gazebo::kNullEntity)
  {
    return 0.0;
  }

  if (readPosition)
  {
    const auto *position = _ecm.Component<ignition::gazebo::components::JointPosition>(_entity);
    if (position != nullptr && !position->Data().empty())
    {
      return position->Data().front();
    }
    return 0.0;
  }

  const auto *velocity = _ecm.Component<ignition::gazebo::components::JointVelocity>(_entity);
  if (velocity != nullptr && !velocity->Data().empty())
  {
    return velocity->Data().front();
  }
  return 0.0;
}

void EnsureJointStateComponents(
    ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::gazebo::Entity _jointEntity)
{
  if (_jointEntity == ignition::gazebo::kNullEntity)
  {
    return;
  }

  if (_ecm.Component<ignition::gazebo::components::JointPosition>(_jointEntity) == nullptr)
  {
    _ecm.CreateComponent(
        _jointEntity,
        ignition::gazebo::components::JointPosition({0.0}));
  }

  if (_ecm.Component<ignition::gazebo::components::JointVelocity>(_jointEntity) == nullptr)
  {
    _ecm.CreateComponent(
        _jointEntity,
        ignition::gazebo::components::JointVelocity({0.0}));
  }

  if (_ecm.Component<ignition::gazebo::components::JointForceCmd>(_jointEntity) == nullptr)
  {
    _ecm.CreateComponent(
        _jointEntity,
        ignition::gazebo::components::JointForceCmd({0.0}));
  }
}

void SetJointForce(
    ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::gazebo::Entity _jointEntity,
    const double _force)
{
  if (_jointEntity == ignition::gazebo::kNullEntity)
  {
    return;
  }

  auto *forceComponent =
      _ecm.Component<ignition::gazebo::components::JointForceCmd>(_jointEntity);
  if (forceComponent == nullptr)
  {
    _ecm.CreateComponent(
        _jointEntity,
        ignition::gazebo::components::JointForceCmd({_force}));
    return;
  }

  forceComponent->SetData(
      {_force},
      [](const std::vector<double> &_left, const std::vector<double> &_right)
      {
        return _left == _right;
      });
}

std::pair<double, double> AckermannSteeringAngles(
    const double _centerAngle,
    const double _frontTrackWidthM,
    const double _wheelBaseM)
{
  if (std::abs(_centerAngle) < 1e-6 || std::abs(_wheelBaseM) < 1e-6)
  {
    return {_centerAngle, _centerAngle};
  }

  const double tangent = std::tan(_centerAngle);
  const double widthRatio = _frontTrackWidthM / (2.0 * _wheelBaseM);
  const double leftAngle = std::atan2(tangent, 1.0 - widthRatio * tangent);
  const double rightAngle = std::atan2(tangent, 1.0 + widthRatio * tangent);
  return {leftAngle, rightAngle};
}

double SteeringTorqueCommand(
    const double _targetAngle,
    const double _currentAngle,
    const double _currentVelocity,
    const double _kp,
    const double _kd,
    const double _limit)
{
  const double error = _targetAngle - _currentAngle;
  const double rawTorque = (_kp * error) - (_kd * _currentVelocity);
  return Clamp(rawTorque, -_limit, _limit);
}

double BrakeTorqueCommand(
    const double _wheelVelocity,
    const double _limit,
    const double _dampingGain)
{
  if (_limit <= 0.0)
  {
    return 0.0;
  }

  return Clamp(-_wheelVelocity * _dampingGain, -_limit, _limit);
}
}  // namespace

class PriusDriveSystem:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager & /*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      std::cerr << "PriusDriveSystem must be attached to a model entity" << std::endl;
      return;
    }

    if (_sdf != nullptr)
    {
      if (_sdf->HasElement("drive_topic"))
      {
        this->driveTopic = _sdf->Get<std::string>("drive_topic");
      }
      if (_sdf->HasElement("gear_topic"))
      {
        this->gearTopic = _sdf->Get<std::string>("gear_topic");
      }
      if (_sdf->HasElement("wheel_base"))
      {
        this->wheelBaseM = _sdf->Get<double>("wheel_base");
      }
      if (_sdf->HasElement("front_track_width"))
      {
        this->frontTrackWidthM = _sdf->Get<double>("front_track_width");
      }
      if (_sdf->HasElement("wheel_radius"))
      {
        this->wheelRadiusM = _sdf->Get<double>("wheel_radius");
      }
      if (_sdf->HasElement("max_steer"))
      {
        this->maxSteeringAngleRad = _sdf->Get<double>("max_steer");
      }
      if (_sdf->HasElement("max_speed"))
      {
        this->maxSpeedMps = _sdf->Get<double>("max_speed");
      }
      if (_sdf->HasElement("front_torque"))
      {
        this->frontAxleDriveTorqueNm = _sdf->Get<double>("front_torque");
      }
      if (_sdf->HasElement("back_torque"))
      {
        this->rearAxleDriveTorqueNm = _sdf->Get<double>("back_torque");
      }
      if (_sdf->HasElement("front_brake_torque"))
      {
        this->frontAxleBrakeTorqueNm = _sdf->Get<double>("front_brake_torque");
      }
      if (_sdf->HasElement("back_brake_torque"))
      {
        this->rearAxleBrakeTorqueNm = _sdf->Get<double>("back_brake_torque");
      }
      if (_sdf->HasElement("flwheel_steering_p_gain"))
      {
        this->steeringKp = _sdf->Get<double>("flwheel_steering_p_gain");
      }
      if (_sdf->HasElement("flwheel_steering_d_gain"))
      {
        this->steeringKd = _sdf->Get<double>("flwheel_steering_d_gain");
      }
      if (_sdf->HasElement("steering_force_limit"))
      {
        this->steeringForceLimitNm = _sdf->Get<double>("steering_force_limit");
      }
      if (_sdf->HasElement("drive_speed_gain"))
      {
        this->driveSpeedGain = _sdf->Get<double>("drive_speed_gain");
      }
      if (_sdf->HasElement("brake_damping_gain"))
      {
        this->brakeDampingGain = _sdf->Get<double>("brake_damping_gain");
      }
    }

    this->frontLeftSteeringJoint = this->model.JointByName(_ecm, "front_left_steer_joint");
    this->frontRightSteeringJoint = this->model.JointByName(_ecm, "front_right_steer_joint");
    this->frontLeftWheelJoint = this->model.JointByName(_ecm, "front_left_wheel_joint");
    this->frontRightWheelJoint = this->model.JointByName(_ecm, "front_right_wheel_joint");
    this->rearLeftWheelJoint = this->model.JointByName(_ecm, "rear_left_wheel_joint");
    this->rearRightWheelJoint = this->model.JointByName(_ecm, "rear_right_wheel_joint");

    const std::vector<ignition::gazebo::Entity> requiredJoints = {
        this->frontLeftSteeringJoint,
        this->frontRightSteeringJoint,
        this->frontLeftWheelJoint,
        this->frontRightWheelJoint,
        this->rearLeftWheelJoint,
        this->rearRightWheelJoint,
    };
    if (std::any_of(
            requiredJoints.begin(),
            requiredJoints.end(),
            [](const ignition::gazebo::Entity _joint)
            {
              return _joint == ignition::gazebo::kNullEntity;
            }))
    {
      std::cerr << "PriusDriveSystem failed to resolve one or more Prius joints" << std::endl;
      this->configured = false;
      return;
    }

    for (const auto joint : requiredJoints)
    {
      EnsureJointStateComponents(_ecm, joint);
    }

    // Transport callbacks only publish the latest independent control values.
    // Relaxed atomics are sufficient because PreUpdate can tolerate a one-tick
    // mixed snapshot without violating correctness or safety.
    if (!this->transportNode.Subscribe(
            this->driveTopic,
            &PriusDriveSystem::OnDriveCommand,
            this))
    {
      std::cerr << "PriusDriveSystem failed to subscribe to drive topic " << this->driveTopic
                << std::endl;
      this->configured = false;
      return;
    }

    if (!this->transportNode.Subscribe(
            this->gearTopic,
            &PriusDriveSystem::OnGearCommand,
            this))
    {
      std::cerr << "PriusDriveSystem failed to subscribe to gear topic " << this->gearTopic
                << std::endl;
      this->configured = false;
      return;
    }

    this->configured = true;
  }

  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
    {
      return;
    }

    const double throttle = this->commandThrottle.load(std::memory_order_relaxed);
    const double brake = this->commandBrake.load(std::memory_order_relaxed);
    const double steeringInput = this->commandSteering.load(std::memory_order_relaxed);
    const int32_t gear = this->commandGear.load(std::memory_order_relaxed);

    const double centerAngle =
        Clamp(-steeringInput, -1.0, 1.0) * this->maxSteeringAngleRad;
    const auto [leftAngle, rightAngle] = AckermannSteeringAngles(
        centerAngle, this->frontTrackWidthM, this->wheelBaseM);

    const double leftSteerPos =
        FirstJointValue(_ecm, this->frontLeftSteeringJoint, true);
    const double rightSteerPos =
        FirstJointValue(_ecm, this->frontRightSteeringJoint, true);
    const double leftSteerVel =
        FirstJointValue(_ecm, this->frontLeftSteeringJoint, false);
    const double rightSteerVel =
        FirstJointValue(_ecm, this->frontRightSteeringJoint, false);

    SetJointForce(
        _ecm,
        this->frontLeftSteeringJoint,
        SteeringTorqueCommand(
            leftAngle,
            leftSteerPos,
            leftSteerVel,
            this->steeringKp,
            this->steeringKd,
            this->steeringForceLimitNm));
    SetJointForce(
        _ecm,
        this->frontRightSteeringJoint,
        SteeringTorqueCommand(
            rightAngle,
            rightSteerPos,
            rightSteerVel,
            this->steeringKp,
            this->steeringKd,
            this->steeringForceLimitNm));

    const bool canDrive = (gear == kGearDriving) || (gear == kGearReverse);
    const double direction = (gear == kGearReverse) ? -1.0 : 1.0;
    const double targetVehicleSpeedMps =
        canDrive ? direction * throttle * this->maxSpeedMps : 0.0;
    const double targetWheelSpeedRadPerSec =
        targetVehicleSpeedMps / std::max(this->wheelRadiusM, 1e-6);

    const double frontLeftWheelVel =
        FirstJointValue(_ecm, this->frontLeftWheelJoint, false);
    const double frontRightWheelVel =
        FirstJointValue(_ecm, this->frontRightWheelJoint, false);
    const double rearLeftWheelVel =
        FirstJointValue(_ecm, this->rearLeftWheelJoint, false);
    const double rearRightWheelVel =
        FirstJointValue(_ecm, this->rearRightWheelJoint, false);

    const double frontAverageWheelVel = 0.5 * (frontLeftWheelVel + frontRightWheelVel);
    const double rearAverageWheelVel = 0.5 * (rearLeftWheelVel + rearRightWheelVel);
    const double frontDriveTorquePerWheel = canDrive
        ? Clamp(
              (targetWheelSpeedRadPerSec - frontAverageWheelVel) * this->driveSpeedGain,
              -(this->frontAxleDriveTorqueNm * 0.5),
              this->frontAxleDriveTorqueNm * 0.5)
        : 0.0;
    const double rearDriveTorquePerWheel = canDrive
        ? Clamp(
              (targetWheelSpeedRadPerSec - rearAverageWheelVel) * this->driveSpeedGain,
              -(this->rearAxleDriveTorqueNm * 0.5),
              this->rearAxleDriveTorqueNm * 0.5)
        : 0.0;

    const double brakeDemand = (gear == kGearParking) ? 1.0 : brake;
    const double frontBrakeTorquePerWheel =
        brakeDemand * (this->frontAxleBrakeTorqueNm * 0.5);
    const double rearBrakeTorquePerWheel =
        brakeDemand * (this->rearAxleBrakeTorqueNm * 0.5);

    SetJointForce(
        _ecm,
        this->frontLeftWheelJoint,
        frontDriveTorquePerWheel +
            BrakeTorqueCommand(
                frontLeftWheelVel,
                frontBrakeTorquePerWheel,
                this->brakeDampingGain));
    SetJointForce(
        _ecm,
        this->frontRightWheelJoint,
        frontDriveTorquePerWheel +
            BrakeTorqueCommand(
                frontRightWheelVel,
                frontBrakeTorquePerWheel,
                this->brakeDampingGain));
    SetJointForce(
        _ecm,
        this->rearLeftWheelJoint,
        rearDriveTorquePerWheel +
            BrakeTorqueCommand(
                rearLeftWheelVel,
                rearBrakeTorquePerWheel,
                this->brakeDampingGain));
    SetJointForce(
        _ecm,
        this->rearRightWheelJoint,
        rearDriveTorquePerWheel +
            BrakeTorqueCommand(
                rearRightWheelVel,
                rearBrakeTorquePerWheel,
                this->brakeDampingGain));
  }

  private: void OnDriveCommand(const ignition::msgs::Vector3d &_msg)
  {
    this->commandThrottle.store(
        Clamp(_msg.x(), 0.0, 1.0), std::memory_order_relaxed);
    this->commandBrake.store(
        Clamp(_msg.y(), 0.0, 1.0), std::memory_order_relaxed);
    this->commandSteering.store(
        Clamp(_msg.z(), -1.0, 1.0), std::memory_order_relaxed);
  }

  private: void OnGearCommand(const ignition::msgs::Int32 &_msg)
  {
    const int32_t requestedGear = _msg.data();
    switch (requestedGear)
    {
      case kGearParking:
      case kGearReverse:
      case kGearNeutral:
      case kGearDriving:
        this->commandGear.store(requestedGear, std::memory_order_relaxed);
        break;
      default:
        this->commandGear.store(kGearNeutral, std::memory_order_relaxed);
        break;
    }
  }

  private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity frontLeftSteeringJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity frontRightSteeringJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity frontLeftWheelJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity frontRightWheelJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity rearLeftWheelJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity rearRightWheelJoint{ignition::gazebo::kNullEntity};

  private: ignition::transport::Node transportNode;
  private: std::string driveTopic{"/cmd_drive"};
  private: std::string gearTopic{"/cmd_gear"};

  private: double wheelBaseM{kDefaultWheelBaseM};
  private: double frontTrackWidthM{kDefaultFrontTrackWidthM};
  private: double wheelRadiusM{kDefaultWheelRadiusM};
  private: double maxSteeringAngleRad{kDefaultMaxSteeringAngleRad};
  private: double maxSpeedMps{kDefaultMaxSpeedMps};
  private: double frontAxleDriveTorqueNm{kDefaultFrontAxleDriveTorqueNm};
  private: double rearAxleDriveTorqueNm{kDefaultRearAxleDriveTorqueNm};
  private: double frontAxleBrakeTorqueNm{kDefaultFrontAxleBrakeTorqueNm};
  private: double rearAxleBrakeTorqueNm{kDefaultRearAxleBrakeTorqueNm};
  private: double steeringKp{kDefaultSteeringKp};
  private: double steeringKd{kDefaultSteeringKd};
  private: double steeringForceLimitNm{kDefaultSteeringForceLimitNm};
  private: double driveSpeedGain{kDefaultDriveSpeedGain};
  private: double brakeDampingGain{kDefaultBrakeDampingGain};

  private: std::atomic<double> commandThrottle{0.0};
  private: std::atomic<double> commandBrake{0.0};
  private: std::atomic<double> commandSteering{0.0};
  private: std::atomic<int32_t> commandGear{kGearParking};
  private: bool configured{false};
};
}  // namespace offroad_gazebo_integration

IGNITION_ADD_PLUGIN(
    offroad_gazebo_integration::PriusDriveSystem,
    ignition::gazebo::System,
    offroad_gazebo_integration::PriusDriveSystem::ISystemConfigure,
    offroad_gazebo_integration::PriusDriveSystem::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    offroad_gazebo_integration::PriusDriveSystem,
    "offroad_gazebo_integration::PriusDriveSystem")
