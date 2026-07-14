#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace crazyflie_lqr_gz_plugin
{

class CrazyflieLqrController:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);

    if (!this->model.Valid(_ecm))
    {
      gzerr << "[CrazyflieLqrController] Invalid model entity\n";
      return;
    }

    this->motorTopic = "/crazyflie/command/motor_speed";

    this->motorPublisher =
      this->node.Advertise<gz::msgs::Actuators>(this->motorTopic);

    const bool subscribed =
      this->node.Subscribe(
        "/crazyflie/desired_state",
        &CrazyflieLqrController::OnDesiredPose,
        this);

    if (subscribed)
    {
      gzmsg
        << "[CrazyflieLqrController] Listening for desired state on "
        << "/crazyflie/desired_state\n";
    }
    else
    {
      gzerr
        << "[CrazyflieLqrController] Failed to subscribe to "
        << "/crazyflie/desired_state\n";
    }

    gzmsg
      << "[CrazyflieLqrController] Plugin loaded for model: "
      << this->model.Name(_ecm) << "\n";

    gzmsg
      << "[CrazyflieLqrController] Motor topic: "
      << this->motorTopic << "\n";

    gzmsg
      << "[CrazyflieLqrController] Discrete LQR enabled\n"
      << "[CrazyflieLqrController] Sampling time: 0.001 s\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    const gz::sim::Entity link =
      this->model.CanonicalLink(_ecm);

    if (link == gz::sim::kNullEntity)
    {
      if (!this->linkErrorPrinted)
      {
        gzerr
          << "[CrazyflieLqrController] "
          << "Could not find canonical link\n";

        this->linkErrorPrinted = true;
      }

      return;
    }

    gz::sim::Link bodyLink(link);

    bodyLink.EnableVelocityChecks(_ecm, true);

    const auto poseOptional =
      bodyLink.WorldPose(_ecm);

    const auto linearVelocityOptional =
      bodyLink.WorldLinearVelocity(_ecm);

    const auto angularVelocityOptional =
      bodyLink.WorldAngularVelocity(_ecm);

    if (!poseOptional ||
        !linearVelocityOptional ||
        !angularVelocityOptional)
    {
      return;
    }

    const auto pose =
      poseOptional.value();

    const auto linearVelocityWorld =
      linearVelocityOptional.value();

    const auto angularVelocityWorld =
      angularVelocityOptional.value();

    // Convert world-frame angular velocity to body-frame rates.
    const auto angularVelocityBody =
      pose.Rot().Inverse().RotateVector(angularVelocityWorld);

    // ----------------------------------------------------------
    // Current state
    //
    // state ordering:
    //
    // x = [
    //   X, Y, Z,
    //   Vx, Vy, Vz,
    //   roll, pitch, yaw,
    //   p, q, r
    // ]
    // ----------------------------------------------------------

    const double x =
      pose.Pos().X();

    const double y =
      pose.Pos().Y();

    const double z =
      pose.Pos().Z();

    const double vx =
      linearVelocityWorld.X();

    const double vy =
      linearVelocityWorld.Y();

    const double vz =
      linearVelocityWorld.Z();

    const double roll =
      pose.Rot().Roll();

    const double pitch =
      pose.Rot().Pitch();

    const double yaw =
      pose.Rot().Yaw();

    const double p =
      angularVelocityBody.X();

    const double q =
      angularVelocityBody.Y();

    const double r =
      angularVelocityBody.Z();

    const double simTime =
      std::chrono::duration<double>(_info.simTime).count();

    const double dt =
      std::chrono::duration<double>(_info.dt).count();

    // ----------------------------------------------------------
    // Desired state received from external Python waypoint node
    //
    // gz::msgs::Pose field mapping:
    //
    // position.x    = desired X
    // position.y    = desired Y
    // position.z    = desired Z
    //
    // orientation.x = desired Vx
    // orientation.y = desired Vy
    // orientation.z = desired Vz
    // orientation.w = desired yaw
    // ----------------------------------------------------------

    double desiredX = 0.0;
    double desiredY = 0.0;
    double desiredZ = 0.0;

    double desiredVx = 0.0;
    double desiredVy = 0.0;
    double desiredVz = 0.0;

    double desiredYaw = 0.0;

    {
      std::lock_guard<std::mutex> lock(
        this->referenceMutex);

      if (this->receivedExternalReference)
      {
        desiredX =
          this->externalDesiredX;

        desiredY =
          this->externalDesiredY;

        desiredZ =
          this->externalDesiredZ;

        desiredVx =
          this->externalDesiredVx;

        desiredVy =
          this->externalDesiredVy;

        desiredVz =
          this->externalDesiredVz;

        desiredYaw =
          this->externalDesiredYaw;
      }
    }

    const double desiredRoll = 0.0;
    const double desiredPitch = 0.0;

    const double desiredP = 0.0;
    const double desiredQ = 0.0;
    const double desiredR = 0.0;

    // Keep the yaw error in [-pi, pi].
    const double yawError =
      std::atan2(
        std::sin(yaw - desiredYaw),
        std::cos(yaw - desiredYaw));

    // ----------------------------------------------------------
    // State error
    //
    // e_k = x_k - x_desired,k
    // ----------------------------------------------------------

    const double stateError[12] =
    {
      x - desiredX,
      y - desiredY,
      z - desiredZ,

      vx - desiredVx,
      vy - desiredVy,
      vz - desiredVz,

      roll - desiredRoll,
      pitch - desiredPitch,
      yawError,

      p - desiredP,
      q - desiredQ,
      r - desiredR
    };

    // ----------------------------------------------------------
    // Discrete LQR gain
    //
    // Sampling time:
    // Ts = 0.001 s
    //
    // Control law:
    // u_k = -Kd * e_k
    //
    // Inputs:
    // u = [
    //   collective,
    //   roll,
    //   pitch,
    //   yaw
    // ]
    // ----------------------------------------------------------
    static constexpr double Kd[4][12] =
    {
      {
        0.000000000000,
        -0.000000000008,
        26.823685246871,
        0.000000000000,
        -0.000000000002,
        80.566689040912,
        0.000000000002,
        0.000000000000,
        0.000000000082,
        0.000000000000,
        -0.000000000000,
        0.000000000114
      },
      {
        -0.000000000001,
        -57.385079402819,
        0.000000000013,
        -0.000000000001,
        -41.495527975778,
        0.000000000025,
        142.955797002037,
        -0.000000000002,
        -0.000000000024,
        27.194019619810,
        -0.000000000000,
        -0.000000000080
      },
      {
        57.385815056659,
        -0.000000000000,
        0.000000000000,
        41.531048677455,
        0.000000000000,
        -0.000000000000,
        -0.000000000000,
        143.205933082420,
        0.000000000000,
        -0.000000000000,
        27.274317615381,
        0.000000000000
      },
      {
        0.000000000000,
        0.000000000013,
        -0.000000000037,
        -0.000000000000,
        0.000000000007,
        0.000000000110,
        -0.000000000014,
        0.000000000000,
        5.771311505144,
        -0.000000000001,
        0.000000000000,
        15.632889699658
      }
    };

    double control[4] =
    {
      0.0,
      0.0,
      0.0,
      0.0
    };

    for (int inputIndex = 0;
         inputIndex < 4;
         ++inputIndex)
    {
      for (int stateIndex = 0;
           stateIndex < 12;
           ++stateIndex)
      {
        control[inputIndex] -=
          Kd[inputIndex][stateIndex] *
          stateError[stateIndex];
      }
    }

    // ----------------------------------------------------------
    // Virtual LQR inputs
    // ----------------------------------------------------------

    // Convert virtual LQR inputs into motor-speed increments.
    static constexpr double collectiveScale = 1.0;
    static constexpr double rollScale = 1.0;
    static constexpr double pitchScale = 1.0;
    static constexpr double yawScale = 1.0;

    double collectiveInput =
      collectiveScale * control[0];

    double rollInput =
      rollScale * control[1];

    double pitchInput =
      pitchScale * control[2];

    double yawInput =
      -yawScale * control[3];

    // These limits protect the nonlinear simulation while
    // keeping the LQR law unchanged inside the permitted range.
    collectiveInput =
      std::clamp(
        collectiveInput,
        -150.0,
        150.0);

    rollInput =
      std::clamp(
        rollInput,
        -100.0,
        100.0);

    pitchInput =
      std::clamp(
        pitchInput,
        -100.0,
        100.0);

    yawInput =
      std::clamp(
        yawInput,
        -40.0,
        40.0);

    // ----------------------------------------------------------
    // Hover feedforward
    //
    // omega_hover =
    // sqrt(m*g / (4*kf))
    // ----------------------------------------------------------

    static constexpr double hoverOmega =
      2321.53;

    // ----------------------------------------------------------
    // Motor mixer
    //
    // m1: (+x, -y), CCW
    // m2: (-x, -y), CW
    // m3: (-x, +y), CCW
    // m4: (+x, +y), CW
    // ----------------------------------------------------------

    double omega1 =
      hoverOmega
      + collectiveInput
      - rollInput
      - pitchInput
      + yawInput;

    double omega2 =
      hoverOmega
      + collectiveInput
      - rollInput
      + pitchInput
      - yawInput;

    double omega3 =
      hoverOmega
      + collectiveInput
      + rollInput
      + pitchInput
      + yawInput;

    double omega4 =
      hoverOmega
      + collectiveInput
      + rollInput
      - pitchInput
      - yawInput;

    static constexpr double minimumOmega =
      0.0;

    static constexpr double maximumOmega =
      2618.0;

    omega1 =
      std::clamp(
        omega1,
        minimumOmega,
        maximumOmega);

    omega2 =
      std::clamp(
        omega2,
        minimumOmega,
        maximumOmega);

    omega3 =
      std::clamp(
        omega3,
        minimumOmega,
        maximumOmega);

    omega4 =
      std::clamp(
        omega4,
        minimumOmega,
        maximumOmega);

    // ----------------------------------------------------------
    // Publish motor speeds
    // ----------------------------------------------------------

    gz::msgs::Actuators motorMessage;

    motorMessage.mutable_velocity()->Add(omega1);
    motorMessage.mutable_velocity()->Add(omega2);
    motorMessage.mutable_velocity()->Add(omega3);
    motorMessage.mutable_velocity()->Add(omega4);

    this->motorPublisher.Publish(motorMessage);

    // ----------------------------------------------------------
    // Logging
    // ----------------------------------------------------------

    if (simTime - this->lastPrintTime >= 0.25)
    {
      gzmsg
        << "[Discrete LQR] "
        << "dt=" << dt << " "
        << "t=" << simTime << " "

        << "desired=["
        << desiredX << ", "
        << desiredY << ", "
        << desiredZ << ", "
        << desiredYaw << "] "

        << "pos=["
        << x << ", "
        << y << ", "
        << z << "] "

        << "vel=["
        << vx << ", "
        << vy << ", "
        << vz << "] "

        << "rpy=["
        << roll << ", "
        << pitch << ", "
        << yaw << "] "

        << "bodyRates=["
        << p << ", "
        << q << ", "
        << r << "] "

        << "u=["
        << collectiveInput << ", "
        << rollInput << ", "
        << pitchInput << ", "
        << yawInput << "] "

        << "omega=["
        << omega1 << ", "
        << omega2 << ", "
        << omega3 << ", "
        << omega4 << "]\n";

      this->lastPrintTime =
        simTime;
    }
  }

private:
  void OnDesiredPose(
    const gz::msgs::Pose &_message)
  {
    std::lock_guard<std::mutex> lock(
      this->referenceMutex);

    this->externalDesiredX =
      _message.position().x();

    this->externalDesiredY =
      _message.position().y();

    this->externalDesiredZ =
      _message.position().z();

    this->externalDesiredVx =
      _message.orientation().x();

    this->externalDesiredVy =
      _message.orientation().y();

    this->externalDesiredVz =
      _message.orientation().z();

    this->externalDesiredYaw =
      _message.orientation().w();

    this->receivedExternalReference = true;
  }

  gz::sim::Model model{
    gz::sim::kNullEntity
  };

  gz::transport::Node node;

  std::mutex referenceMutex;

  double externalDesiredX{0.0};
  double externalDesiredY{0.0};
  double externalDesiredZ{0.0};

  double externalDesiredVx{0.0};
  double externalDesiredVy{0.0};
  double externalDesiredVz{0.0};

  double externalDesiredYaw{0.0};

  bool receivedExternalReference{false};

  gz::transport::Node::Publisher
    motorPublisher;

  std::string motorTopic;

  double lastPrintTime{-1.0};

  bool linkErrorPrinted{false};
};

}

GZ_ADD_PLUGIN(
  crazyflie_lqr_gz_plugin::CrazyflieLqrController,
  gz::sim::System,
  crazyflie_lqr_gz_plugin::CrazyflieLqrController::ISystemConfigure,
  crazyflie_lqr_gz_plugin::CrazyflieLqrController::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  crazyflie_lqr_gz_plugin::CrazyflieLqrController,
  "crazyflie_lqr_gz_plugin::CrazyflieLqrController"
)
