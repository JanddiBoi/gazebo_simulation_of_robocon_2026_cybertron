#pragma once
#include <string>

struct TransitionPose
{
  double x, y, yaw;       // 2D pose to hand AMCL on the new floor
  double fx, fy, fz;      // 3D pose used only by the sim teleport backend
};

class TransitionBackend
{
public:
  virtual ~TransitionBackend() = default;
  virtual void execute(
    const std::string & transition_id,
    int from_floor, int to_floor,
    const TransitionPose & target_pose) = 0;
};

// Gazebo Harmonic: teleports the model to the next floor's pose, simulating
// an elevator ride, via the `gz service` CLI against the world's set_pose
// service (provided by the UserCommands system plugin loaded by most worlds
// by default).
class SimTeleportBackend : public TransitionBackend
{
public:
  SimTeleportBackend(std::string world_name, std::string model_name);
  void execute(
    const std::string & transition_id,
    int from_floor, int to_floor,
    const TransitionPose & target_pose) override;

private:
  std::string world_name_;
  std::string model_name_;
};

// Placeholder for real deployment before you've picked a transition method.
// Blocks until an operator confirms the robot is physically on the new
// floor at the transition point.
class ManualConfirmBackend : public TransitionBackend
{
public:
  void execute(
    const std::string & transition_id,
    int from_floor, int to_floor,
    const TransitionPose & target_pose) override;
};