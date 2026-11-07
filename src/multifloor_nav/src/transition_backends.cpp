#include "multifloor_nav/transition_backends.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

SimTeleportBackend::SimTeleportBackend(std::string world_name, std::string model_name)
: world_name_(std::move(world_name)), model_name_(std::move(model_name)) {}

void SimTeleportBackend::execute(
  const std::string & /*transition_id*/, int /*from_floor*/, int /*to_floor*/,
  const TransitionPose & p)
{
  const double qz = std::sin(p.yaw / 2.0);
  const double qw = std::cos(p.yaw / 2.0);

  std::ostringstream req;
  req << "name: \"" << model_name_ << "\", "
      << "position: {x: " << p.fx << ", y: " << p.fy << ", z: " << p.fz << "}, "
      << "orientation: {x: 0, y: 0, z: " << qz << ", w: " << qw << "}";

  std::ostringstream cmd;
  cmd << "gz service -s /world/" << world_name_ << "/set_pose "
      << "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
      << "--req '" << req.str() << "'";

  std::system(cmd.str().c_str());
}

void ManualConfirmBackend::execute(
  const std::string & transition_id, int /*from_floor*/, int to_floor,
  const TransitionPose & /*target_pose*/)
{
  std::cout << "[ACTION NEEDED] Move robot to floor " << to_floor
            << " via '" << transition_id << "', then press Enter to continue..."
            << std::endl;
  std::cin.get();
}