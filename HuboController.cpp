// general headers
#include <iostream>

// DART, GRIP headers
#include <dynamics/BodyNodeDynamics.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/Dof.h>
#include <planning/Trajectory.h>
#include <utils/UtilsMath.h>

// HUBO headers
// #include "hubo.h"

// local headers
#include "HuboController.h"

using namespace std;
using namespace Eigen;

namespace HACHT {
    HuboController::HuboController(dynamics::SkeletonDynamics* _skel,
                                   const std::vector<int> &_actuatedDofs,
                                   const Eigen::VectorXd &_kP,
                                   const Eigen::VectorXd &_kD,
                                   const std::vector<int> &_ankleDofs,
                                   const Eigen::VectorXd &_anklePGains,
                                   const Eigen::VectorXd &_ankleDGains)
    {
    }

    Eigen::VectorXd HuboController::getTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, double _time) {
    }
}
