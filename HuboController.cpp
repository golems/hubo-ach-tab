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
    //###########################################################
    // constructors and destructors
    HuboController::HuboController(dynamics::SkeletonDynamics* skeleton,
                                   const Eigen::VectorXd p,
                                   const Eigen::VectorXd i,
                                   const Eigen::VectorXd d,
                                   const Eigen::VectorXd mask,
                                   double t_init) {
        skel = skeleton;
        Kp = p.asDiagonal();
        Ki = i.asDiagonal();
        Kd = d.asDiagonal();
        joint_mask = mask.asDiagonal();
        t_last = t_init;
        error_last = VectorXd::Zero(skel->getNumDofs());
        error_deriv = VectorXd::Zero(skel->getNumDofs());
        error_integ = VectorXd::Zero(skel->getNumDofs());
        ref_vel = VectorXd::Zero(skel->getNumDofs());
        ref_pos = VectorXd::Zero(skel->getNumDofs());
    }
    
    //###########################################################
    // Functions
    Eigen::VectorXd HuboController::getTorques(const Eigen::VectorXd& cur_pos,
                                               const Eigen::VectorXd& cur_vel,
                                               double t) {
        // update time
        double dt = t - t_last;
        t_last = t;

        // SPD controller
        // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
        MatrixXd M = skel->getMassMatrix() + Kd * dt;
        MatrixXd invM = M.inverse();
        VectorXd p = -Kp * (cur_pos - ref_pos + cur_vel * dt);
        VectorXd d = -Kd * (cur_vel - ref_vel);
        VectorXd qddot = invM * (-skel->getCombinedVector() + p + d);
        VectorXd torques = p + d - Kd * qddot * dt;
        
        return joint_mask * torques;
    }
}
