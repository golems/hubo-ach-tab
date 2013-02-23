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
                                   double t_init) {
        skel = skeleton;
        Kp = p.asDiagonal();
        Ki = i.asDiagonal();
        Kd = d.asDiagonal();
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
        
        return torques;
        
        
        // // scale how much we use derivative of error versus current
        // // velocity
        // double L = 1.0;

        // // update errors
        // Eigen::VectorXd error = ref_pos - cur_pos;
        // error_deriv = (-cur_vel * L) + ((error_last - error) * (1.0-L) / dt);
        // error_integ += error * dt;
        // error_last = error;
        
        // // return result - trivial PID controller
        // Eigen::VectorXd result = Kp.array() * error.array()
        //     + Kd.array() * error_deriv.array()
        //     + Ki.array() * error_integ.array();

        // std::cout.precision(5);
        // std::cout << "time: " << t << std::endl;
        // std::cout << std::fixed << std::showpos;
        // std::cout.precision(1);
        // for (int i = 0; i < cur_pos.size(); i++) std::cout << cur_pos[i] << ","; std::cout << std::endl;
        // for (int i = 0; i < cur_vel.size(); i++) std::cout << cur_vel[i] << ","; std::cout << std::endl;
        // // for (int i = 0; i < error_last.size(); i++) std::cout << error_last[i] << ","; std::cout << std::endl;
        // // for (int i = 0; i < error.size(); i++) std::cout << error[i] << ","; std::cout << std::endl;
        // // for (int i = 0; i < error_deriv.size(); i++) std::cout << error_deriv[i] << ","; std::cout << std::endl;
        // // for (int i = 0; i < error_integ.size(); i++) std::cout << error_integ[i] << ","; std::cout << std::endl;
        // // for (int i = 0; i < cur_pos.size(); i++) std::cout << result[i] << ","; std::cout << std::endl;
        // for (int i = 0; i < result.size(); i++) std::cout << result[i] << ","; std::cout << std::endl;

        // return result;
    }
}
