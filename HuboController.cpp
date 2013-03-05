/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author T. Kunz
 * @author Saul Reynolds-Haertle
 */

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
