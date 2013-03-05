/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#pragma once

// general headers
#include <vector>
#include <Eigen/Core>

// dart/grip headers
#include <dynamics/SkeletonDynamics.h>

// hubo headers
// #include <hubo.h>

// local headers
#include "HuboAchTab.h"

namespace HACHT {
    class HuboController {
    public:
        //###########################################################
        // variables
        Eigen::VectorXd ref_pos;
        Eigen::VectorXd ref_vel;
        Eigen::MatrixXd Kp;
        Eigen::MatrixXd Kd;
        Eigen::MatrixXd Ki;

        Eigen::VectorXd error_last;
        Eigen::VectorXd error_deriv;
        Eigen::VectorXd error_integ;

        Eigen::MatrixXd joint_mask;

        double t_last;

        dynamics::SkeletonDynamics* skel;

        //###########################################################
        // constructors and destructors
        HuboController(dynamics::SkeletonDynamics* skeleton,
                       const Eigen::VectorXd p,
                       const Eigen::VectorXd i,
                       const Eigen::VectorXd d,
                       const Eigen::VectorXd mask,
                       double t_init);
        virtual ~HuboController() {};

        //###########################################################
        // Functions
        Eigen::VectorXd getTorques(const Eigen::VectorXd& cur_pos,
                                   const Eigen::VectorXd& cur_vel,
                                   double t);
    };
}

// Local Variables:
// mode: c++
// End:
