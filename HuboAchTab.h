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
#include <map>
#include <cstdlib>
#include <inttypes.h>
#include <ctime>
#include <ach.h>

// DART, GRIP headers
#include <Tools/Constants.h>
#include <Tabs/GRIPTab.h>
#include <robotics/Robot.h>
#include <dynamics/BodyNodeDynamics.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <collision/CollisionSkeleton.h>

// HUBO headers
#include <hubo.h>

// local headers


namespace HACHT {
    // forward definition of HuboController becuase HC needs variables
    // from this file
    class HuboController;
    
    class HuboAchTab : public GRIPTab
    {
    public:
        //###########################################################
        // variables
        // pointer to the hubo we're controlling
        robotics::Robot* hubo;
        // the pid controller we'll use control things
        HuboController* contr;

        // pointer to particular links in hubo that we'll be doing
        // special things with
        dynamics::BodyNodeDynamics* hubo_waist;
        dynamics::BodyNodeDynamics* hubo_foot_left;
        dynamics::BodyNodeDynamics* hubo_foot_right;

        // map between joints in hubo ach messages and dofs in
        // simulated hubo
        std::map<int, int> jointmap_phys_to_virtual;
        std::map<int, int> jointmap_virtual_to_phys;


        // hubo parameters, including joint names
        hubo_param_t H_param;
        hubo_state_t H_state;

        // ach channels
        ach_channel_t chan_hubo_state; // for sending out state
        ach_channel_t chan_hubo_ref;   // for recieving reference positions
        
        //###########################################################
        // constructors and destructors
        HuboAchTab(){};
        HuboAchTab(wxWindow * parent,
                   wxWindowID id = -1,
                   const wxPoint & pos = wxDefaultPosition,
                   const wxSize & size = wxDefaultSize,
                   long style = wxTAB_TRAVERSAL);
        virtual ~HuboAchTab() {};

        //###########################################################
        // GRIPTab hooks
        void GRIPStateChange();
        void GRIPEventSceneLoaded();
        void GRIPEventSceneUnLoaded();
        void GRIPEventSimulationBeforeTimestep();
        void GRIPEventSimulationAfterTimestep();
        void GRIPEventRender();
        
        //###########################################################
        // HUBO emulation
        bool HuboInit();
        void ReadRefs();
        void WriteState();

        int FindNamedDof(std::string name);
        dynamics::BodyNodeDynamics* FindNamedNode(std::string name);
        void ComputeIMU(hubo_state_t& H_state);
        void ComputeFTs(hubo_state_t& H_state);
        Eigen::Vector3d ComputeForceFromParent(dynamics::BodyNodeDynamics* body);

        //###########################################################
        // wxwidgets UI event handlers
        void onButtonStartHUBO(wxCommandEvent & _evt);
        void onButtonStopHUBO(wxCommandEvent & _evt);

        //###########################################################
        // Cruft
        DECLARE_DYNAMIC_CLASS(HuboAchTab)
        DECLARE_EVENT_TABLE()
    };
}

// Local Variables:
// mode: c++
// End:
