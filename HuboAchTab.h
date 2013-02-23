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

// DART, GRIP headers
#include <Tools/Constants.h>
#include <Tabs/GRIPTab.h>
#include <robotics/Robot.h>

// HUBO headers
extern "C" {
#include <hubo.h>
}

// local headers


namespace HACHT {
    // forward definition of HuboController becuase HC needs variables
    // from this file
    class HuboController;
    
    // map from joints in hubo ach messages to joints in simulated
    // hubo
    static std::map<int, int> JOINT_TRANSLATION_MAP = {
        { RHY, 0 }, // Right Hip Yaw
        { RHR, 0 }, // Right Hip Roll
        { RHP, 0 }, // Right Hip Pitch
        { RKN, 0 }, // Right Knee Pitch
        { RAP, 0 }, // Right Ankle Pitch
        { RAR, 0 }, // Right Ankle Roll

        { LHY, 0 }, // Left Hip Yaw
        { LHR, 0 }, // Left Hip Roll
        { LHP, 0 }, // Left Hip Pitch
        { LKN, 0 }, // Left Knee Pitch
        { LAP, 0 }, // Left Ankle Pitch
        { LAR, 0 }, // Left Ankle Roll

        { RSP, 0 }, // Right Shoulder Pitch
        { RSR, 0 }, // Right Shoulder Roll
        { RSY, 0 }, // Right Shoulder Yaw
        { REB, 0 }, // Right Elbow Pitch
        { RWY, 0 }, // right wrist yaw
        { RWR, 0 }, // right wrist roll
        { RWP, 0 }, // right wrist Pitch

        { LSP, 0 }, // Left Shoulder Pitch
        { LSR, 0 }, // Left Shoulder Yaw
        { LSY, 0 }, // Left Shoulder Roll
        { LEB, 0 }, // Left Elbow Pitch
        { LWY, 0 }, // left wrist yaw
        { LWR, 0 }, // left wrist roll
        { LWP, 0 }, // left wrist pitch

        { NKY, 0 }, // neck yaw
        { NK1, 0 }, // neck 1
        { NK2, 0 }, // neck 2

        { WST, 0 }, // Trunk Yaw

        { RF1, 0 }, // Right Finger
        { RF2, 0 }, // Right Finger
        { RF3, 0 }, // Right Finger
        { RF4, 0 }, // Right Finger
        { RF5, 0 }, // Right Finger
        { LF1, 0 }, // Left Finger
        { LF2, 0 }, // Left Finger
        { LF3, 0 }, // Left Finger
        { LF4, 0 }, // Left Finger
        { LF5, 0 } // Left Finger
    };

    class HuboAchTab : public GRIPTab
    {
    public:
        //###########################################################
        // variables
        // pointer to the hubo we're controlling
        robotics::Robot* hubo;
        // the pid controller we'll use control things
        HuboController* contr;

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
        
        //###########################################################
        // HUBO emulation - core functions
        void HuboInit();
        void ReadRefs();
        void WriteState();

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
